#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

class SlamToDrone : public rclcpp::Node
{
public:
    SlamToDrone() : Node("slam_to_drone")
    {
        // Parâmetros configuráveis
        this->declare_parameter<std::string>("slam_topic", "/odometry");
        this->declare_parameter<std::string>("slam_msg_type", "odometry"); // "odometry" ou "pose_stamped"
        this->declare_parameter<std::string>("output_topic", "/fmu/in/vehicle_visual_odometry");
        this->declare_parameter<std::string>("frame_id", "map");
        this->declare_parameter<std::string>("child_frame_id", "base_link");

        this->get_parameter("slam_topic", slam_topic_);
        this->get_parameter("slam_msg_type", slam_msg_type_);
        this->get_parameter("output_topic", output_topic_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("child_frame_id", child_frame_id_);

        // Publisher para PX4
        odom_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(output_topic_, 10);

        // Subscriber depende do tipo de mensagem do SLAM
        if (slam_msg_type_ == "pose_stamped")
        {
            pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                slam_topic_, 10,
                std::bind(&SlamToDrone::pose_callback, this, std::placeholders::_1));
            RCLCPP_INFO(get_logger(), "Subscribing to PoseStamped on: %s", slam_topic_.c_str());
        }
        else  // odometry (default)
        {
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                slam_topic_, 10,
                std::bind(&SlamToDrone::odom_callback, this, std::placeholders::_1));
            RCLCPP_INFO(get_logger(), "Subscribing to Odometry on: %s", slam_topic_.c_str());
        }

        RCLCPP_INFO(get_logger(), "SlamToDrone initialized | output: %s", output_topic_.c_str());
    }

private:
    // Converte timestamp ROS para microsegundos PX4
    uint64_t to_px4_timestamp(const rclcpp::Time & t)
    {
        return static_cast<uint64_t>(t.nanoseconds() / 1000);
    }

    // Preenche e publica VehicleOdometry a partir de posição + quaternion
    void publish_px4_odom(
        const rclcpp::Time & stamp,
        const geometry_msgs::msg::Point & pos,
        const geometry_msgs::msg::Quaternion & q,
        const geometry_msgs::msg::Vector3 & vel = geometry_msgs::msg::Vector3(),
        const geometry_msgs::msg::Vector3 & ang_vel = geometry_msgs::msg::Vector3())
    {
        px4_msgs::msg::VehicleOdometry px4_odom;

        px4_odom.timestamp        = to_px4_timestamp(stamp);
        px4_odom.timestamp_sample = px4_odom.timestamp;

        // Frame: ENU (ROS) → NED (PX4)
        // x_ned =  y_enu, y_ned =  x_enu, z_ned = -z_enu
        px4_odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        px4_odom.position[0] =  static_cast<float>(pos.y);
        px4_odom.position[1] =  static_cast<float>(pos.x);
        px4_odom.position[2] = -static_cast<float>(pos.z);

        // Quaternion ENU → NED
        px4_odom.q[0] =  static_cast<float>(q.w);
        px4_odom.q[1] =  static_cast<float>(q.y);
        px4_odom.q[2] =  static_cast<float>(q.x);
        px4_odom.q[3] = -static_cast<float>(q.z);

        // Velocidade linear ENU → NED
        px4_odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
        px4_odom.velocity[0] =  static_cast<float>(vel.y);
        px4_odom.velocity[1] =  static_cast<float>(vel.x);
        px4_odom.velocity[2] = -static_cast<float>(vel.z);

        // Velocidade angular (body frame, mesmo nos dois)
        px4_odom.angular_velocity[0] = static_cast<float>(ang_vel.x);
        px4_odom.angular_velocity[1] = static_cast<float>(ang_vel.y);
        px4_odom.angular_velocity[2] = static_cast<float>(ang_vel.z);

        // Variâncias (confiança da estimativa)
        px4_odom.position_variance    = {0.01f, 0.01f, 0.01f};
        px4_odom.orientation_variance = {0.01f, 0.01f, 0.01f};
        px4_odom.velocity_variance    = {0.01f, 0.01f, 0.01f};

        px4_odom.reset_counter = 0;
        px4_odom.quality       = 100;  // 0-100, confiança máxima

        odom_pub_->publish(px4_odom);
    }

    // Callback para ORB-SLAM3 (PoseStamped) — sem velocidade
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        publish_px4_odom(
            msg->header.stamp,
            msg->pose.position,
            msg->pose.orientation);
    }

    // Callback para VINS-Fusion e Kimera (Odometry) — com velocidade
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        publish_px4_odom(
            msg->header.stamp,
            msg->pose.pose.position,
            msg->pose.pose.orientation,
            msg->twist.twist.linear,
            msg->twist.twist.angular);
    }

    std::string slam_topic_, slam_msg_type_, output_topic_, frame_id_, child_frame_id_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlamToDrone>());
    rclcpp::shutdown();
    return 0;
}