#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <queue>
#include <thread>
#include <mutex>

#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"

Estimator estimator;

std::queue<sensor_msgs::msg::Image::ConstSharedPtr> img_buf;
std::mutex m_buf;

class VinsWrapper : public rclcpp::Node
{
public:

    VinsWrapper() : Node("vins_wrapper")
    {
        this->declare_parameter<std::string>("config_file", "");
        this->get_parameter("config_file", config_file_);

        if(config_file_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "config_file parameter not set");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Using config file: %s", config_file_.c_str());

        // Usar timer de um disparo para chamar init após o nó estar completamente construído
        // Isso garante que shared_from_this() funciona corretamente
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(0),
            [this]() {
                init_timer_->cancel();
                this->postConstruct();
            }
        );
    }

private:

    void postConstruct()
    {
        // Agora shared_from_this() é seguro de usar
        registerPub(shared_from_this());

        readParameters(config_file_);
        estimator.setParameter();

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            IMU_TOPIC,
            rclcpp::SensorDataQoS(),
            std::bind(&VinsWrapper::imuCallback, this, std::placeholders::_1)
        );

        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            IMAGE0_TOPIC,
            rclcpp::SensorDataQoS(),
            std::bind(&VinsWrapper::imgCallback, this, std::placeholders::_1)
        );

        sync_thread_ = std::thread(&VinsWrapper::syncProcess, this);

        RCLCPP_INFO(this->get_logger(), "VINS wrapper initialized");
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double t = msg->header.stamp.sec +
                   msg->header.stamp.nanosec * 1e-9;

        Eigen::Vector3d acc(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z
        );

        Eigen::Vector3d gyr(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z
        );

        estimator.inputIMU(t, acc, gyr);
    }

    void imgCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(m_buf);
        img_buf.push(msg);
    }

    cv::Mat getImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        cv_bridge::CvImageConstPtr ptr;

        if(msg->encoding == "8UC1")
        {
            sensor_msgs::msg::Image img;

            img.header = msg->header;
            img.height = msg->height;
            img.width = msg->width;
            img.is_bigendian = msg->is_bigendian;
            img.step = msg->step;
            img.data = msg->data;
            img.encoding = "mono8";

            ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        }
        else
        {
            ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        }

        return ptr->image.clone();
    }

    void syncProcess()
    {
        while(rclcpp::ok())
        {
            cv::Mat image;
            double time = 0;

            {
                std::lock_guard<std::mutex> lock(m_buf);

                if(!img_buf.empty())
                {
                    auto msg = img_buf.front();
                    img_buf.pop();

                    time = msg->header.stamp.sec +
                           msg->header.stamp.nanosec * 1e-9;

                    image = getImage(msg);
                }
            }

            if(!image.empty())
            {
                estimator.inputImage(time, image);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    std::string config_file_;

    rclcpp::TimerBase::SharedPtr init_timer_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

    std::thread sync_thread_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<VinsWrapper>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}