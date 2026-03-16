# VINS-Fusion_simulated-environment

1. Segue os passos da primeira pasta para clonar o VINS-FUSION.

2. Faça as alterações propostas na segunda pasta para utilizar o VINS-FUSION como uma biblioteca externa para projetos utilizando ros2 Humble no Ubuntu 22.04 e depois faça build.

3. Segue os passos da terceira pasta para clonar e buildar o ambiente da simulação.

4. Mova as pastas dentro de src/ para a pasta src/ dentro da frtl_2025_ws.

```bash
colcon build
```
5. Para testar: Run Task -> simulate -> fase1_25. Depois: Run Task -> agent.

Em todos os terminais:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source ~/vins_ws/install/setup.bash
```

Em um terminal:

```bash
ros2 run ros_gz_image image_bridge /camera/image_raw /camera/depth/image_raw
```

Em outro terminal:

```bash
ros2 run ros_gz_bridge parameter_bridge /world/fase1_25/model/x500_simulation_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU --ros-args -r /world/fase1_25/model/x500_simulation_0/link/base_link/sensor/imu_sensor/imu:=/imu0
```

Para rodar o VINS-FUSION:

```bash
ros2 launch vins_wrapper mono_imu.launch.py
```

Por fim:

```bash
ros2 launch cbr_fase1 simulation.launch.py
``` 

OBS: foi testado no dia 12/03/2026 e funcionou, entretanto cabe ressaltar que a angulação da câmera dentro da simulação não é favorável aos testes e talvez seja necessário utilizar outra fase para melhor teste.
