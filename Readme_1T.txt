	
	   /// PICO ///

--- MONTAR ---
lsusb
lsblk
mkdir -p ~/pico_mount
sudo mount /dev/sda1 ~/pico_mount

--- COPIAR ---
sudo cp pico_micro_ros_example.uf2 /home/leyla/pico_mount/

--- EJECUTAR CON DOCKER ---
DESCONECTAR LA PICO Y CONECTAR SIN BOOT Y CORRER DOCKER
sudo docker run -it --rm --net=host --privileged -v /dev:/dev microros/micro-ros-agent:jazzy serial --dev /dev/ttyACM0 -b 115200


	/// kinect_ws ///
	
colcon build 
spurce install/setup.bash
ros2 launch py_pkg ejecutar.py


	/// mpu_ws ///

colcon build 
spurce install/setup.bash
ros2 run imu_publisher imu


