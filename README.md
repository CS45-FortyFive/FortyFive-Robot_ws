Following Instructions adapted from TurtleBot3 Manual. For more detailed explanation:
http://emanual.robotis.com/docs/en/platform/turtlebot3/

### 1.1 Install Ubuntu on Remote PC
Download and install the Ubuntu 16.04 on the Robot PC and Remote PC from the following link.

https://www.ubuntu.com/download/alternative-downloads

### 1.2 Install ROS on Remote PC
Run the following command in a terminal window.
```
$ sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh && chmod 755 ./install_ros_kinetic.sh && bash ./install_ros_kinetic.sh
```

### 1.3 Install Dependent ROS Packages
```
$ sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs

$ cd ~
$ git clone https://github.com/CS45-FortyFive/FortyFive-Robot_ws.git
$ cd FortyFive-Robot_ws
$ catkin_make
$ echo "source ~/FortyFive-Robot_ws/devel/setup.bash" >> ~/.bashrc # Adds workspace to search path
```

### 1.4 Network Configuration
ROS 1 requires IP addresses in order to communicate between Robot PC and the remote PC. The remote PC and TurtleBot PC should be connected to the same wifi router.

Run the following command in a terminal window on remote PC to find out the IP address.
`ifconfig`

After that in /.bashrc edit address of localhost in the ROS_MASTER_URI and ROS_HOSTNAME with the IP address acquired from the terminal window.

For Simulation on Local Machine
```
$ echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
$ echo "export ROS_HOSTNAME=localhost" >> ~/.bashrc
$ echo "export ROS_IP=localhost" >> ~/.bashrc
$ echo "export FORTYFIVE_ROBOT_MODEL=waffle" >> ~/.bashrc
```
When you are done, do not forget to source ~/.bashrc.

### 2.1 Bringup The Robot
On your remote PC run following command on your terminal
```
roscore
```

For Simulation on Local computer
```
$ roslaunch fortyfive_robot_bringup fortyfive_robot_fake.launch
```

On robot PC run following command on your terminal
```
$ roslaunch fortyfive_robot_bringup fortyfive_robot_robot.launch
```

You Should see the following output on your terminal
```
SUMMARY
========

PARAMETERS
 * /rosdistro: kinetic
 * /rosversion: 1.12.13
 * /fortyfive_robot_core/baud: 115200
 * /fortyfive_robot_core/port: /dev/ttyACM0
 * /fortyfive_robot_core/tf_prefix:
 * /fortyfive_robot_lds/frame_id: base_scan
 * /fortyfive_robot_lds/port: /dev/ttyUSB0

NODES
  /
    fortyfive_robot_core (rosserial_python/serial_node.py)
    fortyfive_robot_diagnostics (fortyfive_robot_bringup/fortyfive_robot_diagnostics)
    fortyfive_robot_lds (hls_lfcd_lds_driver/hlds_laser_publisher)

ROS_MASTER_URI=http://192.168.1.2:11311

process[fortyfive_robot_core-1]: started with pid [14198]
process[fortyfive_robot_lds-2]: started with pid [14199]
process[fortyfive_robot_diagnostics-3]: started with pid [14200]
[INFO] [1531306690.947198]: ROS Serial Python Node
[INFO] [1531306691.000143]: Connecting to /dev/ttyACM0 at 115200 baud
[INFO] [1531306693.522019]: Note: publish buffer size is 1024 bytes
[INFO] [1531306693.525615]: Setup publisher on sensor_state [fortyfive_robot_msgs/SensorState]
[INFO] [1531306693.544159]: Setup publisher on version_info [fortyfive_robot_msgs/VersionInfo]
[INFO] [1531306693.620722]: Setup publisher on imu [sensor_msgs/Imu]
[INFO] [1531306693.642319]: Setup publisher on cmd_vel_rc100 [geometry_msgs/Twist]
[INFO] [1531306693.687786]: Setup publisher on odom [nav_msgs/Odometry]
[INFO] [1531306693.706260]: Setup publisher on joint_states [sensor_msgs/JointState]
[INFO] [1531306693.722754]: Setup publisher on battery_state [sensor_msgs/BatteryState]
[INFO] [1531306693.759059]: Setup publisher on magnetic_field [sensor_msgs/MagneticField]
[INFO] [1531306695.979057]: Setup publisher on /tf [tf/tfMessage]
[INFO] [1531306696.007135]: Note: subscribe buffer size is 1024 bytes
[INFO] [1531306696.009083]: Setup subscriber on cmd_vel [geometry_msgs/Twist]
[INFO] [1531306696.040047]: Setup subscriber on sound [fortyfive_robot_msgs/Sound]
[INFO] [1531306696.069571]: Setup subscriber on motor_power [std_msgs/Bool]
[INFO] [1531306696.096364]: Setup subscriber on reset [std_msgs/Empty]
[INFO] [1531306696.390979]: Setup TF on Odometry [odom]
[INFO] [1531306696.394314]: Setup TF on IMU [imu_link]
[INFO] [1531306696.397498]: Setup TF on MagneticField [mag_link]
[INFO] [1531306696.400537]: Setup TF on JointState [base_link]
[INFO] [1531306696.407813]: --------------------------
[INFO] [1531306696.411412]: Connected to OpenCR board!
[INFO] [1531306696.415140]: This core(v1.2.1) is compatible with TB3 Burger
[INFO] [1531306696.418398]: --------------------------
[INFO] [1531306696.421749]: Start Calibration of Gyro
[INFO] [1531306698.953226]: Calibration End
```
### 2.2 Bringup Gazebo World (For Simulation)
This is to simulate a world in Gazebo 
```
$ export FORTYFIVE_ROBOT_MODEL=waffle
$ roslaunch fortyfive_robot_gazebo fortyfive_robot_world.launch
```

### 3.1 Run SLAM Nodes
On your Remote PC open a new terminal and run following commands.
```
$ export FORTYFIVE_ROBOT_MODEL=waffle
$ roslaunch fortyfive_robot_slam fortyfive_robot_slam.launch slam_methods:=gmapping
```

The slam_methods options include gmapping, cartographer, hector, karto, frontier_exploration, and you can choose one of them. But for our project we will stick with gmapping.

### 3.2 Control Robot over Terminal
On your Remote PC open a new terminal and run following commands.
```
$ export FORTYFIVE_ROBOT_MODEL=${TB3_MODEL}
$ roslaunch fortyfive_robot_teleop fortyfive_robot_teleop_key.launch
```

You should see following output:
```
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, s : force stop

CTRL-C to quit
```
Now you can control the robot with your keyboard. Press W to move Forward. X to go backwards. A to turn left and D to turn right. Pressing S will stop the movement of the Robot.

### 3.3 Save SLAM Map
On your Remote PC run following command to save the map created by gmapping.

```
$ rosrun map_server map_saver -f ~/map
```

### 4.1 Navigate Robot on Created map
On your remote PC run following command on your terminal
`roscore`

On robot PC run following command on your terminal
```
$ roslaunch fortyfive_robot_bringup fortyfive_robot_robot.launch
```

On your remote PC run following command on your terminal:
```
$ export FORTYFIVE_ROBOT_MODEL=${TB3_MODEL}
$ roslaunch fortyfive_robot_navigation fortyfive_robot_navigation.launch map_file:=$HOME/map.yaml
```

### 4.2 Set Navigation Goal
On your Remote PC. If you press 2D Nav Goal in the menu of RViz, a very large green arrow appears. This green arrow is a marker that can specify the destination of the robot. The root of the arrow is the x and y position of the robot, and the orientation pointed by the arrow is the theta direction of the robot. Click this arrow at the position where the robot will move, and drag it to set the orientation like the instruction below.
Click the 2D Nav Goal button.
Click on a specific point in the map to set a goal position and drag the cursor to the direction where TurtleBot should be facing at the end.
The robot will create a path to avoid obstacles to its destination based on the map. Then, the robot moves along the path. At this time, even if an obstacle is suddenly detected, the robot moves to the target point avoiding the obstacle.

### 5.1 Install RealSense ROS Package:

Install Prerequisites:
```
$ wget -O enable_kernel_sources.sh http://bit.ly/en_krnl_src
$ bash ./enable_kernel_sources.sh
```

Install Sensor Packages:
```
$ sudo apt install ros-kinetic-librealsense ros-kinetic-realsense-camera
$ sudo reboot
```

Install Kernel 4.10 work-around:
```
$ sudo apt-get install libglfw3-dev
$ cd ~
$ git clone https://github.com/IntelRealSense/librealsense.git
$ cd librealsense
$ mkdir build && cd build
$ cmake ../
$ make && sudo make install
$ cd ..
$ sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
$ sudo udevadm control –reload-rules && udevadm trigger
$ ./scripts/patch-realsense-ubuntu-xenial.sh

```

### 5.2 Install Intel® RealSense™ ROS from Sources
```
$ cd ~/catkin_ws/src/
```
Clone the latest Intel® RealSense™ ROS into 'catkin_ws/src/'
```
$ git clone https://github.com/IntelRealSense/realsense-ros.git
$ cd realsense-ros/
$ git checkout `git tag | sort -V | grep -P "^\d+\.\d+\.\d+" | tail -1`
$ cd ..
```

```
$ catkin_init_workspace
$ cd ..
$ catkin_make clean
$ catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
$ catkin_make install
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

### 5.2 Start Camera
On the Robot run following command:
```
$ roslaunch realsense2_camera rs_camera.launch
```

### 6.1 Assemble WidowX MKII Robot Arm
Following Instructions adapted from TurtleBot3 Manual. For more detailed explanation please visit:
https://widowx-arm.readthedocs.io/en/latest/index.html
Before installing arm dependencies, carefully assemble the robotic arm by following the directions in this link.
```
http://www.trossenrobotics.com/productdocs/assemblyguides/widowx-robot-arm-mk2.html
```
### 6.3 Install WidowX Arm Dependancies
```
$ sudo apt install git htop
$ sudo apt install ros-kinetic-moveit ros-kinetic-pcl-ros
```

Set dialout permission for Arbotix:
yourUserAccount is the name of your pc
```
$ sudo usermod -a -G dialout yourUserAccount
```
Then Restart your computer:
```
sudo reboot
```
Clone Widowx Arm repository and build:
```
$ mkdir -p ~/widowx_arm/src
$ cd ~/widowx_arm/src
$ git clone https://github.com/Interbotix/widowx_arm.git .
$ git clone https://github.com/Interbotix/arbotix_ros.git -b parallel_gripper
$ cd ~/widowx_arm
$ catkin_make
```

Test execution without additional sensors:
```
$ cd ~/widowx_arm
$ source devel/setup.bash
$ roslaunch widowx_arm_bringup arm_moveit.launch sim:=false sr300:=false```
```
### 6.3 Install ROS Arm Dependancies
On your Remote PC run following commands on your terminal to download and build the OpenMANIPULATOR-X package.

```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations.git
$ cd ~/catkin_ws && catkin_make
```

### 7.1 Operate the Arm
On your remote PC run following command on your terminal:
```
$ roscore
```
On the Robot PC run following command on your terminal:
```
$ roslaunch fortyfive_robot_bringup fortyfive_robot_robot.launch
```
On the Remote PC run following command on your terminal:
```
$ roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
```

Now you can control the Robot Arm, using GUI
```
$ roslaunch turtlebot3_manipulation_gui turtlebot3_manipulation_gui.launch
```
or You can use Rviz

```
$ roslaunch turtlebot3_manipulation_moveit_config moveit_rviz.launch
```
