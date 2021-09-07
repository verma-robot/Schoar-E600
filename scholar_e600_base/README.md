
# 文件结构

 - scholar_base_bringup : Scholar E600移动机器人驱动包
 - scholar_base_calibrate : 线速度、角速度校准包
 - scholar_base_control : ros control配置文件
 - scholar_base_description : Scholar E600移动机器人base的URDF文件. 
 - scholar_base_gazebo : Scholar E600移动机器人base的Gazebo配置文件
 - scholar_imu_bringup : Scholar E600内置IMU的ROS驱动包
 - scholar_joy_teleop : Scholar E600键盘控制

# 安装

## 按照如下步骤安装、编译 :

  ```
  cd ~/catkin_ws/src
  git clone https://github.com/verma-robot/RosHand.git scholar-ros
  cd ~/catkin_ws
  catkin_make

  ```
## 修改Scholar E600 串口权限
  copy the udev rule file `scholar_600.rules` from `~/catkin_ws/src/scholar-ros/scholar_base_bringup/udev` to `/etc/udev/rules.d/`:

  ```
  cd ~/catkin_ws/src/scholar-ros
  sudo cp scholar_base_bringup/udev/scholar_600.rules /etc/udev/rules.d/
  
  ```
## 修改Scholar E600 内置IMU的串口权限
  copy the udev rule file `imu.rules` from `~/catkin_ws/src/scholar-ros/scholar_imu_bringup/udev` to `/etc/udev/rules.d/`:

  ```
  cd ~/catkin_ws/src/scholar-ros
  sudo cp scholar_imu_bringup/udev/imu.rules /etc/udev/rules.d/
  
  ```
# 控制机器人(real robot)

## 启动机器人

### 启动Scholar E600

  ```
  roslaunch scholar_base_bringup bringup.launch 

  ```
### 启动Scholar E600内置IMU

  ```
  roslaunch scholar_imu_bringup bringup.launch 

  ```
## 键盘控制机器人

  ```
  roslaunch scholar_joy_teleop scholar_teleop.launch 

  ```

## 校准机器人线速度、角速度

### 校准线速度：

  ```
  rosrun scholar_base_calibrate calibrate_linear.py 

  ```

### 校准角速度：

  ```
  rosrun scholar_base_calibrate angular_calibrate.py 

  ```


### rviz可视化

  ```

   roslaunch scholar_rviz view_robot.launch 

  ```

#  Gazebo仿真

## 启动机器人（Gazebo）

  ```
  roslaunch scholar_base_gazebo scholar_base_gazebo.launch

  ```

## rviz可视化

  ```

   roslaunch scholar_rviz view_robot.launch 

  ```


## Report a Bug
  Any bugs, issues or suggestions may be sent to support@verma-robot.com

  Thanks!
