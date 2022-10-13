# multirotor_ws
中国机器人大赛无人机挑战赛实物组
## 命令列表
1.加载代码环境

* cd ~/multirotor_ws/
* source ./devel/setup.bash

2.px4&雷达 通信

* roslaunch mavros px4.launch

* roslaunch rplidar_ros rplidar.launch

3.无人机通信

* rosrun init_control multirotor_communication.py

4.键盘控制

* rosrun init_control multirotor_keyboard_control.py

5.起飞&悬停

* rosrun init_control my_takeoff 0.5 1.5
    >(0.5 为起飞速度 1.5 为起飞高度)

* rosrun init_control offb_node

6.导航

* roslaunch my_navigation my_cartographer.launch
* rosrun my_navigation laser_transfer.py

7.路径规划

* roslaunch my_motion_planning 2d_motion_planning.launch
