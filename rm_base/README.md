# scu_rm_ros
2022 scu_rm_ros code
rm_base 通信结点

# 环境搭建
source /opt/ros/galactic/setup.bash
mkdir -p ~/scu_rm_ros/src
cd ~/scu_rm_ros
colcon build

cd ~/scu_rm_ros/src
ros2 pkg create --build-type ament_cmake rm_base
ros2 pkg create --build-type ament_cmake rm_interfaces

把src中的同名包（rm_base、rm_interfaces）替换为提供的同名的包

cd ~/scu_rm_ros
. install/local_setup.bash
colcon build 


## 16位包格式:包头 cmd控制(默认为0xa1) mode模式切换（0x00正常模式上位机不发送，0x01自瞄模式发送数据） pitch、yaw（float32:4字节） BCC校验位 包尾
HEAD CMD  MODE      PITCH YAW        bcc TAIL
0xff 0x01 0x00/0x01 3-6   7-10 11-13 14  0x0d
eg. 0xff 0x01 0x01 (2.0) (3.0) 0x00 0x00 0x00 BCC校验位 0x0d
下位机只需要上位机（miniPC）提供pitch(3-6位)、yaw(7-10位)，加速度或者其他参数需要进一步改进；
上位机只需要下位机（STM32）cmd控制与mode自瞄/正常模式切换

## 在launch文件中修改node参数
serial_name：使用的串口名，serial_send：串口发送，serial_recv：串口接收
·默认           {"serial_name": "/dev/USBtty0"},
                {"serial_send": False},
                {"serial_recv": True}
  表示上位机仅接收下位机数据，此时mode=0x00；当mode=0x01表示开启自瞄时，程序开启发送模块serial_send=True，发送自瞄需要的数据
·Debug测试      {"serial_name": "/dev/USBtty0"},
                {"serial_send": True},
                {"serial_recv": True}
  开启接收发送

# 测试
结点启动终端：
ros2 launch launch/serial_test_launch.py
（另起一个终端）测试终端，模拟自瞄结点广播topic：
ros2 topic pub /cmd_gimbal rm_interfaces/msg/GimbalCmd "{position: {pitch: 3.0, yaw: 2.0}}"



## run指令结点建立（未完成）
ros2 run rm_base simple_base_node --ros-args --remap __node:=【结点名】
收发一体结点，
  发：接收topic：名字（/cmd_gimbal）, 类型msg（GimbalCmd）.position.yaw/pitch ，发送到串口
  收：接收串口，mode=0x00正常模式/0x01自瞄模式1
