ROS Jazzy:
===============
cpp_pubsub
===============
ТЕРМИНАЛ 1:
ros2 run cpp_pubsub talker

ТЕРМИНАЛ 2:
ros2 run cpp_pubsub listener


robot_control
===============
#Для запуска алгоритма жука
ТЕРМИНАЛ 1:
ros2 launch robot_control bug.launch.py

#Для запуска алгоритма движения вдоль стены
ТЕРМИНАЛ 1:
ros2 launch robot_control wall.launch.py


pid_control
===============
ТЕРМИНАЛ 1:
ros2 launch pid_control control.launch.py task:=line #Траекторией может быть line, circle и oval

#Для отображения траектории движения внутри окна stage нажимает ctrl+A

control_selector
===============
ТЕРМИНАЛ 1:
ros2 launch control_selector control.launch.py #Для запуска окна Stage

ТЕРМИНАЛ 2:
ros2 run control_selector interactive_selector_node #Для запуска модуля управления

simple_map
===============
ТЕРМИНАЛ 1:
ros2 launch simple_map simple_map.launch.py



ROS Noetic && ROS Foxy:
===============
#.bashrc
alias sr1='source /opt/ros/noetic/setup.bash; source ~/ros1_ws/devel/setup.sh'
alias sr2='source /opt/ros/foxy/setup.bash; source ~/ros2_ws/install/setup.sh'

patrol_bot
===============

ТЕРМИНАЛ 1:

sr1

roscore

ТЕРМИНАЛ 2:

sr1

sr2

ros2 run ros1_bridge dynamic_bridge

ТЕРМИНАЛ 3:

sr1 

roslaunch navigation navi.launch

ТЕРМИНАЛ 4:

sr2

ros2 run patrol_bot patrol_bot_node
