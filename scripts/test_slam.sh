#!/bin/sh



cd ~/catkin_ws

catkin_make


# Start roscore
xterm -e "source /opt/ros/neotic/setup.bash;roscore" &
sleep 5

# Launch the TurtleBot in a simulated world
xterm -e "source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5

# Launch SLAM using gmapping
xterm -e "source devel/setup.bash; roslaunch gmapping slam_gmapping_pr2.launch" &
sleep 5


# Launch AMCL
xterm -e "source devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch"&
sleep 5

# Launch RViz for navigation visualization
xterm -e "source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

# Launch keyboard teleoperation for manual control
xterm -e "source devel/setup.bash; roslaunch turtlebot_teleop keyboard_teleop.launch"
