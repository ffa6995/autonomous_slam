# Set burger
export TURTLEBOT3_MODEL=burger
# Start Ros on PC
roscore
# on bot
# Set burger
export TURTLEBOT3_MODEL=burger
# start bot
roslaunch turtlebot3_bringup turtlebot3_robot.launch
# start raspicam
roslaunch raspicam_node camerav2_410x308_30fps.launch

#for slam see:
#https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#run-slam-node






# Für SLAM Automatisch:
# Set burger
export TURTLEBOT3_MODEL=burger
# Start Ros on PC
roscore
# on bot
# Set burger
export TURTLEBOT3_MODEL=burger
# start bot
roslaunch turtlebot3_bringup turtlebot3_robot.launch
# start raspicam
roslaunch raspicam_node camerav2_410x308_30fps.launch
# pixcam
roslaunch  pixy_node pixy_usb.launch
# topic is /my_pixy/block_data

# ON PC
roslaunch turtlebot3_slam turtlebot3_slam.launch
# start explore
roslaunch explore_lite explore.launch
# Start Move Base
roslaunch turtlebot3_navigation move_base.launch 
# add in rvic global_plan Path

# PIXY FINDER
roslaunch rosbulla_navigate pixyfinder.launch
# Drive to COrds
roslaunch rosbulla_navigate coord_movebase.launch
# Save Current Possition
roslaunch rosbulla_navigate coord_saver.launch
# Save Coord
rostopic pub /coord_saver std_msgs/Int32 "data: 0"


/tmp/my_map.yaml

roslaunch turtlebot3_bringup turtlebot3_remote.launch
rosrun map_server map_server /tmp/my_map.yaml
roslaunch turtlebot3_navigation amcl.launch
roslaunch turtlebot3_navigation move_base.launch 


https://github.com/dheera/rosboard
http://robotwebtools.org/ 