source /opt/ros/melodic/setup.bash
source ~/vision_ws/devel/setup.bash
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch zoo_robot robot_lidar.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch robot_slam navigation.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch match_mini auto_slam.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch robot_slam view_nav.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch match_mini voice_controller.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch robot_vision usb_cam.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch robot_vision apriltag_detector.launch; exec bash"' \
