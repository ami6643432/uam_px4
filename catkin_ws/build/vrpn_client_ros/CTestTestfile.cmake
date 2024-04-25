# CMake generated Testfile for 
# Source directory: /home/husarion/catkin_ws/src/vrpn_client_ros
# Build directory: /home/husarion/catkin_ws/build/vrpn_client_ros
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_vrpn_client_ros_roslint_package "/home/husarion/catkin_ws/build/vrpn_client_ros/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/vrpn_client_ros/test_results/vrpn_client_ros/roslint-vrpn_client_ros.xml" "--working-dir" "/home/husarion/catkin_ws/build/vrpn_client_ros" "--return-code" "/opt/ros/melodic/share/roslint/cmake/../../../lib/roslint/test_wrapper /home/husarion/catkin_ws/build/vrpn_client_ros/test_results/vrpn_client_ros/roslint-vrpn_client_ros.xml make roslint_vrpn_client_ros")
add_test(_ctest_vrpn_client_ros_roslaunch-check_launch "/home/husarion/catkin_ws/build/vrpn_client_ros/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/vrpn_client_ros/test_results/vrpn_client_ros/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/husarion/catkin_ws/build/vrpn_client_ros/test_results/vrpn_client_ros" "/home/husarion/catkin_ws/src/ros_comm/tools/roslaunch/scripts/roslaunch-check -o \"/home/husarion/catkin_ws/build/vrpn_client_ros/test_results/vrpn_client_ros/roslaunch-check_launch.xml\" \"/home/husarion/catkin_ws/src/vrpn_client_ros/launch\" ")
subdirs("gtest")
