# CMake generated Testfile for 
# Source directory: /home/husarion/catkin_ws/src/ros_comm/tools/rosnode
# Build directory: /home/husarion/catkin_ws/build/rosnode
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_rosnode_rostest_test_rosnode.test "/home/husarion/catkin_ws/build/rosnode/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/rosnode/test_results/rosnode/rostest-test_rosnode.xml" "--return-code" "/usr/bin/python2 /home/husarion/catkin_ws/src/ros_comm/tools/rostest/scripts/rostest --pkgdir=/home/husarion/catkin_ws/src/ros_comm/tools/rosnode --package=rosnode --results-filename test_rosnode.xml --results-base-dir \"/home/husarion/catkin_ws/build/rosnode/test_results\" /home/husarion/catkin_ws/src/ros_comm/tools/rosnode/test/rosnode.test ")
subdirs("gtest")
