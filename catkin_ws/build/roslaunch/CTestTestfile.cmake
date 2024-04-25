# CMake generated Testfile for 
# Source directory: /home/husarion/catkin_ws/src/ros_comm/tools/roslaunch
# Build directory: /home/husarion/catkin_ws/build/roslaunch
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_roslaunch_nosetests_test.unit "/home/husarion/catkin_ws/build/roslaunch/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/roslaunch/test_results/roslaunch/nosetests-test.unit.xml" "--return-code" "\"/usr/bin/cmake\" -E make_directory /home/husarion/catkin_ws/build/roslaunch/test_results/roslaunch" "/usr/bin/nosetests-2.7 -P --process-timeout=60 --where=/home/husarion/catkin_ws/src/ros_comm/tools/roslaunch/test/unit --with-xunit --xunit-file=/home/husarion/catkin_ws/build/roslaunch/test_results/roslaunch/nosetests-test.unit.xml")
subdirs("gtest")
