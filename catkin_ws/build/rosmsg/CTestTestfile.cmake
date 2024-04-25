# CMake generated Testfile for 
# Source directory: /home/husarion/catkin_ws/src/ros_comm/tools/rosmsg
# Build directory: /home/husarion/catkin_ws/build/rosmsg
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_rosmsg_nosetests_test "/home/husarion/catkin_ws/build/rosmsg/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/rosmsg/test_results/rosmsg/nosetests-test.xml" "--return-code" "\"/usr/bin/cmake\" -E make_directory /home/husarion/catkin_ws/build/rosmsg/test_results/rosmsg" "/usr/bin/nosetests-2.7 -P --process-timeout=60 --where=/home/husarion/catkin_ws/src/ros_comm/tools/rosmsg/test --with-xunit --xunit-file=/home/husarion/catkin_ws/build/rosmsg/test_results/rosmsg/nosetests-test.xml")
subdirs("gtest")
