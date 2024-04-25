# CMake generated Testfile for 
# Source directory: /home/husarion/catkin_ws/src/ros_comm/test/test_rostopic
# Build directory: /home/husarion/catkin_ws/build/test_rostopic
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_test_rostopic_nosetests_test "/home/husarion/catkin_ws/build/test_rostopic/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/test_rostopic/test_results/test_rostopic/nosetests-test.xml" "--return-code" "\"/usr/bin/cmake\" -E make_directory /home/husarion/catkin_ws/build/test_rostopic/test_results/test_rostopic" "/usr/bin/nosetests-2.7 -P --process-timeout=60 --where=/home/husarion/catkin_ws/src/ros_comm/test/test_rostopic/test --with-xunit --xunit-file=/home/husarion/catkin_ws/build/test_rostopic/test_results/test_rostopic/nosetests-test.xml")
subdirs("gtest")
