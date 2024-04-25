# CMake generated Testfile for 
# Source directory: /home/husarion/catkin_ws/src/ros_comm/tools/rosgraph
# Build directory: /home/husarion/catkin_ws/build/rosgraph
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_rosgraph_nosetests_test "/home/husarion/catkin_ws/build/rosgraph/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/rosgraph/test_results/rosgraph/nosetests-test.xml" "--return-code" "\"/usr/bin/cmake\" -E make_directory /home/husarion/catkin_ws/build/rosgraph/test_results/rosgraph" "/usr/bin/nosetests-2.7 -P --process-timeout=60 --where=/home/husarion/catkin_ws/src/ros_comm/tools/rosgraph/test --with-xunit --xunit-file=/home/husarion/catkin_ws/build/rosgraph/test_results/rosgraph/nosetests-test.xml")
subdirs("gtest")
