# CMake generated Testfile for 
# Source directory: /home/husarion/catkin_ws/src/ros_comm/tools/rosmaster
# Build directory: /home/husarion/catkin_ws/build/rosmaster
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_rosmaster_nosetests_test "/home/husarion/catkin_ws/build/rosmaster/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/rosmaster/test_results/rosmaster/nosetests-test.xml" "--return-code" "\"/usr/bin/cmake\" -E make_directory /home/husarion/catkin_ws/build/rosmaster/test_results/rosmaster" "/usr/bin/nosetests-2.7 -P --process-timeout=60 --where=/home/husarion/catkin_ws/src/ros_comm/tools/rosmaster/test --with-xunit --xunit-file=/home/husarion/catkin_ws/build/rosmaster/test_results/rosmaster/nosetests-test.xml")
subdirs("gtest")
