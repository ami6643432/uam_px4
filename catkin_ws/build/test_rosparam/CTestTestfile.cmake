# CMake generated Testfile for 
# Source directory: /home/husarion/catkin_ws/src/ros_comm/test/test_rosparam
# Build directory: /home/husarion/catkin_ws/build/test_rosparam
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_test_rosparam_rostest_test_rosparam.test "/home/husarion/catkin_ws/build/test_rosparam/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/test_rosparam/test_results/test_rosparam/rostest-test_rosparam.xml" "--return-code" "/usr/bin/python2 /home/husarion/catkin_ws/src/ros_comm/tools/rostest/scripts/rostest --pkgdir=/home/husarion/catkin_ws/src/ros_comm/test/test_rosparam --package=test_rosparam --results-filename test_rosparam.xml --results-base-dir \"/home/husarion/catkin_ws/build/test_rosparam/test_results\" /home/husarion/catkin_ws/src/ros_comm/test/test_rosparam/test/rosparam.test ")
add_test(_ctest_test_rosparam_nosetests_test "/home/husarion/catkin_ws/build/test_rosparam/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/test_rosparam/test_results/test_rosparam/nosetests-test.xml" "--return-code" "\"/usr/bin/cmake\" -E make_directory /home/husarion/catkin_ws/build/test_rosparam/test_results/test_rosparam" "/usr/bin/nosetests-2.7 -P --process-timeout=60 --where=/home/husarion/catkin_ws/src/ros_comm/test/test_rosparam/test --with-xunit --xunit-file=/home/husarion/catkin_ws/build/test_rosparam/test_results/test_rosparam/nosetests-test.xml")
subdirs("gtest")
