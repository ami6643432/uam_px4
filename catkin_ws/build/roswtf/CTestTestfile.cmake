# CMake generated Testfile for 
# Source directory: /home/husarion/catkin_ws/src/ros_comm/utilities/roswtf
# Build directory: /home/husarion/catkin_ws/build/roswtf
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_roswtf_rostest_test_roswtf.test "/home/husarion/catkin_ws/build/roswtf/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/roswtf/test_results/roswtf/rostest-test_roswtf.xml" "--return-code" "/usr/bin/python2 /home/husarion/catkin_ws/src/ros_comm/tools/rostest/scripts/rostest --pkgdir=/home/husarion/catkin_ws/src/ros_comm/utilities/roswtf --package=roswtf --results-filename test_roswtf.xml --results-base-dir \"/home/husarion/catkin_ws/build/roswtf/test_results\" /home/husarion/catkin_ws/src/ros_comm/utilities/roswtf/test/roswtf.test ")
add_test(_ctest_roswtf_nosetests_test "/home/husarion/catkin_ws/build/roswtf/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/roswtf/test_results/roswtf/nosetests-test.xml" "--return-code" "\"/usr/bin/cmake\" -E make_directory /home/husarion/catkin_ws/build/roswtf/test_results/roswtf" "/usr/bin/nosetests-2.7 -P --process-timeout=60 --where=/home/husarion/catkin_ws/src/ros_comm/utilities/roswtf/test --with-xunit --xunit-file=/home/husarion/catkin_ws/build/roswtf/test_results/roswtf/nosetests-test.xml")
subdirs("gtest")
