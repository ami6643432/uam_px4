# CMake generated Testfile for 
# Source directory: /home/husarion/catkin_ws/src/realtime_tools
# Build directory: /home/husarion/catkin_ws/build/realtime_tools
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_realtime_tools_gtest_realtime_box_tests "/home/husarion/catkin_ws/build/realtime_tools/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/realtime_tools/test_results/realtime_tools/gtest-realtime_box_tests.xml" "--return-code" "/home/husarion/catkin_ws/devel/.private/realtime_tools/lib/realtime_tools/realtime_box_tests --gtest_output=xml:/home/husarion/catkin_ws/build/realtime_tools/test_results/realtime_tools/gtest-realtime_box_tests.xml")
add_test(_ctest_realtime_tools_gtest_realtime_buffer_tests "/home/husarion/catkin_ws/build/realtime_tools/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/realtime_tools/test_results/realtime_tools/gtest-realtime_buffer_tests.xml" "--return-code" "/home/husarion/catkin_ws/devel/.private/realtime_tools/lib/realtime_tools/realtime_buffer_tests --gtest_output=xml:/home/husarion/catkin_ws/build/realtime_tools/test_results/realtime_tools/gtest-realtime_buffer_tests.xml")
add_test(_ctest_realtime_tools_gtest_realtime_clock_tests "/home/husarion/catkin_ws/build/realtime_tools/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/realtime_tools/test_results/realtime_tools/gtest-realtime_clock_tests.xml" "--return-code" "/home/husarion/catkin_ws/devel/.private/realtime_tools/lib/realtime_tools/realtime_clock_tests --gtest_output=xml:/home/husarion/catkin_ws/build/realtime_tools/test_results/realtime_tools/gtest-realtime_clock_tests.xml")
add_test(_ctest_realtime_tools_rostest_test_realtime_publisher.test "/home/husarion/catkin_ws/build/realtime_tools/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/realtime_tools/test_results/realtime_tools/rostest-test_realtime_publisher.xml" "--return-code" "/usr/bin/python2 /home/husarion/catkin_ws/src/ros_comm/tools/rostest/scripts/rostest --pkgdir=/home/husarion/catkin_ws/src/realtime_tools --package=realtime_tools --results-filename test_realtime_publisher.xml --results-base-dir \"/home/husarion/catkin_ws/build/realtime_tools/test_results\" /home/husarion/catkin_ws/src/realtime_tools/test/realtime_publisher.test ")
add_test(_ctest_realtime_tools_rostest_test_realtime_server_goal_handle.test "/home/husarion/catkin_ws/build/realtime_tools/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/realtime_tools/test_results/realtime_tools/rostest-test_realtime_server_goal_handle.xml" "--return-code" "/usr/bin/python2 /home/husarion/catkin_ws/src/ros_comm/tools/rostest/scripts/rostest --pkgdir=/home/husarion/catkin_ws/src/realtime_tools --package=realtime_tools --results-filename test_realtime_server_goal_handle.xml --results-base-dir \"/home/husarion/catkin_ws/build/realtime_tools/test_results\" /home/husarion/catkin_ws/src/realtime_tools/test/realtime_server_goal_handle.test ")
subdirs("gtest")