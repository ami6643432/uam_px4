# CMake generated Testfile for 
# Source directory: /home/husarion/catkin_ws/src/pluginlib/pluginlib
# Build directory: /home/husarion/catkin_ws/build/pluginlib
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_pluginlib_gtest_pluginlib_utest "/home/husarion/catkin_ws/build/pluginlib/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/pluginlib/test_results/pluginlib/gtest-pluginlib_utest.xml" "--return-code" "/home/husarion/catkin_ws/devel/.private/pluginlib/lib/pluginlib/pluginlib_utest --gtest_output=xml:/home/husarion/catkin_ws/build/pluginlib/test_results/pluginlib/gtest-pluginlib_utest.xml")
add_test(_ctest_pluginlib_gtest_pluginlib_unique_ptr_test "/home/husarion/catkin_ws/build/pluginlib/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/pluginlib/test_results/pluginlib/gtest-pluginlib_unique_ptr_test.xml" "--return-code" "/home/husarion/catkin_ws/devel/.private/pluginlib/lib/pluginlib/pluginlib_unique_ptr_test --gtest_output=xml:/home/husarion/catkin_ws/build/pluginlib/test_results/pluginlib/gtest-pluginlib_unique_ptr_test.xml")
subdirs("gtest")
