# CMake generated Testfile for 
# Source directory: /home/husarion/catkin_ws/src/ros_comm/test/test_rosbag_storage
# Build directory: /home/husarion/catkin_ws/build/test_rosbag_storage
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_test_rosbag_storage_gtest_bag_player "/home/husarion/catkin_ws/build/test_rosbag_storage/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/test_rosbag_storage/test_results/test_rosbag_storage/gtest-bag_player.xml" "--return-code" "/home/husarion/catkin_ws/devel/.private/test_rosbag_storage/lib/test_rosbag_storage/bag_player --gtest_output=xml:/home/husarion/catkin_ws/build/test_rosbag_storage/test_results/test_rosbag_storage/gtest-bag_player.xml")
add_test(_ctest_test_rosbag_storage_gtest_create_and_iterate_bag "/home/husarion/catkin_ws/build/test_rosbag_storage/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/test_rosbag_storage/test_results/test_rosbag_storage/gtest-create_and_iterate_bag.xml" "--return-code" "/home/husarion/catkin_ws/devel/.private/test_rosbag_storage/lib/test_rosbag_storage/create_and_iterate_bag --gtest_output=xml:/home/husarion/catkin_ws/build/test_rosbag_storage/test_results/test_rosbag_storage/gtest-create_and_iterate_bag.xml")
add_test(_ctest_test_rosbag_storage_gtest_swap_bags "/home/husarion/catkin_ws/build/test_rosbag_storage/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/test_rosbag_storage/test_results/test_rosbag_storage/gtest-swap_bags.xml" "--return-code" "/home/husarion/catkin_ws/devel/.private/test_rosbag_storage/lib/test_rosbag_storage/swap_bags --gtest_output=xml:/home/husarion/catkin_ws/build/test_rosbag_storage/test_results/test_rosbag_storage/gtest-swap_bags.xml")
subdirs("gtest")
