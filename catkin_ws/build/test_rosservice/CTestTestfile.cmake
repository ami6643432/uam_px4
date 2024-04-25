# CMake generated Testfile for 
# Source directory: /home/husarion/catkin_ws/src/ros_comm/test/test_rosservice
# Build directory: /home/husarion/catkin_ws/build/test_rosservice
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_test_rosservice_nosetests_test "/home/husarion/catkin_ws/build/test_rosservice/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/test_rosservice/test_results/test_rosservice/nosetests-test.xml" "--return-code" "\"/usr/bin/cmake\" -E make_directory /home/husarion/catkin_ws/build/test_rosservice/test_results/test_rosservice" "/usr/bin/nosetests-2.7 -P --process-timeout=60 --where=/home/husarion/catkin_ws/src/ros_comm/test/test_rosservice/test --with-xunit --xunit-file=/home/husarion/catkin_ws/build/test_rosservice/test_results/test_rosservice/nosetests-test.xml")
add_test(_ctest_test_rosservice_rostest_test_rosservice.test "/home/husarion/catkin_ws/build/test_rosservice/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/test_rosservice/test_results/test_rosservice/rostest-test_rosservice.xml" "--return-code" "/usr/bin/python2 /home/husarion/catkin_ws/src/ros_comm/tools/rostest/scripts/rostest --pkgdir=/home/husarion/catkin_ws/src/ros_comm/test/test_rosservice --package=test_rosservice --results-filename test_rosservice.xml --results-base-dir \"/home/husarion/catkin_ws/build/test_rosservice/test_results\" /home/husarion/catkin_ws/src/ros_comm/test/test_rosservice/test/rosservice.test ")
subdirs("gtest")
