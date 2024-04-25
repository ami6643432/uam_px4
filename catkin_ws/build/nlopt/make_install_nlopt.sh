#!/bin/sh

DESTDIR=/home/husarion/catkin_ws/build/nlopt/nlopt_install make install

cp -r /home/husarion/catkin_ws/build/nlopt/nlopt_install//home/husarion/catkin_ws/install/* /home/husarion/catkin_ws/devel/.private/nlopt/
