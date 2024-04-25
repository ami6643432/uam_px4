#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/husarion/catkin_ws/src/ros_comm/clients/rospy"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/husarion/catkin_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/husarion/catkin_ws/install/lib/python2.7/dist-packages:/home/husarion/catkin_ws/build/rospy/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/husarion/catkin_ws/build/rospy" \
    "/usr/bin/python2" \
    "/home/husarion/catkin_ws/src/ros_comm/clients/rospy/setup.py" \
     \
    build --build-base "/home/husarion/catkin_ws/build/rospy" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/husarion/catkin_ws/install" --install-scripts="/home/husarion/catkin_ws/install/bin"
