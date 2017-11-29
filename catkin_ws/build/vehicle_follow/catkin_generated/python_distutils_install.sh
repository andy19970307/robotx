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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/david/robotx/catkin_ws/src/vehicle_follow"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/david/robotx/catkin_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/david/robotx/catkin_ws/install/lib/python2.7/dist-packages:/home/david/robotx/catkin_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/david/robotx/catkin_ws/build" \
    "/usr/bin/python" \
    "/home/david/robotx/catkin_ws/src/vehicle_follow/setup.py" \
    build --build-base "/home/david/robotx/catkin_ws/build/vehicle_follow" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/david/robotx/catkin_ws/install" --install-scripts="/home/david/robotx/catkin_ws/install/bin"
