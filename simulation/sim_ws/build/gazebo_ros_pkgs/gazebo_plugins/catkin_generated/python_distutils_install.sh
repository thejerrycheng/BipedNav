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

echo_and_run cd "/home/jerry/sim_ws/src/gazebo_ros_pkgs/gazebo_plugins"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/jerry/sim_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/jerry/sim_ws/install/lib/python3/dist-packages:/home/jerry/sim_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/jerry/sim_ws/build" \
    "/home/jerry/miniconda3/bin/python3" \
    "/home/jerry/sim_ws/src/gazebo_ros_pkgs/gazebo_plugins/setup.py" \
     \
    build --build-base "/home/jerry/sim_ws/build/gazebo_ros_pkgs/gazebo_plugins" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/jerry/sim_ws/install" --install-scripts="/home/jerry/sim_ws/install/bin"
