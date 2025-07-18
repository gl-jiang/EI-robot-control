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

echo_and_run cd "/home/jgl20/work/workssrc/4myarms/src/remote_control-x86-can-v2/master1/src/kdl_parser-noetic-devel/kdl_parser_py"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/jgl20/work/workssrc/4myarms/src/remote_control-x86-can-v2/master1/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/jgl20/work/workssrc/4myarms/src/remote_control-x86-can-v2/master1/install/lib/python3/dist-packages:/home/jgl20/work/workssrc/4myarms/src/remote_control-x86-can-v2/master1/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/jgl20/work/workssrc/4myarms/src/remote_control-x86-can-v2/master1/build" \
    "/usr/bin/python3" \
    "/home/jgl20/work/workssrc/4myarms/src/remote_control-x86-can-v2/master1/src/kdl_parser-noetic-devel/kdl_parser_py/setup.py" \
    egg_info --egg-base /home/jgl20/work/workssrc/4myarms/src/remote_control-x86-can-v2/master1/build/kdl_parser-noetic-devel/kdl_parser_py \
    build --build-base "/home/jgl20/work/workssrc/4myarms/src/remote_control-x86-can-v2/master1/build/kdl_parser-noetic-devel/kdl_parser_py" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/jgl20/work/workssrc/4myarms/src/remote_control-x86-can-v2/master1/install" --install-scripts="/home/jgl20/work/workssrc/4myarms/src/remote_control-x86-can-v2/master1/install/bin"
