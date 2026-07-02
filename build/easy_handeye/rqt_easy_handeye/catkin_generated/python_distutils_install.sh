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

echo_and_run cd "/home/zhuyupei/alicia_wa_full/src/easy_handeye/rqt_easy_handeye"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/zhuyupei/alicia_wa_full/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/zhuyupei/alicia_wa_full/install/lib/python3/dist-packages:/home/zhuyupei/alicia_wa_full/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/zhuyupei/alicia_wa_full/build" \
    "/usr/bin/python3" \
    "/home/zhuyupei/alicia_wa_full/src/easy_handeye/rqt_easy_handeye/setup.py" \
    egg_info --egg-base /home/zhuyupei/alicia_wa_full/build/easy_handeye/rqt_easy_handeye \
    build --build-base "/home/zhuyupei/alicia_wa_full/build/easy_handeye/rqt_easy_handeye" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/zhuyupei/alicia_wa_full/install" --install-scripts="/home/zhuyupei/alicia_wa_full/install/bin"
