#!/bin/sh
SRC=kernel
CROSS_COMPILE=arm-arago-linux-gnueabi-
ARCH=arm
DEFAULT_CONFIG=""

case "$1" in
    clean)
        echo -n "Starting $SRC make $1"
	echo ""
        make CROSS_COMPILE=$CROSS_COMPILE ARCH=$ARCH clean
        echo "."
        ;;
    uImage)
        echo -n "Starting $SRC make $1"
	echo ""
        make CROSS_COMPILE=$CROSS_COMPILE ARCH=$ARCH uImage
        echo "."
        ;;
    *)
        echo "Usage: $0 {clean|uImage}"
        exit 1
        ;;
esac

exit 0


#DAEMON=/usr/sbin/pcscd
#NAME=pcscd
#DESC="PCSC Daemon"
#PIDFILE=/var/run/pcscd/pcscd.pid
#ARGS=""
#test -f $DAEMON || exit 0
