#!/bin/bash
export QT5PATH=/usr/local/Qt-5.4.2/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$QT5PATH
SCRIPT_DIR="$( cd "$( dirname "$0" )" && pwd )"

if [ ! -f $QT5PATH/libQt5Core.so ]; then
    echo "Invalid QT5 path. Edit this script and specify the QT5 path."
    exit
fi

if [ ! -f $SCRIPT_DIR/LeddarCDemo ]; then
    echo "Please compile LeddarCDemo first."
    exit
fi

$SCRIPT_DIR/LeddarCDemo