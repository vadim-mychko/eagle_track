#!/bin/bash

export OPENCV_DIR="$HOME/opencv-4.9.0"
export LD_LIBRARY_PATH="$OPENCV_DIR/install/lib":$LD_LIBRARY_PATH
export PKG_CONFIG_PATH="$OPENCV_DIR/build/unix-install":$PKG_CONFIG_PATH
