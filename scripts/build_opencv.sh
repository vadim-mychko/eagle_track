#!/bin/bash

OPENCV_VER="4.9.0"
TARGET_DIR="$HOME/opencv-$OPENCV_VER"
BUILD_DIR="$TARGET_DIR/build"
INSTALL_DIR="$TARGET_DIR/install"

PYTHON_VER=310
PYTHON_EXEC="$HOME/eagle_eval/venv/bin/python"
PYTHON_PKG="$HOME/eagle_eval/venv/lib/python3.10/site-packages"
PYTHON_HEADERS="$HOME/.pyenv/versions/3.10.13/include/python3.10"
PYTHON_LIB="$HOME/.pyenv/versions/3.10.13/lib/libpython3.10.so.1.0"

mkdir -p $TARGET_DIR && cd $TARGET_DIR

wget -O opencv.zip https://github.com/opencv/opencv/archive/refs/tags/$OPENCV_VER.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/refs/tags/$OPENCV_VER.zip
unzip opencv.zip && rm opencv.zip
unzip opencv_contrib.zip && rm opencv_contrib.zip

mkdir -p $BUILD_DIR && cd $BUILD_DIR

cmake -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
      -D BUILD_opencv_python2=OFF \
      -D BUILD_opencv_python3=ON \
      -D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib-$OPENCV_VER/modules \
      -D OPENCV_GENERATE_PKGCONFIG=ON \
      -D OPENCV_PC_FILE_NAME=opencv.pc \
      -D PYTHON_VERSION=$PYTHON_VER \
      -D PYTHON_EXECUTABLE=$PYTHON_EXEC \
      -D PYTHON_DEFAULT_EXECUTABLE=$PYTHON_EXEC \
      -D PYTHON3_EXECUTABLE=$PYTHON_EXEC \
      -D OPENCV_PYTHON3_INSTALL_PATH=$PYTHON_PKG \
      -D PYTHON3_PACKAGES_PATH=$PYTHON_PKG \
      -D PYTHON3_NUMPY_INCLUDE_DIRS=$PYTHON_PKG/numpy/core/include \
      -D PYTHON3_INCLUDE_DIR=$PYTHON_HEADERS \
      -D PYTHON3_LIBRARY=$PYTHON_LIB \
      ../opencv-$OPENCV_VER

make -j$(nproc)
mkdir -p $INSTALL_DIR && make install -j$(nproc)

# export CMAKE_ARGS="-DWITH_TBB=ON -DENABLE_FAST_MATH=1 -DCUDA_FAST_MATH=1 -DWITH_CUDA=ON -DWITH_CUBLAS=1 -DWITH_CUDNN=ON -DOPENCV_DNN_CUDA=ON -DCUDA_ARCH_BIN=7.5"
