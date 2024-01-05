#!/bin/bash
mkdir -p wpilib_build
cd wpilib_build
rm -rf *
export CC=gcc-11
export CXX=g++-11

cmake -DBUILD_GMOCK=OFF \
-DINSTALL_GTEST=OFF \
-DWITH_CSCORE=OFF \
-DWITH_GUI=OFF \
-DWITH_JAVA=OFF \
-DWITH_CSCORE=OFF \
-DWITH_EXAMPLES=OFF \
-DWITH_SIMULATION_MODULES=OFF \
-DWITH_TESTS=OFF \
-DWITH_WPILIB=OFF \
../allwpilib

make -j24

cd ..
mkdir build
cd build
cmake ..
make