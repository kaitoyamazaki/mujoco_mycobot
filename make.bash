#!/bin/bash

# buildディレクトリを削除
rm -rf build

# 再度buildディレクトリを作成
mkdir build

# buildディレクトリに移動
cd build

# CMakeとMakeを実行
cmake ..
make
./mujoco ../mycobot_pi.xml