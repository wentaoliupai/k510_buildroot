#!/bin/bash

# set cross build toolchain
export PATH=$PATH:/mnt/Andestech/AndeSight_STD_v321/toolchains/nds64le-linux-glibc-v5d/bin

rm -rf out
mkdir out
pushd out
cmake -DCMAKE_BUILD_TYPE=Release                 \
      -DCMAKE_INSTALL_PREFIX=`pwd`               \
      -DCMAKE_TOOLCHAIN_FILE=cmake/Riscv64.cmake \
      ..

make -j && make install

popd
