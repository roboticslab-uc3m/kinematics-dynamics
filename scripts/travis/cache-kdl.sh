#!/usr/bin/env bash

set -e

KDL_CLONE_BRANCH=master
KDL_CLONE_URL=https://github.com/orocos/orocos_kinematics_dynamics
KDL_SOURCE_DIR=~/"kdl-$KDL_VER"
KDL_BUILD_DIR="$KDL_SOURCE_DIR/build"
KDL_CACHE_DIR="$CACHE_DIR/kdl"
KDL_CMAKE_OPTIONS="-DCMAKE_INSTALL_PREFIX:PATH=$KDL_CACHE_DIR"

echo "Cloning KDL's $KDL_CLONE_BRANCH branch"
git clone --depth=1 --branch="$KDL_CLONE_BRANCH" "$KDL_CLONE_URL" "$KDL_SOURCE_DIR"
last_commit_sha=`git -C "$KDL_SOURCE_DIR" rev-parse HEAD`

if [ ! -d "$KDL_CACHE_DIR" ] || \
   [ ! -f "$KDL_CACHE_DIR/.last_commit_sha" ] || \
   [ ! `cat $KDL_CACHE_DIR/.last_commit_sha` = "$last_commit_sha" ];
then
  echo "KDL not in cache or not the latest commit of $KDL_CLONE_BRANCH branch"
  rm -rf "$KDL_CACHE_DIR"/*
  cmake -H"$KDL_SOURCE_DIR/orocos_kdl" -B"$KDL_BUILD_DIR" $KDL_CMAKE_OPTIONS
  make -C "$KDL_BUILD_DIR" -j$(nproc) install
  echo "$last_commit_sha" > "$KDL_CACHE_DIR/.last_commit_sha"
else
  echo "KDL found in cache (`cat $KDL_CACHE_DIR/.last_commit_sha`)"
fi

# make installed KDL discoverable by CMake's find_package() command
export CMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH:$KDL_CACHE_DIR"
export LD_LIBRARY_PATH="$KDL_CACHE_DIR/lib:$LD_LIBRARY_PATH"
