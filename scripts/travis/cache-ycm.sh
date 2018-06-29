#!/bin/sh

YCM_CLONE_BRANCH="${ROBOTOLOGY_CHECKOUT:-${YCM_CHECKOUT:-master}}"
YCM_CLONE_URL=https://github.com/robotology/ycm
YCM_SOURCE_DIR=~/"ycm-$YCM_CLONE_BRANCH"
YCM_BUILD_DIR="$YCM_SOURCE_DIR/build"
YCM_CACHE_DIR="$CACHE_DIR/ycm-$YCM_CLONE_BRANCH"
YCM_CMAKE_OPTIONS="$YCM_CMAKE_OPTIONS -DCMAKE_INSTALL_PREFIX:PATH=$YCM_CACHE_DIR"

case "$YCM_CLONE_BRANCH" in

  master|devel)

    echo "Cloning YCM's $YCM_CLONE_BRANCH branch"
    git clone --depth=1 --branch="$YCM_CLONE_BRANCH" "$YCM_CLONE_URL" "$YCM_SOURCE_DIR"
    last_commit_sha=`git -C "$YCM_SOURCE_DIR" rev-parse HEAD`

    if [ ! -d "$YCM_CACHE_DIR" ] || \
       [ ! -f "$YCM_CACHE_DIR/.last_commit_sha" ] || \
       [ ! `cat $YCM_CACHE_DIR/.last_commit_sha` = "$last_commit_sha" ];
    then
      echo "YCM not in cache or not the latest commit of $YCM_CLONE_BRANCH branch"
      rm -rf "$YCM_CACHE_DIR"/*
      cmake -H"$YCM_SOURCE_DIR" -B"$YCM_BUILD_DIR" $YCM_CMAKE_OPTIONS
      make -C "$YCM_BUILD_DIR" -j$(nproc) install
      echo "$last_commit_sha" > "$YCM_CACHE_DIR/.last_commit_sha"
    else
      echo "YCM found in cache (`cat $YCM_CACHE_DIR/.last_commit_sha`)"
    fi

    ;;

  *)

    if [ ! -d "$YCM_CACHE_DIR" ];
    then
      wget -q "$YCM_CLONE_URL/archive/$YCM_CLONE_BRANCH.tar.gz" -P "$YCM_SOURCE_DIR"
      tar xzf "$YCM_SOURCE_DIR/$YCM_CLONE_BRANCH.tar.gz" -C "$YCM_SOURCE_DIR" --strip-components=1
      cmake -H"$YCM_SOURCE_DIR" -B"$YCM_BUILD_DIR" $YCM_CMAKE_OPTIONS
      make -C "$YCM_BUILD_DIR" -j$(nproc) install
    else
      echo "YCM $YCM_CLONE_BRANCH already in cache"
    fi

    ;;

esac

# make installed YCM discoverable by CMake's find_package() command
export CMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH:$YCM_CACHE_DIR"
