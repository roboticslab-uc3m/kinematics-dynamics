#!/bin/sh

YARP_CLONE_BRANCH="${ROBOTOLOGY_CHECKOUT:-$YARP_CHECKOUT}"
YARP_CLONE_URL=https://github.com/robotology/yarp
YARP_SOURCE_DIR=~/"yarp-$YARP_CLONE_BRANCH"
YARP_BUILD_DIR="$YARP_SOURCE_DIR/build"
YARP_CACHE_DIR="$CACHE_DIR/yarp-$YARP_CLONE_BRANCH"
YARP_CMAKE_OPTIONS="-DSKIP_ACE:BOOL=ON -DCREATE_LIB_MATH:BOOL=ON"
YARP_CMAKE_OPTIONS="$YARP_CMAKE_OPTIONS -DCMAKE_INSTALL_PREFIX:PATH=$YARP_CACHE_DIR"

case "$YARP_CLONE_BRANCH" in

  master|devel)

    echo "Cloning YARP's $YARP_CLONE_BRANCH branch"
    git clone --depth=1 --branch="$YARP_CLONE_BRANCH" "$YARP_CLONE_URL" "$YARP_SOURCE_DIR"
    last_commit_sha=`git -C "$YARP_SOURCE_DIR" rev-parse HEAD`

    if [ ! -d "$YARP_CACHE_DIR" ] || \
       [ ! -f "$YARP_CACHE_DIR/.last_commit_sha" ] || \
       [ ! `cat $YARP_CACHE_DIR/.last_commit_sha` = "$last_commit_sha" ];
    then
      echo "YARP not in cache or not the latest commit of $YARP_CLONE_BRANCH branch"
      rm -rf "$YARP_CACHE_DIR"/*
      cmake -H"$YARP_SOURCE_DIR" -B"$YARP_BUILD_DIR" $YARP_CMAKE_OPTIONS
      make -C "$YARP_BUILD_DIR" -j$(nproc) install
      echo "$last_commit_sha" > "$YARP_CACHE_DIR/.last_commit_sha"
    else
      echo "YARP found in cache (`cat $YARP_CACHE_DIR/.last_commit_sha`)"
    fi

    ;;

  *)

    if [ ! -d "$YARP_CACHE_DIR" ];
    then
      wget -q "$YARP_CLONE_URL/archive/$YARP_CLONE_BRANCH.tar.gz" -P "$YARP_SOURCE_DIR"
      tar xzf "$YARP_SOURCE_DIR/$YARP_CLONE_BRANCH.tar.gz" -C "$YARP_SOURCE_DIR" --strip-components=1
      cmake -H"$YARP_SOURCE_DIR" -B"$YARP_BUILD_DIR" $YARP_CMAKE_OPTIONS
      make -C "$YARP_BUILD_DIR" -j$(nproc) install
    else
      echo "YARP $YARP_CLONE_BRANCH already in cache"
    fi

    ;;

esac

# make installed YARP discoverable by CMake's find_package() command
export YARP_DIR="$YARP_CACHE_DIR/lib/cmake/YARP"
export LD_LIBRARY_PATH="$YARP_CACHE_DIR/lib:$LD_LIBRARY_PATH"
export YARP_DATA_DIRS="$YARP_CACHE_DIR/share/yarp:$YARP_DATA_DIRS"
