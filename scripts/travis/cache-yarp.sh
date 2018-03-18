#!/bin/sh

YARP_SOURCE_DIR="~/yarp-$YARP_VER"
YARP_BUILD_DIR="$YARP_SOURCE_DIR/build"
YARP_CACHE_DIR="$CACHE_DIR/yarp"
YARP_CMAKE_OPTIONS="-DSKIP_ACE:BOOL=ON -DCREATE_LIB_MATH:BOOL=ON"
YARP_CLONE_BRANCH=devel

if [ "$TRAVIS_EVENT_TYPE" = "cron" ]; then
  echo "Cloning YARP's $YARP_CLONE_BRANCH branch"
  git clone --depth=1 --branch="$YARP_CLONE_BRANCH" https://github.com/robotology/yarp "$YARP_SOURCE_DIR"
  mkdir -p "$YARP_SOURCE_DIR/build"
  cmake -H"$YARP_SOURCE_DIR" -B"$YARP_BUILD_DIR" $YARP_CMAKE_OPTIONS
  sudo make -C "$YARP_BUILD_DIR" -j$(nproc) install
elif [ ! -d "$YARP_CACHE_DIR"] || [ ! -f "$YARP_CACHE_DIR/.version" ] || [ ! $(cat "$YARP_CACHE_DIR/.version") = "$YARP_VER" ]; then
  echo "YARP not in cache or wrong version"
  rm -rf "$YARP_CACHE_DIR"/*
  wget -q "https://github.com/robotology/yarp/archive/v$YARP_VER.tar.gz" -P "$YARP_SOURCE_DIR"
  tar xzf "$YARP_SOURCE_DIR/v$YARP_VER.tar.gz" "yarp-$YARP_VER" -C "$YARP_SOURCE_DIR"
  mkdir -p "$YARP_BUILD_DIR"
  cmake -H"$YARP_SOURCE_DIR" -B"$YARP_BUILD_DIR" -DCMAKE_INSTALL_PREFIX:PATH="$YARP_CACHE_DIR" $YARP_CMAKE_OPTIONS
  sudo make -C "$YARP_BUILD_DIR" -j$(nproc) install
  echo "$YARP_VER" > "$YARP_CACHE_DIR/.version"
else
  echo "YARP directory found in cache (version $(cat "$YARP_CACHE_DIR/.version"))"
fi

# make installed YARP discoverable by CMake's find_package() command
if [ ! "$TRAVIS_EVENT_TYPE" = "cron" ]; then export YARP_DIR="$YARP_CACHE_DIR/lib/cmake/YARP"; fi
