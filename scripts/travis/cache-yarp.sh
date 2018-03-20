#!/bin/sh

YARP_SOURCE_DIR=~/"yarp-$YARP_VER"
YARP_BUILD_DIR="$YARP_SOURCE_DIR/build"
YARP_CACHE_DIR="$CACHE_DIR/yarp"
YARP_CMAKE_OPTIONS="-DCREATE_LIB_MATH:BOOL=ON"
YARP_CLONE_BRANCH=devel
YARP_CLONE_URL=https://github.com/robotology/yarp

if [ "$TRAVIS_EVENT_TYPE" = "cron" ]; then
  echo "Cloning YARP's $YARP_CLONE_BRANCH branch"
  git clone --depth=1 --branch="$YARP_CLONE_BRANCH" "$YARP_CLONE_URL" "$YARP_SOURCE_DIR"
  mkdir -p "$YARP_SOURCE_DIR/build"
  cmake -H"$YARP_SOURCE_DIR" -B"$YARP_BUILD_DIR" $YARP_CMAKE_OPTIONS
  make -C "$YARP_BUILD_DIR" -j$(nproc)
elif [ ! -d "$YARP_CACHE_DIR" ] || [ ! -f "$YARP_CACHE_DIR/.version" ] || [ ! $(cat "$YARP_CACHE_DIR/.version") = "$YARP_VER" ]; then
  echo "YARP not in cache or wrong version"
  rm -rf "$YARP_CACHE_DIR"/*
  wget -q "$YARP_CLONE_URL/archive/v$YARP_VER.tar.gz" -P "$YARP_SOURCE_DIR"
  tar xzf "$YARP_SOURCE_DIR/v$YARP_VER.tar.gz" -C "$YARP_SOURCE_DIR" --strip-components=1
  mkdir -p "$YARP_BUILD_DIR"
  cmake -H"$YARP_SOURCE_DIR" -B"$YARP_BUILD_DIR" -DCMAKE_INSTALL_PREFIX:PATH="$YARP_CACHE_DIR" $YARP_CMAKE_OPTIONS
  make -C "$YARP_BUILD_DIR" -j$(nproc) install
  echo "$YARP_VER" > "$YARP_CACHE_DIR/.version"
else
  echo "YARP directory found in cache (version $(cat "$YARP_CACHE_DIR/.version"))"
fi

# make installed YARP discoverable by CMake's find_package() command
if [ ! "$TRAVIS_EVENT_TYPE" = "cron" ]; then
  export YARP_DIR="$YARP_CACHE_DIR/lib/cmake/YARP"
  export LD_LIBRARY_PATH="$YARP_CACHE_DIR/lib:$LD_LIBRARY_PATH"
  export YARP_DATA_DIRS="$YARP_CACHE_DIR:$YARP_DATA_DIRS"
else
  export YARP_DIR="$YARP_BUILD_DIR"
  export LD_LIBRARY_PATH="$YARP_BUILD_DIR/lib:$LD_LIBRARY_PATH"
  export YARP_DATA_DIRS="$YARP_BUILD_DIR:$YARP_DATA_DIRS"
fi
