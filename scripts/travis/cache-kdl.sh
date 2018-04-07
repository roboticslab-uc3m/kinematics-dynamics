#!/bin/sh

KDL_SOURCE_DIR=~/"kdl-$KDL_VER"
KDL_BUILD_DIR="$KDL_SOURCE_DIR/build"
KDL_CACHE_DIR="$CACHE_DIR/kdl"
KDL_CLONE_URL=https://github.com/orocos/orocos_kinematics_dynamics

if [ "$TRAVIS_EVENT_TYPE" = "cron" ]; then
  echo "Cloning KDL's default branch"
  git clone --depth=1 "$KDL_CLONE_URL" "$KDL_SOURCE_DIR"
  mkdir -p "$KDL_SOURCE_DIR/build"
  cmake -H"$KDL_SOURCE_DIR/orocos_kdl" -B"$KDL_BUILD_DIR"
  make -C "$KDL_BUILD_DIR" -j$(nproc)
elif [ ! -d "$KDL_CACHE_DIR" ] || [ ! -f "$KDL_CACHE_DIR/.version" ] || [ ! $(cat "$KDL_CACHE_DIR/.version") = "$KDL_VER" ]; then
  echo "KDL not in cache or wrong version"
  rm -rf "$KDL_CACHE_DIR"/*
  wget -q "$KDL_CLONE_URL/archive/v$KDL_VER.tar.gz" -P "$KDL_SOURCE_DIR"
  tar xzf "$KDL_SOURCE_DIR/v$KDL_VER.tar.gz" -C "$KDL_SOURCE_DIR" --strip-components=1
  mkdir -p "$KDL_BUILD_DIR"
  cmake -H"$KDL_SOURCE_DIR/orocos_kdl" -B"$KDL_BUILD_DIR" -DCMAKE_INSTALL_PREFIX:PATH="$KDL_CACHE_DIR"
  make -C "$KDL_BUILD_DIR" -j$(nproc) install
  echo "$KDL_VER" > "$KDL_CACHE_DIR/.version"
else
  echo "KDL directory found in cache (version $(cat "$KDL_CACHE_DIR/.version"))"
fi

# make installed KDL discoverable by CMake's find_package() command
if [ ! "$TRAVIS_EVENT_TYPE" = "cron" ]; then
  export orocos_kdl_DIR="$KDL_CACHE_DIR/share/orocos_kdl"
  export LD_LIBRARY_PATH="$KDL_CACHE_DIR/lib:$LD_LIBRARY_PATH"
else
  export orocos_kdl_DIR="$KDL_BUILD_DIR"
  export LD_LIBRARY_PATH="$KDL_BUILD_DIR/lib:$LD_LIBRARY_PATH"
fi
