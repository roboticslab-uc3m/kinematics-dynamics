#!/bin/sh

KDL_SOURCE_DIR="~/kdl-$KDL_VER"
KDL_BUILD_DIR="$KDL_SOURCE_DIR/build"
KDL_CACHE_DIR="$CACHE_DIR/kdl"

if [ "$TRAVIS_EVENT_TYPE" = "cron" ]; then
  echo "Cloning KDL's default branch"
  git clone --depth=1 https://github.com/orocos/orocos_kinematics_dynamics "$KDL_SOURCE_DIR"
  mkdir -p "$KDL_SOURCE_DIR/build"
  cmake -H"$KDL_SOURCE_DIR/orocos_kdl" -B"$KDL_BUILD_DIR"
  sudo make -C "$KDL_BUILD_DIR" -j$(nproc) install
elif [ ! "$(ls -A "$KDL_CACHE_DIR")" ] || [ ! -f "$KDL_CACHE_DIR/.version" ] || [ ! $(cat "$KDL_CACHE_DIR/.version") = "$KDL_VER" ]; then
  echo "KDL not in cache or wrong version"
  rm -rf "$KDL_CACHE_DIR"/*
  git clone --depth=1 --branch="$KDL_VER" https://github.com/orocos/orocos_kinematics_dynamics "$KDL_SOURCE_DIR"
  mkdir -p "$KDL_BUILD_DIR"
  cmake -H"$KDL_SOURCE_DIR/orocos_kdl" -B"$KDL_BUILD_DIR" -DCMAKE_INSTALL_PREFIX:PATH="$KDL_CACHE_DIR"
  sudo make -C "$KDL_BUILD_DIR" -j$(nproc) install
  echo "$KDL_VER" > "$KDL_CACHE_DIR/.version"
else
  echo "KDL directory found in cache (version $(cat "$KDL_CACHE_DIR/.version"))"
fi

# make installed KDL discoverable by CMake's find_package() command
if [ ! "$TRAVIS_EVENT_TYPE" = "cron" ]; then export orocos_kdl_DIR="$KDL_CACHE_DIR/share/orocos_kdl/cmake"; fi
