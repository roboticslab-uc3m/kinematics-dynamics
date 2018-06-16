#!/bin/sh

YCM_SOURCE_DIR=~/"ycm-$YCM_VER"
YCM_BUILD_DIR="$YCM_SOURCE_DIR/build"
YCM_CACHE_DIR="$CACHE_DIR/ycm"
YCM_CLONE_BRANCH=devel
YCM_CLONE_URL=https://github.com/robotology/ycm

if [ "$TRAVIS_EVENT_TYPE" = "cron" ]; then
  echo "Cloning YCM's $YCM_CLONE_BRANCH branch"
  git clone --depth=1 --branch="$YCM_CLONE_BRANCH" "$YCM_CLONE_URL" "$YCM_SOURCE_DIR"
  mkdir -p "$YCM_SOURCE_DIR/build"
  cmake -H"$YCM_SOURCE_DIR" -B"$YCM_BUILD_DIR"
  make -C "$YCM_BUILD_DIR" -j$(nproc)
elif [ ! -d "$YCM_CACHE_DIR" ] || [ ! -f "$YCM_CACHE_DIR/.version" ] || [ ! $(cat "$YCM_CACHE_DIR/.version") = "$YCM_VER" ]; then
  echo "YCM not in cache or wrong version"
  rm -rf "$YCM_CACHE_DIR"/*
  wget -q "$YCM_CLONE_URL/archive/v$YCM_VER.tar.gz" -P "$YCM_SOURCE_DIR"
  tar xzf "$YCM_SOURCE_DIR/v$YCM_VER.tar.gz" -C "$YCM_SOURCE_DIR" --strip-components=1
  mkdir -p "$YCM_BUILD_DIR"
  cmake -H"$YCM_SOURCE_DIR" -B"$YCM_BUILD_DIR" -DCMAKE_INSTALL_PREFIX:PATH="$YCM_CACHE_DIR"
  make -C "$YCM_BUILD_DIR" -j$(nproc) install
  echo "$YCM_VER" > "$YCM_CACHE_DIR/.version"
else
  echo "YCM directory found in cache (version $(cat "$YCM_CACHE_DIR/.version"))"
fi

# make installed YCM discoverable by CMake's find_package() command
if [ ! "$TRAVIS_EVENT_TYPE" = "cron" ]; then
  export YCM_DIR="$YCM_CACHE_DIR/share/YCM/cmake"
else
  export YCM_DIR="$YCM_BUILD_DIR"
fi
