#!/usr/bin/env bash

set -e

package_name=
repo_url=
repo_checkout=
clone_only_branches="master devel develop production"
cmake_home_dir=
additional_cmake_options=
prepend_to_linker_path=
prepend_to_standard_path=
additional_export_paths=

# https://gist.github.com/magnetikonline/22c1eb412daa350eeceee76c97519da8
ARGUMENT_LIST=(
    "package-name"
    "repo-url"
    "repo-checkout"
    "clone-only-branches"
    "cmake-home-dir"
    "additional-cmake-options"
    "prepend-to-linker-path"
    "prepend-to-standard-path"
    "additional-export-paths"
)

# read arguments
opts=$(getopt \
    --longoptions "$(printf "%s:," "${ARGUMENT_LIST[@]}")" \
    --name "$(basename "$0")" \
    --options "" \
    -- "$@"
)

eval set -- $opts

while [[ $# -gt 0 ]]; do
    case "$1" in
        --package-name)
            package_name=$2
            shift 2
            ;;
        --repo-url)
            repo_url=$2
            shift 2
            ;;
        --repo-checkout)
            repo_checkout=$2
            shift 2
            ;;
        --clone-only-branches)
            clone_only_branches=$2
            shift 2
            ;;
        --cmake-home-dir)
            cmake_home_dir=$2
            shift 2
            ;;
        --additional-cmake-options)
            additional_cmake_options=$2
            shift 2
            ;;
        --prepend-to-linker-path)
            prepend_to_linker_path=$2
            shift 2
            ;;
        --prepend-to-standard-path)
            prepend_to_standard_path=$2
            shift 2
            ;;
        --additional-export-paths)
            additional_export_paths=$2
            shift 2
            ;;
        *)
            break
            ;;
    esac
done

if [ -z "$package_name" -o -z "$repo_url" -o -z "$repo_checkout" ]; then
    echo "Missing required options. Traceback:"
    for v in "${ARGUMENT_LIST[@]}"; do
        v_param=$(echo $v | tr '-' '_')
        echo "  --$v=${!v_param}"
    done
    exit 1
fi

repo_source_dir=~/"$package_name-$repo_checkout"
repo_build_dir="$repo_source_dir/build"
repo_cache_dir="$CACHE_DIR/$package_name-$repo_checkout"
repo_cmake_options="$additional_cmake_options -DCMAKE_INSTALL_PREFIX:PATH=$repo_cache_dir"

cmake_home_dir="$repo_source_dir/$cmake_home_dir"

is_clone_only_branch=false

for branch in $clone_only_branches; do
    if [ "$repo_checkout" = "$branch" ] ; then
        is_clone_only_branch=true
        break
    fi
done

if $is_clone_only_branch; then

    echo "Cloning $package_name's $repo_checkout branch"
    git clone --depth=1 --branch="$repo_checkout" "$repo_url" "$repo_source_dir"
    last_commit_sha=$(git -C "$repo_source_dir" rev-parse HEAD)

    if [ ! -d "$repo_cache_dir" ] || \
       [ ! -f "$repo_cache_dir/.last_commit_sha" ] || \
       [ ! $(cat $repo_cache_dir/.last_commit_sha) = "$last_commit_sha" ];
    then
      echo "$package_name not in cache or not the latest commit of $repo_checkout branch"
      rm -rf "$repo_cache_dir"/*
      cmake -H"$cmake_home_dir" -B"$repo_build_dir" $repo_cmake_options
      make -C "$repo_build_dir" -j$(nproc) install
      echo "$last_commit_sha" > "$repo_cache_dir/.last_commit_sha"
    else
      echo "$package_name found in cache ($(cat $repo_cache_dir/.last_commit_sha))"
    fi

else

    if [ ! -d "$repo_cache_dir" ];
    then
      echo "Downloading $package_name $repo_checkout from archive"
      wget -q "$repo_url/archive/$repo_checkout.tar.gz" -P "$repo_source_dir"
      tar xzf "$repo_source_dir/$repo_checkout.tar.gz" -C "$repo_source_dir" --strip-components=1
      cmake -H"$cmake_home_dir" -B"$repo_build_dir" $repo_cmake_options
      make -C "$repo_build_dir" -j$(nproc) install
    else
      echo "$package_name $repo_checkout already in cache"
    fi

fi

# make installed package discoverable by CMake's find_package() command
export CMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH:$repo_cache_dir"

if [ ! -z "$prepend_to_linker_path" ]; then
    export LD_LIBRARY_PATH="$repo_cache_dir/$prepend_to_linker_path:$LD_LIBRARY_PATH"
fi

if [ ! -z "$prepend_to_standard_path" ]; then
    export PATH="$repo_cache_dir/$prepend_to_standard_path:$PATH"
fi

if [ ! -z "$additional_export_paths" ]; then
    IFS=';' read -ra ITEMS <<< "$additional_export_paths"
    for (( count=0; count<"${#ITEMS[@]}"; count+=2 )); do
        export "${ITEMS[$count]}"="$repo_cache_dir/${ITEMS[$(( $count+1 ))]}:${!ITEMS[$count]}"
    done
fi
