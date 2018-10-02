#!/usr/bin/env bash

set -e

#-- Initialize variables
package_name=
repo_url=
repo_checkout=
clone_only_branches="master devel develop production"
cmake_home_dir=
additional_cmake_options=
prepend_to_linker_path=
prepend_to_standard_path=
additional_export_paths=

#-- Available getopt long option names
#-- https://gist.github.com/magnetikonline/22c1eb412daa350eeceee76c97519da8
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

#-- Read arguments
opts=$(getopt \
    --longoptions "$(printf "%s:," "${ARGUMENT_LIST[@]}")" \
    --name "$(basename "${BASH_SOURCE[0]}")" \
    --options "" \
    -- "$@"
)

eval set -- $opts

#-- Parse options
while [ "$#" -gt 1 ]; do
    in=$(echo "$1" | sed -e 's/^--//')
    for v in "${ARGUMENT_LIST[@]}"; do
        if [ "$v" = "$in" ]; then
            declare "$(echo "$v" | tr '-' '_')"="$2"
            break
        fi
    done
    shift 2
done

#-- Check required arguments
if [ -z "$package_name" ] || [ -z "$repo_url" ] || [ -z "$repo_checkout" ]; then
    echo "Missing required options. Traceback:"
    for v in "${ARGUMENT_LIST[@]}"; do
        v_param=$(echo $v | tr '-' '_')
        echo "  --$v=${!v_param}"
    done
    return 1
fi

#-- Configure paths
repo_source_dir=~/"$package_name-$repo_checkout"
repo_build_dir="$repo_source_dir/build"
repo_cache_dir="$CACHE_DIR/$package_name-$repo_checkout"
cmake_home_dir="$repo_source_dir/$cmake_home_dir"

#-- Configure CMake command line options
repo_cmake_options="$additional_cmake_options -DCMAKE_INSTALL_PREFIX:PATH=$repo_cache_dir"

is_clone_only_branch=false

for branch in $clone_only_branches; do
    if [ "$repo_checkout" = "$branch" ]; then
        is_clone_only_branch=true
        break
    fi
done

if $is_clone_only_branch; then

    #-- Clone, build and store in cache

    echo "Cloning $package_name's $repo_checkout branch"
    git clone --depth=1 --branch="$repo_checkout" "$repo_url" "$repo_source_dir"
    last_commit_sha=$(git -C "$repo_source_dir" rev-parse HEAD)

    if [ ! -d "$repo_cache_dir" ] || \
       [ ! -f "$repo_cache_dir/.last_commit_sha" ] || \
       [ ! "$(cat $repo_cache_dir/.last_commit_sha)" = "$last_commit_sha" ];
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

    #-- Download zipped file from archive, build and store in cache

    if [ ! -d "$repo_cache_dir" ]; then
        echo "Downloading $package_name $repo_checkout from archive"
        wget -q "$repo_url/archive/$repo_checkout.tar.gz" -P "$repo_source_dir"
        tar xzf "$repo_source_dir/$repo_checkout.tar.gz" -C "$repo_source_dir" --strip-components=1
        cmake -H"$cmake_home_dir" -B"$repo_build_dir" $repo_cmake_options
        make -C "$repo_build_dir" -j$(nproc) install
    else
        echo "$package_name $repo_checkout already in cache"
    fi

fi

#-- Remove clone/download tree (QA#70)
rm -rf "$repo_source_dir"

#-- Make installed package discoverable by CMake's find_package() command
export CMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH:$repo_cache_dir"

#-- Miscellanea, prepends stuff to LD_LIBRARY_PATH and PATH

if [ ! -z "$prepend_to_linker_path" ]; then
    export LD_LIBRARY_PATH="$repo_cache_dir/$prepend_to_linker_path:$LD_LIBRARY_PATH"
fi

if [ ! -z "$prepend_to_standard_path" ]; then
    export PATH="$repo_cache_dir/$prepend_to_standard_path:$PATH"
fi

#-- Expands lists of values of the form 'VAR1;val1;VAR2;val2;...' to:
#--  export VAR1=val1
#--  export VAR2=val2
#--  ...
if [ ! -z "$additional_export_paths" ]; then
    IFS=';' read -ra ITEMS <<< "$additional_export_paths"
    for (( count=0; count<"${#ITEMS[@]}"; count+=2 )); do
        export "${ITEMS[$count]}"="$repo_cache_dir/${ITEMS[$(( $count+1 ))]}:${!ITEMS[$count]}"
    done
fi
