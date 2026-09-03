#!/bin/sh
set -e

if [ ! -d "build" ] || [ ! -f "build/CMakeCache.txt" ]; then
    nix develop -c cmake -B build -DCMAKE_BUILD_TYPE=Release
fi

nix develop -c cmake --build build -j"$(nproc)" "$@"
