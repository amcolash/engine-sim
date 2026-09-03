# Default recipe: list available commands
default:
    @just --list

# Configure and build the project with CMake
build *FLAGS:
    #!/usr/bin/env bash
    set -e
    if [ ! -d "build" ] || [ ! -f "build/CMakeCache.txt" ]; then
        nix develop -c cmake -B build -DCMAKE_BUILD_TYPE=Release
    fi
    nix develop -c cmake --build build -j"$(nproc)" {{FLAGS}}

# Run interactive engine simulator GUI
run *ARGS: build
    nix develop -c ./build/engine-sim-app {{ARGS}}

# Run the test suite
test *ARGS: build
    nix develop -c ./build/engine-sim-test {{ARGS}}

# Export all game vehicles audio (full batch)
export-audio *ARGS: build
    nix develop -c ./build/engine-sim-app --export-audio recipes/game_vehicles.json {{ARGS}}

# Export fast test vehicle audio
export-fast *ARGS: build
    nix develop -c ./build/engine-sim-app --export-audio recipes/fast_test.json {{ARGS}}

# Export single MR engine script directly
export-engine SCRIPT *ARGS: build
    nix develop -c ./build/engine-sim-app --export-engine {{SCRIPT}} {{ARGS}}

# Remove build directory
clean:
    rm -rf build
