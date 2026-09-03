#!/bin/sh
nix develop -c ./build/engine-sim-app --export-audio recipes/fast_test.json "$@"