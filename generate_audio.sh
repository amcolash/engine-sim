#!/bin/sh
nix develop -c ./build/engine-sim-app --export-audio recipes/game_vehicles.json "$@"