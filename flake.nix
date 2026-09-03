{
  description = "Engine Simulator - Real-time combustion engine sound synthesis";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
        engine-sim = pkgs.callPackage ./default.nix {};
      in
      {
        packages = {
          default = engine-sim;
          engine-sim = engine-sim;
        };

        apps = {
          default = flake-utils.lib.mkApp {
            drv = engine-sim;
            name = "engine-sim";
          };
          engine-sim = flake-utils.lib.mkApp {
            drv = engine-sim;
            name = "engine-sim";
          };
        };

        devShells.default = pkgs.mkShell {
          inputsFrom = [ engine-sim ];
          packages = with pkgs; [
            gdb
            clang-tools
          ];
          shellHook = ''
            export ENGINE_SIM_DATA_ROOT="$PWD"
            export LIBGL_DRIVERS_PATH="${pkgs.mesa}/lib/dri:$LIBGL_DRIVERS_PATH"
            export __EGL_VENDOR_LIBRARY_DIRS="${pkgs.mesa}/share/glvnd/egl_vendor.d:$__EGL_VENDOR_LIBRARY_DIRS"
            export LD_LIBRARY_PATH="${pkgs.lib.makeLibraryPath (with pkgs; [
              libGL
              libGLU
              SDL2
              SDL2_image
              libx11
              mesa
            ])}:$LD_LIBRARY_PATH"
            # Force SDL (including SDL3 via sdl2-compat) to use X11/GLX rather than
            # Wayland/EGL, which fails on this system due to AMD DRM permissions.
            export SDL_VIDEODRIVER=x11
            unset WAYLAND_DISPLAY
            echo "=========================================="
            echo " Engine Simulator Native Linux Environment "
            echo "=========================================="
            echo "Build:   cmake -B build && cmake --build build"
            echo "Run:     ./build/engine-sim-app"
            echo "Package: nix build"
            echo "=========================================="
          '';
        };
      }
    );
}

