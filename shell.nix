{ pkgs ? import <nixpkgs> {} }:
(pkgs.callPackage ./default.nix {}).overrideAttrs (old: {
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
  '';
})

