{ pkgs ? import <nixpkgs> {} }:
(pkgs.callPackage ./default.nix {}).overrideAttrs (old: {
  shellHook = ''
    export ENGINE_SIM_DATA_ROOT="$PWD"
    export LD_LIBRARY_PATH="${pkgs.lib.makeLibraryPath (with pkgs; [
      libGL
      libGLU
      SDL2
      SDL2_image
      libx11
    ])}:$LD_LIBRARY_PATH"
  '';
})

