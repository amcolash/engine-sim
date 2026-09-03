{ pkgs ? import <nixpkgs> {}
, src ? builtins.fetchGit { url = ./.; submodules = true; }
}:

pkgs.stdenv.mkDerivation {
  pname = "engine-sim";
  version = "0.1.12";

  inherit src;

  nativeBuildInputs = with pkgs; [
    cmake
    pkg-config
    bison
    flex
    makeWrapper
  ];

  buildInputs = with pkgs; [
    SDL2
    SDL2_image
    boost
    libGL
    libGLU
    libx11
    libxrandr
  ];

  cmakeFlags = [
    "-DCMAKE_BUILD_TYPE=Release"
    "-DENGINE_SIM_DATA_ROOT=${placeholder "out"}/share/engine-sim"
    "-DDISCORD_ENABLED=OFF"
  ];

  installPhase = ''
    runHook preInstall

    mkdir -p $out/bin $out/share/engine-sim
    cp engine-sim-app $out/bin/engine-sim

    # Copy assets and engine definitions
    cp -r $src/assets $out/share/engine-sim/
    cp -r $src/es $out/share/engine-sim/

    # Copy required delta-studio basic fonts and shaders
    mkdir -p $out/share/engine-sim/dependencies/submodules/delta-studio/engines/basic
    cp -r $src/dependencies/submodules/delta-studio/engines/basic/fonts \
          $src/dependencies/submodules/delta-studio/engines/basic/shaders \
          $out/share/engine-sim/dependencies/submodules/delta-studio/engines/basic/

    # Wrap binary with graphics library paths
    wrapProgram $out/bin/engine-sim \
      --prefix LD_LIBRARY_PATH : ${pkgs.lib.makeLibraryPath (with pkgs; [
        libGL
        libGLU
        SDL2
        SDL2_image
        libx11
      ])} \
      --set-default ENGINE_SIM_DATA_ROOT $out/share/engine-sim

    runHook postInstall
  '';

  meta = with pkgs.lib; {
    description = "Internal combustion engine simulation designed for acoustic sound generation";
    homepage = "https://github.com/ange-yaghi/engine-sim";
    license = licenses.mit;
    platforms = platforms.linux;
    mainProgram = "engine-sim";
  };
}

