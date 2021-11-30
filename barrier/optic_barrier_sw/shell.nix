{ pkgs ? import <nixpkgs> {} }:

pkgs.mkShell {
  buildInputs = with pkgs; [
    # cross compiler
    pkgsCross.raspberryPi.buildPackages.gcc8

    # symlink the crosscompiler under the name used in CMakeLists.txt
    (runCommandLocal "rpi-cross-gcc-symlink" {} ''
       mkdir -p $out/bin
       ln -s ${pkgsCross.raspberryPi.buildPackages.gcc}/bin/armv6l-unknown-linux-gnueabihf-gcc $out/bin/arm-linux-gnueabihf-gcc
     '')
    # keep this line if you use bash
    bashInteractive
  ];
}
