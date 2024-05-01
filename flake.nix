{
  description = "gbp-rs";
  inputs = {
    # wgsl_analyzer.url = "github:wgsl-analyzer/wgsl-analyzer";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs = {
        nixpkgs.follows = "nixpkgs";
        flake-utils.follows = "flake-utils";
      };
    };
  };

  outputs = {
    self,
    nixpkgs,
    rust-overlay,
    flake-utils,
    ...
  } @ inputs:
    inputs.flake-utils.lib.eachDefaultSystem (system: let
      overlays = [(import rust-overlay)];
      pkgs = import inputs.nixpkgs {inherit system overlays;};
      rust-extensions = [
        "rust-src"
        "rust-analyzer"
        "llvm-tools-preview" # used with `cargo-pgo`
      ];
      rust-targets = ["wasm32-unknown-unknown"];
      bevy-deps = with pkgs; [
        udev
        alsa-lib
        vulkan-loader
        xorg.libX11
        xorg.libXcursor
        xorg.libXi
        xorg.libXrandr
        libxkbcommon
        wayland
        egl-wayland
        # wgsl-analyzer-pkgs.wgsl_analyzer
        # wgsl_analyzer.packages.${system}
        # wgsl_analyzer.outputs.packages.${system}.default
      ];
      # wgsl-analyzer-pkgs = import inputs.wgsl_analyzer {inherit system;};
      cargo-subcommands = with pkgs; [
        cargo-bloat
        cargo-expand
        cargo-outdated
        cargo-show-asm
        cargo-make
        cargo-modules
        cargo-nextest
        cargo-rr
        cargo-udeps
        cargo-watch
        cargo-wizard
        cargo-pgo
        # cargo-tree

        #   # cargo-profiler
        #   # cargo-feature
      ];
      rust-deps = with pkgs;
        [
          # rustup
          taplo # TOML formatter and LSP
          bacon
          mold # A Modern Linker
          clang # For linking
          gdb # debugger
          # lldb # debugger
          rr # time-traveling debugger
        ]
        ++ cargo-subcommands;
    in
      with pkgs; {
        formatter.${system} = pkgs.alejandra;
        devShells.default = pkgs.mkShell rec {
          nativeBuildInputs = with pkgs; [
            pkgs.pkg-config
          ];
          buildInputs =
            [
              # (rust-bin.stable.latest.default.override
              #   {
              #     extensions = rust-extensions;
              #     targets = rust-targets;
              #   })
              # (rust-bin.beta.latest.default.override {
              #     extensions = ["rust-src" "rust-analyzer"];
              # })
              (
                rust-bin.selectLatestNightlyWith (toolchain:
                  toolchain.default.override {
                    extensions =
                      rust-extensions
                      ++ [
                        "rustc-codegen-cranelift-preview"
                      ];
                    targets = ["wasm32-unknown-unknown"];
                  })
              )
              just
              gh
            ]
            ++ rust-deps ++ bevy-deps;

          LD_LIBRARY_PATH = lib.makeLibraryPath buildInputs;
        };
      });
}
