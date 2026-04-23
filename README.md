# AAUSAT6-vision

`cmake`

## Dirs

- `config/` for parameters when running. Can contain multiple configs for experiments
- `src/` for code
- `data/` for keypoint database
- `tests/` for tests
- `scripts/` for scripts (runscripts, experiments, buildscripts)
- `docs/` for markdown docs

## Usage

```
vision <config.yaml> [experiment_overlay.yaml]
```

Experiment overlays are merged on top of the base config. See `config/default.yaml` and `config/experiments/`.

## req.

**Ubuntu**:
```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config libopencv-dev libyaml-cpp-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

# maybe also
sudo apt install -y gstreamer1.0-plugins-{good,bad,ugly} gst-libav
```

**Arch**:
```bash
sudo pacman -Sy --needed base-devel cmake pkgconf opencv yaml-cpp gstreamer gst-plugins-base

# maybe also
sudo pacman -Sy gst-plugins-good gst-plugins-bad gst-plugins-ugly gst-libav
```

## Build
Use `scripts/build.sh` or set up your cmake to do it

Here is also a manual release command:
```bash
cmake -B build/Release -DCMAKE_BUILD_TYPE=Release && cmake --build build/Release
```

## Run

1. Find the script
2. Run with extra configs like `build/Release/src/vision config/default.yaml config/experiments/naitsa.yaml`
