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

## Configuration

**pipeline**
- `pipeline.queue_size` (default: `32`) — max frames buffered per stage queue

**input**
- `input.source` (**required**) — file path or GStreamer pipeline string (detected by `!`)
- `input.loop` (default: `false`) — reopen source when it ends

**optical_flow**
- `optical_flow.max_corners` (default: `200`) — max corners to track
- `optical_flow.quality_level` (default: `0.01`) — min corner quality relative to strongest
- `optical_flow.min_distance` (default: `10.0`) — min pixel distance between corners
- `optical_flow.tracking_threshold` (default: `0.75`) — fraction of points that must track; below this triggers ORB redetect. Set above `1.0` to always redetect
- `optical_flow.redetect_interval` (default: `30`) — force ORB redetect every N frames regardless

**orb**
- `orb.n_features` (default: `1000`) — max keypoints per frame
- `orb.scale_factor` (default: `1.2`) — image pyramid scale between levels
- `orb.n_levels` (default: `8`) — number of pyramid levels
- `orb.edge_threshold` (default: `31`) — border size with no feature detection, should match `patch_size`
- `orb.patch_size` (default: `31`) — patch size for the BRIEF descriptor
- `orb.min_matches` (default: `10`) — minimum good matches after ratio test to count as a valid detection; frames with no match are dropped

**pictures**
- `pictures.path` (default: `"/tmp/vision"`) — directory of reference images for ORB matching, scanned recursively and kept in sync at runtime

**matching**
- `matching.hamming_ratio_test` (default: `0.75`) — Lowe's ratio test threshold; lower is stricter
- `matching.cross_check` (default: `false`) — only keep matches consistent in both directions

**ed-ransac**
- `stabilizer.lowe_ratio` (default: `0.75`) — Lowe's ratio test threshold for feature matching
- `stabilizer.reprojection_threshold` (default: `3.0`) — RANSAC reprojection error threshold in pixels
- `stabilizer.ed_threshold` (default: `0.5`) — edge detection threshold in pixels
- `stabilizer.min_inlies` (default: `10`) — minimum inliers required for a valid transform
- `stabilizer.smooth_radius` (default: `15`) — number of trailing frames used for trajectory smoothing

**pose**
- `pose.enabled` (default: `true`) — enable pose estimation from homography inliers

**output**
- `output.gstreamer_pipeline` (default: `"appsrc ! videoconvert ! autovideosink"`) — GStreamer sink pipeline
- `output.width` (default: `1280`) — output width in pixels
- `output.height` (default: `720`) — output height in pixels
- `output.fps` (default: `30`) — output frame rate

**logging**
- `logging.level` (default: `info`) — `debug`, `info`, `warn`, or `error`
- `logging.file` (default: `""`) — log file path, empty means stdout only

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
