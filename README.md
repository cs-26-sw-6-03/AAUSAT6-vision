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

**ransac**
- `ransac.method` (default: `EG_RANSAC`) — homography method: `RANSAC`, `EG_RANSAC`, `USAC_MAGSAC`
- `ransac.reproj_threshold` (default: `3.0`) — max reprojection error in pixels to count as inlier
- `ransac.confidence` (default: `0.99`) — confidence level for termination
- `ransac.max_iterations` (default: `2000`) — max RANSAC iterations

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

## Technologies

