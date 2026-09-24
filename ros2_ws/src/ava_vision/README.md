# ava_vision

Overhead ArUco perception for the **AVA arm** robot: a single USB webcam, mounted
looking straight down over the table, detects a cube (ArUco marker ID 0 on its top),
computes its position in the robot `base_link`/`world` frame and publishes it on
`/cup_pose` for the MoveIt pick-and-place task. Four table-corner markers (IDs 1–4)
serve as the calibration rig for the camera transform. This is the perception half of
**Part 4** of the series (`docs/part4-computer-vision-pick-and-place.md`).

## Nodes (executables)

| Executable | Role |
|---|---|
| `camera_node` | USB webcam capture → `camera/image_raw` + `camera/camera_info` (loads intrinsics from `camera_info_url`). |
| `aruco_detector_node` | Detect markers → `aruco/pose`, `aruco/poses`, `aruco/debug_image`; broadcasts TF `camera_optical_frame → aruco_marker_<id>`. |
| `cube_pose_publisher` | TF `world → aruco_marker_0` → `/cup_pose` (PoseStamped) with EWMA smoothing + outlier rejection, RViz markers, the table collision object for MoveIt, and a continuous table-marker verification log (RMS). |
| `camera_calibration_node` | Interactive checkerboard **intrinsics** calibration → YAML. |
| `calibrate_extrinsics` | One-shot **extrinsics** calibration: solves the camera pose from the four table markers (Kabsch/SVD) and writes `config/camera_extrinsics.yaml`. |
| `generate_markers` | Generate printable ArUco markers. |

## TF tree

```
world == base_link ──(static TF from config/camera_extrinsics.yaml)──
   camera_optical_frame ──(detector)── aruco_marker_0 (cube), aruco_marker_1..4 (table)
```

## Quick start

```bash
# 0) Build
colcon build --packages-select ava_vision && source install/setup.bash

# 1) Generate + print markers (cube = ID 0 @ 31 mm; table corners = IDs 1-4, larger)
ros2 run ava_vision generate_markers --marker-ids 0,1,2,3,4 --size 600 --dictionary 4X4_50

# 2) INTRINSICS: checkerboard calibration (same resolution you will run at!)
ros2 launch ava_vision camera_calibration.launch.py \
    camera_id:=0 checkerboard_width:=9 checkerboard_height:=6 square_size:=0.025
#   SPACE=capture (>=20 poses), c=calibrate, s=save -> ./calibration/*.yaml

# 3) Start perception with your intrinsics
ros2 launch ava_vision vision_bringup.launch.py \
    camera_info_url:=/abs/path/to/your_calibration.yaml
#   check: rqt_image_view /aruco/debug_image  (all 5 markers boxed)

# 4) EXTRINSICS: measure your four table-marker centers in base_link (meters),
#    put them into config/aruco_params.yaml (table_markers_expected), then:
ros2 run ava_vision calibrate_extrinsics --ros-args \
    --params-file <install>/config/aruco_params.yaml \
    -p output_file:=src/ava_vision/config/camera_extrinsics.yaml
#   rebuild + restart vision. The verification log should show RMS < ~1 cm.
```

The full pick-and-place (simulation and real) lives in `ava_moveit_config` —
see that package's README.

## Key config

- `config/aruco_params.yaml` — marker sizes/IDs, dictionary, camera id/resolution,
  cube-pose-publisher parameters (smoothing, outlier jump threshold, table geometry),
  expected table-marker positions.
- `config/camera_extrinsics.yaml` — static `base_link → camera_optical_frame`
  (written by `calibrate_extrinsics`; device-specific — recalibrate after any camera move).

## Notes

- ArUco dictionary is `DICT_4X4_50` for all markers.
- A 31 mm marker at 0.5 m is small: use 720p+, good and even lighting. Sub-pixel corner
  refinement is enabled.
- The cube marker sits on the cube **top**; the publisher reports a fixed cube-center
  height, the grasp height itself is a task parameter.
- The outlier filter rejects single-frame jumps > `min_position_jump` and re-seeds
  automatically when the cube is genuinely moved. Sanity-check the raw detection with
  `ros2 run tf2_ros tf2_echo world aruco_marker_0` if `/cup_pose` looks stale.
- If the camera fails to open, check which `/dev/video*` device it enumerated as after
  replugging (laptops: the internal webcam also claims indices).
