# ava_vision

Overhead ArUco perception for the **AVA arm** robot: a single USB webcam, mounted
looking straight down over the table, detects a cube (ArUco marker ID 0 on its top),
computes its position in the robot frame and publishes it on `/cup_pose` for the MoveIt
pick-and-place task. Four table-corner markers (IDs 1–4) serve as the calibration rig
for the camera transform.

## Nodes (executables)

| Executable | Role |
|---|---|
| `camera_node` | USB webcam capture → `camera/image_raw` + `camera/camera_info` (loads intrinsics from `camera_info_url`). |
| `aruco_detector_node` | Detect markers → `aruco/pose`, `aruco/poses`, `aruco/debug_image`; broadcasts TF `camera_optical_frame → aruco_marker_<id>`. |
| `cube_pose_publisher` | TF `world → aruco_marker_0` → `/cup_pose` (PoseStamped) with smoothing and outlier rejection, RViz markers, the table collision object for MoveIt, and a table-marker verification log (RMS). |
| `camera_calibration_node` | Interactive checkerboard **intrinsics** calibration → YAML. |
| `calibrate_extrinsics` | One-shot **extrinsics** calibration from the four table markers; writes `config/camera_extrinsics.yaml`. |
| `generate_markers` | Generate printable ArUco markers. |

## TF tree

```
world == base_footprint ──(static TF from config/camera_extrinsics.yaml)──
   camera_optical_frame ──(detector)── aruco_marker_0 (cube), aruco_marker_1..4 (table)
```

The code and config still name the robot frame `base_link`, but since the CAD redesign the table
plane is `base_footprint` z = 0. Recalibrate the extrinsics against `base_footprint` before
trusting `/cup_pose`.

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

# 4) EXTRINSICS: measure your four table-marker centers in base_footprint (meters),
#    put them into config/aruco_params.yaml (table_markers_expected), then:
ros2 run ava_vision calibrate_extrinsics --ros-args \
    --params-file <install>/config/aruco_params.yaml \
    -p output_file:=src/ava_vision/config/camera_extrinsics.yaml
#   rebuild + restart vision. The verification log should show RMS < ~1 cm.
```

The pick-and-place demo lives in `ava_moveit_config`.

## Key config

- `config/aruco_params.yaml` — marker sizes/IDs, dictionary, camera id/resolution,
  cube-pose-publisher parameters, expected table-marker positions.
- `config/camera_extrinsics.yaml` — static robot base → `camera_optical_frame` (still named `base_link`)
  (written by `calibrate_extrinsics`; recalibrate after any camera move).

## Notes

- ArUco dictionary is `DICT_4X4_50` for all markers.
- A 31 mm marker at 0.5 m is small: use 720p+ and good, even lighting.
- If the camera fails to open, check which `/dev/video*` device it enumerated as after
  replugging (laptops: the internal webcam also claims indices).
