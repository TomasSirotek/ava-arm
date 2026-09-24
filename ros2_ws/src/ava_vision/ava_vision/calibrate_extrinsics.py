#!/usr/bin/env python3
"""One-shot empirical calibration of base_link -> camera_optical_frame.

The 4 table markers (aruco_marker_1..4) sit at KNOWN base_link positions
(measured by the user, supplied via `table_markers_expected`). The ArUco detector
broadcasts the marker CENTERS in the camera optical frame
(`camera_optical_frame -> aruco_marker_<id>`), independent of the (possibly wrong)
static base_link->camera transform. From >=3 correspondences
(marker-in-optical  <->  known marker-in-base) we solve the rigid transform
base_link->camera_optical_frame with Kabsch/SVD, then write it to
camera_extrinsics.yaml (camera_x/y/z + camera_roll/pitch/yaw) so the static
transform publisher in vision_bringup uses the TRUE pose. No more guessing.

Run once with the camera up and all 4 table markers visible:

  ros2 run ava_vision calibrate_extrinsics --ros-args \
    --params-file <install>/ava_vision/config/aruco_params.yaml \
    -p output_file:=<src>/ava_vision/config/camera_extrinsics.yaml
"""
import datetime
import json
import math
import os
import shutil

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class CalibrateExtrinsics(Node):
    def __init__(self):
        super().__init__('calibrate_extrinsics')

        self.declare_parameter('camera_optical_frame', 'camera_optical_frame')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('table_marker_frames',
                               ['aruco_marker_1', 'aruco_marker_2',
                                'aruco_marker_3', 'aruco_marker_4'])
        self.declare_parameter('table_markers_expected', '')
        self.declare_parameter('marker_z', 0.0)
        self.declare_parameter('samples', 200)
        self.declare_parameter('min_markers', 3)
        self.declare_parameter('min_samples_per_marker', 20)
        self.declare_parameter('timeout_sec', 40.0)
        self.declare_parameter('expected_translation', [0.0, -0.048, 0.535])
        self.declare_parameter('output_file', '')

        self.camera_frame = self.get_parameter('camera_optical_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.marker_frames = list(self.get_parameter('table_marker_frames').value)
        self.marker_z = float(self.get_parameter('marker_z').value)
        self.samples = int(self.get_parameter('samples').value)
        self.min_markers = int(self.get_parameter('min_markers').value)
        self.min_spm = int(self.get_parameter('min_samples_per_marker').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.expected_t = np.array(
            list(self.get_parameter('expected_translation').value), dtype=float)
        self.output_file = self.get_parameter('output_file').value

        expected_str = self.get_parameter('table_markers_expected').value
        self.expected = {}
        if expected_str:
            try:
                raw = json.loads(expected_str)
                self.expected = {str(k): [float(v[0]), float(v[1])]
                                 for k, v in raw.items()}
            except Exception as e:
                self.get_logger().error(f'table_markers_expected parse failed: {e}')
        if not self.expected:
            self.get_logger().error(
                'No table_markers_expected given — pass it via --params-file. Aborting.')
            raise SystemExit(2)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.collected = {f: [] for f in self.marker_frames}
        self.last_stamp = {f: None for f in self.marker_frames}
        self.finished = False
        self._ticks = 0
        self._max_ticks = int(self.timeout_sec / 0.05)

        self.get_logger().info(
            'Kalibrierung gestartet. Sammle Marker-Posen in '
            f'{self.camera_frame} … (alle 4 Tisch-Marker sichtbar halten)')
        self.timer = self.create_timer(0.05, self._tick)

    # --------------------------------------------------------------------- #
    def _tick(self):
        self._ticks += 1
        for frame in self.marker_frames:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.camera_frame, frame, Time())
            except (LookupException, ConnectivityException, ExtrapolationException):
                continue
            stamp = (tf.header.stamp.sec, tf.header.stamp.nanosec)
            if stamp == self.last_stamp[frame]:
                continue          # no new detection since last tick
            self.last_stamp[frame] = stamp
            t = tf.transform.translation
            self.collected[frame].append([t.x, t.y, t.z])

        have = [f for f in self.marker_frames if len(self.collected[f]) >= self.samples]
        if len(have) == len([f for f in self.marker_frames if f.split('_')[-1] in self.expected]):
            self._done()
        elif self._ticks % 20 == 0:
            counts = {f.split('_')[-1]: len(self.collected[f]) for f in self.marker_frames}
            self.get_logger().info(f'  Samples je Marker: {counts}')
            if self._ticks >= self._max_ticks:
                self.get_logger().warn('Timeout — rechne mit dem, was da ist.')
                self._done()

    # --------------------------------------------------------------------- #
    def _done(self):
        self.timer.cancel()
        self.finished = True
        try:
            self._solve_and_write()
        except SystemExit:
            raise
        except Exception as e:
            self.get_logger().error(f'Calibration failed: {e}')

    def _solve_and_write(self):
        ids, P_opt, P_base = [], [], []
        for frame in self.marker_frames:
            mid = frame.split('_')[-1]
            pts = self.collected[frame]
            if mid not in self.expected:
                continue
            if len(pts) < self.min_spm:
                self.get_logger().warn(
                    f'  Marker {mid}: only {len(pts)} samples (<{self.min_spm}) — skipped.')
                continue
            mean = np.mean(np.array(pts), axis=0)
            std = np.std(np.array(pts), axis=0)
            ids.append(mid)
            P_opt.append(mean)
            ex = self.expected[mid]
            P_base.append([ex[0], ex[1], self.marker_z])
            self.get_logger().info(
                f'  Marker {mid}: optical mean=({mean[0]:+.3f},{mean[1]:+.3f},{mean[2]:+.3f}) '
                f'std=({std[0]*1000:.1f},{std[1]*1000:.1f},{std[2]*1000:.1f})mm '
                f'n={len(pts)}')

        if len(ids) < self.min_markers:
            self.get_logger().error(
                f'Nur {len(ids)} brauchbare Marker (<{self.min_markers}). '
                'Check visibility/lighting. Aborting.')
            raise SystemExit(3)

        P_opt = np.array(P_opt)
        P_base = np.array(P_base)

        # Reject collinear configuration (rank of in-plane spread).
        s_base = np.linalg.svd(P_base - P_base.mean(0), compute_uv=False)
        if s_base[1] < 1e-3:
            self.get_logger().error('Markers (nearly) collinear — rotation not solvable. Aborting.')
            raise SystemExit(4)

        R, t = self._kabsch(P_opt, P_base)        # base = R @ opt + t

        # Residuals.
        pred = (R @ P_opt.T).T + t
        res = np.linalg.norm(pred - P_base, axis=1)
        rms = float(np.sqrt(np.mean(res ** 2)))

        # Euler (tf2 convention R = Rz(yaw) Ry(pitch) Rx(roll) == scipy intrinsic ZYX).
        yaw, pitch, roll = Rotation.from_matrix(R).as_euler('ZYX')
        quat = Rotation.from_matrix(R).as_quat()   # [x, y, z, w]

        self.get_logger().info('—' * 60)
        self.get_logger().info('KALIBRIER-ERGEBNIS base_link -> camera_optical_frame:')
        self.get_logger().info(
            f'  camera_x={t[0]:+.4f}  camera_y={t[1]:+.4f}  camera_z={t[2]:+.4f}')
        self.get_logger().info(
            f'  camera_roll={roll:+.5f}  camera_pitch={pitch:+.5f}  camera_yaw={yaw:+.5f}')
        self.get_logger().info(
            f'  quat(xyzw)=({quat[0]:+.4f},{quat[1]:+.4f},{quat[2]:+.4f},{quat[3]:+.4f})')
        for mid, r in zip(ids, res):
            self.get_logger().info(f'    Marker {mid}: Residuum {r*100:.2f} cm')
        self.get_logger().info(f'  RMS = {rms*100:.2f} cm   (Marker: {",".join(ids)})')

        # Sanity checks.
        ok = True
        if rms > 0.02:
            self.get_logger().warn(
                f'  ⚠ RMS {rms*100:.1f} cm > 2 cm — check measurements/detection, do NOT use blindly!')
            ok = False
        if t[2] <= 0.0:
            self.get_logger().warn(f'  ⚠ camera_z={t[2]:.3f} <= 0 (the camera should be above the table)!')
            ok = False
        if R[2, 2] >= 0.0:
            self.get_logger().warn(
                f'  ⚠ R[2,2]={R[2,2]:.3f} >= 0 (Optik schaut nicht nach unten) — Spiegelung?')
            ok = False
        dt = np.linalg.norm(t - self.expected_t)
        self.get_logger().info(
            f'  Quer-Check Translation vs. gemessen {tuple(round(float(v),3) for v in self.expected_t)}: '
            f'Abweichung {dt*100:.1f} cm')
        if dt > 0.10:
            self.get_logger().warn(
                f'  ⚠ Translation {dt*100:.0f} cm away from the manual measurement — check plausibility.')
        self.get_logger().info('—' * 60)

        self._write_yaml(t, roll, pitch, yaw, quat, rms, ids, res, ok)

    @staticmethod
    def _kabsch(P_src, P_dst):
        """Rigid transform mapping P_src -> P_dst: dst = R @ src + t."""
        c_s = P_src.mean(axis=0)
        c_d = P_dst.mean(axis=0)
        H = (P_src - c_s).T @ (P_dst - c_d)
        U, _, Vt = np.linalg.svd(H)
        d = np.sign(np.linalg.det(Vt.T @ U.T))
        D = np.diag([1.0, 1.0, d])
        R = Vt.T @ D @ U.T
        t = c_d - R @ c_s
        return R, t

    def _resolve_output(self):
        if self.output_file:
            return self.output_file
        # Default: derive the SOURCE config path from the install layout so a
        # subsequent `colcon build` keeps the calibration.
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('ava_vision')
        if '/install/' in share:
            ws = share.split('/install/')[0]
            src_cfg = os.path.join(ws, 'src', 'ava_vision',
                                   'config', 'camera_extrinsics.yaml')
            if os.path.isdir(os.path.dirname(src_cfg)):
                return src_cfg
        return os.path.join(share, 'config', 'camera_extrinsics.yaml')

    def _write_yaml(self, t, roll, pitch, yaw, quat, rms, ids, res, ok):
        out = self._resolve_output()
        ts = datetime.datetime.now().isoformat(timespec='seconds')
        resline = '  '.join(f'{m}:{r*100:.2f}cm' for m, r in zip(ids, res))
        content = (
            '# Static transform: base_link -> camera_optical_frame\n'
            '#\n'
            f'# AUTO-GENERATED by calibrate_extrinsics on {ts}.\n'
            f'# Solved (Kabsch/SVD) from table markers {",".join(ids)} at their\n'
            '# measured base_link positions. RMS = '
            f'{rms*100:.2f} cm  (per-marker: {resline}).\n'
            f'# Quaternion (xyzw) = [{quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f}].\n'
            '# Re-run `ros2 run ava_vision calibrate_extrinsics` if the\n'
            '# camera is moved, then `colcon build --packages-select ava_vision`.\n'
            '#\n'
            '# Loaded by vision_bringup.launch.py -> tf2_ros static_transform_publisher\n'
            '# (--roll/--pitch/--yaw path; pitch~0/roll~pi for a downward camera => no gimbal lock).\n'
            '\n'
            f'camera_x: {t[0]:.6f}\n'
            f'camera_y: {t[1]:.6f}\n'
            f'camera_z: {t[2]:.6f}\n'
            f'camera_roll: {roll:.6f}\n'
            f'camera_pitch: {pitch:.6f}\n'
            f'camera_yaw: {yaw:.6f}\n'
        )
        try:
            if os.path.exists(out):
                shutil.copyfile(out, out + '.bak')
            with open(out, 'w') as f:
                f.write(content)
            self.get_logger().info(f'✅ camera_extrinsics.yaml geschrieben: {out}')
            if os.path.exists(out + '.bak'):
                self.get_logger().info(f'   (Backup: {out}.bak)')
            self.get_logger().info(
                'Jetzt: colcon build --packages-select ava_vision, '
                'Restart the vision stack (Ctrl-C), check the verification RMS (<1 cm).')
            if not ok:
                self.get_logger().warn(
                    'WARNING: not all sanity checks passed — review the result before use.')
        except Exception as e:
            self.get_logger().error(f'Writing failed ({out}): {e}')
            self.get_logger().error('YAML content (apply manually):\n' + content)


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateExtrinsics()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
