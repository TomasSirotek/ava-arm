#!/usr/bin/env python3
# TODO(so101-migration): written for the OLD OmArm Zero kinematics
# (5-DOF, wrist roll BEFORE wrist flex, two-finger gripper). Joint names were
# renamed to the SO-101 convention, but the link offsets and IK math still describe the old
# arm and are NOT valid for the SO-101 model in ava_description. Rewrite before use.
"""
Custom 5-DOF inverse kinematics for the ava arm.

MoveIt/KDL on this arm uses position_only_ik (orientation is ignored), and a full
6-DOF pose IK is over-constrained on 5 DOF and unreliable. For a top-down grasp we
only need position (3) + approach-axis direction (2) = 5 constraints == 5 DOF.

This module solves exactly that with a Levenberg-Marquardt (damped least squares)
iteration over a numpy forward-kinematics built from the URDF chain
base_link -> shoulder_pan..wrist_flex -> gripper_base_1 -> tcp.

Motion reference = the 'tcp' frame, defined in the URDF at the fingertips (+9 cm X,
+1.5 cm Y from gripper_base_1) and rotated rpy(0, pi/2, 0) so tcp +Z == gripper_base_1
+X == the finger/APPROACH axis. That axis CAN stand vertical across the workspace; the
gb1-Z hinge axis never can. Target: place tcp at a 3D position AND align tcp +Z with a
given world direction (default straight down) so tcp +Z lies ON the cube/marker-0 Z axis
(clean vertical top-down grasp). Putting the fingertips (tcp) at the marker leaves room
to grasp.

ik_solve(target_xyz, approach_dir=(0,0,-1), seed=None, align_axis=2) -> list[5] or None.
"""
import math
import os
import xml.etree.ElementTree as ET

import numpy as np

# IK chain ends at 'tcp' (URDF gripper_base_to_tcp: +translation, rpy(0, pi/2, 0)).
# tcp +Z == gripper_base_1 +X == finger/approach axis. tcp = fingertip motion reference.
JOINT_CHAIN = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_roll',
               'wrist_flex', 'gripper_base_to_tcp']
REV_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_roll', 'wrist_flex']


def _rpy_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _axis_R(axis, q):
    """Rotation about unit axis by angle q (Rodrigues)."""
    x, y, z = axis
    c, s, t = math.cos(q), math.sin(q), 1.0 - math.cos(q)
    return np.array([
        [t*x*x + c,   t*x*y - s*z, t*x*z + s*y],
        [t*x*y + s*z, t*y*y + c,   t*y*z - s*x],
        [t*x*z - s*y, t*y*z + s*x, t*z*z + c],
    ])


def _T(R, p):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = p
    return M


class ArmIK:
    def __init__(self, xacro_path=None):
        if xacro_path is None:
            from ament_index_python.packages import get_package_share_directory
            xacro_path = os.path.join(
                get_package_share_directory('ava_description'),
                'urdf', 'ava.xacro')
        import xacro
        urdf_xml = xacro.process_file(xacro_path).toxml()
        root = ET.fromstring(urdf_xml)

        joints = {j.get('name'): j for j in root.findall('joint')}
        self.chain = []          # list of dicts in order
        self.limits = []         # (lower, upper) per revolute joint
        for name in JOINT_CHAIN:
            j = joints[name]
            origin = j.find('origin')
            xyz = [float(v) for v in (origin.get('xyz', '0 0 0')).split()]
            rpy = [float(v) for v in (origin.get('rpy', '0 0 0')).split()]
            entry = {
                'name': name,
                'type': j.get('type'),
                'xyz': np.array(xyz),
                'R0': _rpy_to_R(*rpy),
            }
            if j.get('type') == 'revolute':
                ax = [float(v) for v in j.find('axis').get('xyz').split()]
                ax = np.array(ax)
                entry['axis'] = ax / np.linalg.norm(ax)
                lim = j.find('limit')
                self.limits.append((float(lim.get('lower')), float(lim.get('upper'))))
            self.chain.append(entry)
        self.limits = np.array(self.limits)   # (5,2)

    # ------------------------------------------------------------------ #
    def fk(self, q):
        """Forward kinematics base_link -> tcp (gripper_base_1 + translation).
        Returns 4x4 transform."""
        T = np.eye(4)
        qi = 0
        for e in self.chain:
            T = T @ _T(e['R0'], e['xyz'])
            if e['type'] == 'revolute':
                T = T @ _T(_axis_R(e['axis'], q[qi]), np.zeros(3))
                qi += 1
        return T

    def fk_pos_axis(self, q, axis_idx=2):
        """Return the tcp position and the chosen tcp axis (0=X,1=Y,2=Z) in world.
        Default Z: the axis we align with 'approach_dir' (tcp +Z = gb1-X = finger/
        approach axis, to be put straight down ON the marker-0 Z)."""
        T = self.fk(q)
        return T[:3, 3].copy(), T[:3, axis_idx].copy()

    # ------------------------------------------------------------------ #
    @staticmethod
    def _orient_err(zdir, target):
        """Rotation vector that rotates zdir onto target (handles 180 deg)."""
        zdir = zdir / (np.linalg.norm(zdir) + 1e-12)
        target = target / (np.linalg.norm(target) + 1e-12)
        c = np.cross(zdir, target)
        s = np.linalg.norm(c)
        d = float(np.dot(zdir, target))
        angle = math.atan2(s, d)
        if s > 1e-6:
            axis = c / s
        elif d > 0:
            return np.zeros(3)            # already aligned
        else:
            # antiparallel: pick any perpendicular axis
            perp = np.array([1.0, 0, 0])
            if abs(zdir[0]) > 0.9:
                perp = np.array([0, 1.0, 0])
            axis = np.cross(zdir, perp)
            axis /= np.linalg.norm(axis)
        return angle * axis

    def _residual(self, q, p_target, dir_target, w_orient, axis_idx):
        p, a = self.fk_pos_axis(q, axis_idx)
        e_pos = p - p_target
        e_rot = w_orient * self._orient_err(a, dir_target)
        return np.concatenate([e_pos, e_rot])

    def _metrics(self, q, p_target, dir_target, axis_idx):
        p, a = self.fk_pos_axis(q, axis_idx)
        a = a / np.linalg.norm(a)
        pos_err = float(np.linalg.norm(p - p_target))
        ang_err = math.degrees(math.atan2(
            np.linalg.norm(np.cross(a, dir_target)), float(np.dot(a, dir_target))))
        return pos_err, ang_err

    # ------------------------------------------------------------------ #
    def ik_solve(self, target_xyz, approach_dir=(0, 0, -1), seed=None,
                 align_axis=2, w_orient=0.1, pos_tol=0.01, ang_tol=15.0,
                 quick=False):
        """Solve for [Rev1..Rev5] via bounded least-squares from many seeds.

        Places the tcp at target_xyz AND aligns its axis 'align_axis' (2=Z default =
        finger/approach axis) with 'approach_dir' (default straight down), so tcp +Z
        lies on the marker-0 Z line. Rotation about that axis is free. Returns
        list[5] joints, or None if no seed reaches pos_tol (m) and ang_tol (deg).

        quick=True: fast FEASIBILITY mode for reachability scans (fewer samples, no
        structured/random restarts) — an unreachable probe returns None in ~1-2 s
        instead of ~15 s. May miss borderline solutions; use it only for scans,
        never for the motion itself.
        """
        from scipy.optimize import least_squares
        target_xyz = np.array(target_xyz, dtype=float)
        zt = np.array(approach_dir, dtype=float)
        zt = zt / np.linalg.norm(zt)
        lo, hi = self.limits[:, 0].copy(), self.limits[:, 1].copy()
        # keep seeds strictly inside bounds for the TRF solver
        loi, hii = lo + 1e-4, hi - 1e-4

        rng = np.random.default_rng(0)
        seeds = []
        if seed is not None:
            seeds.append(np.clip(np.array(seed, dtype=float), loi, hii))
        # Sample-then-refine: cheaply score many random configs by FK (position +
        # orientation residual) and seed the solver with the most promising ones.
        # Finds solutions that fixed structured seeds miss (e.g. tcp +Z vertical
        # with the finger offset).
        samp = loi + rng.random((400 if quick else 1500, 5)) * (hii - loi)
        scored = []
        for qs in samp:
            p, a = self.fk_pos_axis(qs, align_axis)
            an = a / (np.linalg.norm(a) + 1e-12)
            r = (np.linalg.norm(p - target_xyz)
                 + 0.05 * math.acos(max(-1.0, min(1.0, float(np.dot(an, zt))))))
            scored.append((r, qs))
        scored.sort(key=lambda t: t[0])
        seeds.extend(qs for _, qs in scored[:4 if quick else 12])
        if not quick:
            # Plus a structured base-yaw sweep + a few random restarts.
            for r1 in np.linspace(loi[0], hii[0], 9):
                seeds.append(np.array([r1, -1.0, -2.0, -1.57, -1.0]))
                seeds.append(np.array([r1, -2.0, -0.5, -1.57, -0.8]))
                # NEUTRAL-roll family prototype (Rev4 ~ 0, Rev5 bends instead) —
                # without these seeds the solver only ever finds the fully-twisted
                # Rev4 ~ -180 family and the posture preference has nothing to pick.
                seeds.append(np.array([r1, -1.9, -0.1, -0.05, -2.8]))
            for _ in range(12):
                seeds.append(loi + rng.random(5) * (hii - loi))

        seed_arr = None if seed is None else np.clip(np.array(seed, dtype=float), lo, hi)
        best_q, best_score = None, float('inf')
        for q0 in seeds:
            q0 = np.clip(q0, loi, hii)
            try:
                res = least_squares(
                    self._residual, q0, bounds=(lo, hi),
                    args=(target_xyz, zt, w_orient, align_axis),
                    method='trf', xtol=1e-10, ftol=1e-10, max_nfev=200)
            except Exception:
                continue
            q = res.x
            pos_err, ang_err = self._metrics(q, target_xyz, zt, align_axis)
            if pos_err < pos_tol and ang_err < ang_tol:
                # Among VALID solutions, prefer the one CLOSEST to the seed (current
                # joints) -> small, smooth joint moves between waypoints (pre-grasp ->
                # grasp -> lift -> place). Without this the solver can jump to a far
                # config (same tcp pose, different elbow/wrist branch) -> a huge motion
                # that executes extremely slowly / stalls MoveIt in sim. Continuity is
                # only a tie-breaker; all candidates already meet pos_tol AND ang_tol.
                score = pos_err + 0.0005 * ang_err
                if seed_arr is not None:
                    score += 0.05 * float(np.linalg.norm(q - seed_arr))
                # POSTURE preference (rotation about the vertical approach axis is
                # free): there are exactly TWO vertical wrist families — roll fully
                # twisted (Rev4 ~ -180) or roll NEUTRAL (Rev4 ~ 0, Rev5 bends instead).
                # The twisted family mirrors the lateral tcp offset -> the real
                # gripper landed on the SIDE of the cube (user report). Prefer the
                # NEUTRAL-roll family: penalize |Rev4| beyond ~30 deg.
                score += 0.01 * max(0.0, abs(float(q[3])) - 0.5)
                if score < best_score:
                    best_score, best_q = score, q.copy()
                # EARLY EXIT: excellent accuracy + neutral wrist roll + (near the
                # seed, if one was given). Without this, every solve grinds through
                # all remaining seeds even when the first candidate is already
                # perfect — the linear-descent chain then stalls the arm for tens
                # of seconds ("robot stuck before descending") while it computes.
                if (pos_err < 0.003 and ang_err < 3.0
                        and abs(float(q[3])) < 0.5
                        and (seed_arr is None
                             or float(np.linalg.norm(q - seed_arr)) < 1.0)):
                    break
        return None if best_q is None else best_q.tolist()


def _selftest():
    ik = ArmIK()
    print('Loaded chain:', [e['name'] for e in ik.chain])
    print('Limits (rad):\n', ik.limits)
    targets = [(0.12, 0.0, 0.06), (0.15, 0.05, 0.05),
               (0.15, -0.05, 0.05), (0.18, 0.0, 0.04)]
    for t in targets:
        q = ik.ik_solve(t, approach_dir=(0, 0, -1), align_axis=2)
        if q is None:
            print(f'target {t}: NO SOLUTION')
            continue
        p, a = ik.fk_pos_axis(q, 2)
        ang = math.degrees(math.acos(max(-1, min(1, np.dot(a/np.linalg.norm(a), [0, 0, -1])))))
        print(f'target {t}: q={[round(v,3) for v in q]}  '
              f'pos_err={np.linalg.norm(p-np.array(t))*1000:.1f}mm  Z·down_angle={ang:.1f}deg')


if __name__ == '__main__':
    _selftest()
