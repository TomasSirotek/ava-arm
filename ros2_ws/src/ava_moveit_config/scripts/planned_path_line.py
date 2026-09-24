#!/usr/bin/env python3
"""Visualisiert die GEPLANTE Endeffektor-Bahn als Linie.

Abonniert /display_planned_path (moveit_msgs/DisplayTrajectory), berechnet pro
Trajektorienpunkt die Pose von gripper_base_1 ueber den /compute_fk-Service und
publiziert die Bahn als:
  - nav_msgs/Path           auf /planned_eef_path          (RViz: Add -> Path)
  - visualization_msgs/Marker (LINE_STRIP) auf /planned_eef_path_marker (RViz: Add -> Marker)

WICHTIG: Die FK-Service-Aufrufe laufen im main()-Loop, NICHT im Subscription-
Callback. Wuerde man rclpy.spin_until_future_complete() aus einem Callback heraus
aufrufen, spinnt man den Node verschachtelt aus seinem eigenen Callback
(Re-Entrancy) und Service-Antworten gehen verloren.
"""
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory, RobotState
from moveit_msgs.srv import GetPositionFK
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker

EEF_LINK = "gripper_base_1"
BASE_FRAME = "world"
MAX_FK_POINTS = 250   # bei sehr langen Trajektorien gleichmaessig ausduennen


class PlannedPathLine(Node):
    def __init__(self):
        super().__init__("planned_path_line")
        self.fk_cli = self.create_client(GetPositionFK, "/compute_fk")
        self.path_pub = self.create_publisher(Path, "/planned_eef_path", 1)
        self.marker_pub = self.create_publisher(Marker, "/planned_eef_path_marker", 1)
        self.create_subscription(DisplayTrajectory, "/display_planned_path", self._cb, 1)

        self.pending = None        # zuletzt empfangene Trajektorie (im main verarbeitet)
        self.last_path = None       # zum periodischen Re-Publish
        self.last_marker = None
        # Re-publish so an RViz display added later still sees the line
        self.create_timer(1.0, self._republish)

        self.get_logger().info("Waiting for /compute_fk ...")
        self.fk_cli.wait_for_service()
        self.get_logger().info("Bereit. Abonniere /display_planned_path "
                               "-> /planned_eef_path (+ _marker)")

    def _cb(self, msg: DisplayTrajectory):
        # nur speichern; Verarbeitung erfolgt im main()-Loop (keine Re-Entrancy)
        self.pending = msg

    def _republish(self):
        if self.last_path is not None:
            self.path_pub.publish(self.last_path)
        if self.last_marker is not None:
            self.marker_pub.publish(self.last_marker)

    def _fk(self, joint_names, positions, base_state):
        """FK fuer EEF_LINK bei gegebener Gelenkstellung. Liefert Point oder None."""
        req = GetPositionFK.Request()
        req.header.frame_id = BASE_FRAME
        req.fk_link_names = [EEF_LINK]
        # vollstaendigen Startzustand uebernehmen, Arm-Joints ueberschreiben
        names = list(base_state.name)
        pos = list(base_state.position)
        idx = {n: i for i, n in enumerate(names)}
        for n, p in zip(joint_names, positions):
            if n in idx:
                pos[idx[n]] = p
            else:
                names.append(n)
                pos.append(p)
        rs = RobotState()
        rs.joint_state.name = names
        rs.joint_state.position = pos
        req.robot_state = rs

        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        res = fut.result()
        if res is None or res.error_code.val != 1 or not res.pose_stamped:
            return None
        return res.pose_stamped[0].pose.position

    def process(self, msg: DisplayTrajectory):
        if not msg.trajectory:
            return
        start = msg.trajectory_start.joint_state
        # alle Punkte ueber alle (Teil-)Trajektorien einsammeln
        all_pts = []
        for rt in msg.trajectory:
            jt = rt.joint_trajectory
            for p in jt.points:
                all_pts.append((jt.joint_names, p.positions))

        # ggf. gleichmaessig ausduennen
        n = len(all_pts)
        if n > MAX_FK_POINTS:
            stride = (n + MAX_FK_POINTS - 1) // MAX_FK_POINTS
            sampled = all_pts[::stride]
            if sampled[-1] is not all_pts[-1]:
                sampled.append(all_pts[-1])   # Endpunkt immer mitnehmen
            self.get_logger().info(f"Trajektorie hat {n} Punkte -> auf {len(sampled)} ausgeduennt")
            all_pts = sampled

        pts = []
        for jn, positions in all_pts:
            pos = self._fk(jn, positions, start)
            if pos is not None:
                pts.append(pos)

        if not pts:
            self.get_logger().warn("Keine FK-Punkte berechnet (compute_fk ohne Loesung?)")
            return

        self._build_and_publish(pts)
        self.get_logger().info(f"EEF-Bahn publiziert: {len(pts)} Punkte")

    def _build_and_publish(self, pts):
        now = self.get_clock().now().to_msg()

        path = Path()
        path.header.frame_id = BASE_FRAME
        path.header.stamp = now
        for pt in pts:
            ps = PoseStamped()
            ps.header.frame_id = BASE_FRAME
            ps.header.stamp = now
            ps.pose.position = pt
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        m = Marker()
        m.header.frame_id = BASE_FRAME
        m.header.stamp = now
        m.ns = "planned_eef_path"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.006           # Linienbreite in m
        m.color.r = 0.1
        m.color.g = 0.6
        m.color.b = 1.0
        m.color.a = 1.0
        m.pose.orientation.w = 1.0
        m.points = [Point(x=pt.x, y=pt.y, z=pt.z) for pt in pts]

        self.last_path = path
        self.last_marker = m
        self.path_pub.publish(path)
        self.marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = PlannedPathLine()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.pending is not None:
                traj = node.pending
                node.pending = None
                node.process(traj)     # FK + Publish im main-Kontext (keine Re-Entrancy)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
