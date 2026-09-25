#!/usr/bin/env python3
"""Live joint-torque check against the servo each joint uses.

Reads /joint_states (effort = torque the joint needs, N·m) and compares it with the
servo on that joint:  OK   <= continuous (stall / 2)
                      HOT  <= stall      (holds briefly, overheats / jitters)
                      FAIL >  stall      (cannot hold it)

Usage (ROS sourced, Gazebo running):
    python3 tools/torque_monitor.py            # live table, Ctrl+C to quit
    python3 tools/torque_monitor.py --once     # 2 s average, print once, exit
"""
import argparse
import time

import rclpy
from sensor_msgs.msg import JointState

KGCM_TO_NM = 0.0980665
# Stall torque (kg·cm). MG996R: TowerPro datasheet at 6 V (9.4 at 4.8 V; "13" in shop listings).
# SG90: datasheet at 4.8 V. Check against the datasheet of the servos you actually bought.
SERVOS = {"MG996R": 11.0 * KGCM_TO_NM, "SG90": 1.8 * KGCM_TO_NM}

# Which servo drives which joint. MG996R on the three joints that carry the arm and load.
JOINT_SERVO = {
    "shoulder_pan": "SG90",
    "shoulder_lift": "MG996R",
    "elbow_flex": "MG996R",
    "wrist_flex": "MG996R",
    "wrist_roll": "SG90",
    "gripper": "SG90",
}

SMOOTHING = 0.2  # exponential smoothing of |effort|; Gazebo effort is noisy while moving
COLORS = {"OK": "\033[32m", "HOT": "\033[33m", "FAIL": "\033[31m"}
RESET = "\033[0m"


def status(nm, servo):
    stall = SERVOS[servo]
    return "OK" if nm <= stall / 2 else "HOT" if nm <= stall else "FAIL"


def table(avg, peak):
    lines = [f"{'joint':<14}{'servo':<8}{'now N·m':>9}{'% stall':>9}{'peak N·m':>10}  status",
             "-" * 58]
    for joint, servo in JOINT_SERVO.items():
        if joint not in avg:
            lines.append(f"{joint:<14}{servo:<8}{'-':>9}")
            continue
        nm, stall = avg[joint], SERVOS[servo]
        st = status(nm, servo)
        lines.append(f"{joint:<14}{servo:<8}{nm:>9.3f}{nm / stall * 100:>8.0f}%{peak[joint]:>10.3f}  "
                     f"{COLORS[st]}{st}{RESET}")
    worst = [j for j in avg if status(avg[j], JOINT_SERVO.get(j, "SG90")) == "FAIL"]
    lines.append("")
    lines.append(f"{COLORS['FAIL']}FAILS: {', '.join(worst)}{RESET}" if worst else f"{COLORS['OK']}all joints hold{RESET}")
    lines.append(f"stall: MG996R {SERVOS['MG996R']:.3f} N·m, SG90 {SERVOS['SG90']:.3f} N·m · continuous = stall / 2")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--once", action="store_true", help="average for 2 s, print once and exit")
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("torque_monitor")
    avg, peak = {}, {}

    def on_msg(msg):
        for name, eff in zip(msg.name, msg.effort):
            e = abs(eff)
            avg[name] = e if name not in avg else avg[name] + SMOOTHING * (e - avg[name])
            peak[name] = max(peak.get(name, 0.0), e)

    node.create_subscription(JointState, "/joint_states", on_msg, 10)
    try:
        if args.once:
            end = time.time() + 2.0
            while time.time() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
            print(table(avg, peak) if avg else "no /joint_states with effort received")
            return
        while rclpy.ok():
            end = time.time() + 0.5
            while time.time() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
            print("\033[2J\033[H" + (table(avg, peak) if avg else "waiting for /joint_states ..."), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
