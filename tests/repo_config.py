"""Readers for the hand-written configs the packages share, so tests can compare them.

Plain Python (PyYAML + the standard library): no ROS, no xacro, and none of the git-ignored
model files (CAD export, generated URDF, meshes) are needed.
"""
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parent.parent
SRC = REPO / "ros2_ws" / "src"
CONTROL_XACRO = SRC / "ava_description" / "urdf" / "ava.control.xacro"
SRDF = SRC / "ava_moveit_config" / "config" / "ava.srdf"
JOINT_LIMITS_YAML = SRC / "ava_moveit_config" / "config" / "joint_limits.yaml"

# The arm's joints in J1..J6 order (PCA9685 channel = index).
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

_PROPERTY = re.compile(r'<xacro:property\s+name="([^"]+)"\s+value="([^"]+)"\s*/>')


def _evaluate(value):
    """A property value: a number, a `${...}` expression using only numbers and `pi`, or plain text."""
    if value.startswith("${") and value.endswith("}"):
        return eval(value[2:-1], {"__builtins__": {}}, {"pi": math.pi})  # noqa: S307 - our own file
    try:
        return float(value)
    except ValueError:
        return value  # e.g. servo_release_on_deactivate="false"


def control_properties():
    """Every `<xacro:property>` in ava.control.xacro: numbers evaluated, other values as text."""
    return {name: _evaluate(value) for name, value in _PROPERTY.findall(CONTROL_XACRO.read_text())}


def load_yaml(path):
    return yaml.safe_load(Path(path).read_text())


def srdf_group_states():
    """Named poses from the SRDF: {state name: {joint: value (rad)}}."""
    root = ET.parse(SRDF).getroot()
    return {
        state.get("name"): {j.get("name"): float(j.get("value")) for j in state.findall("joint")}
        for state in root.findall("group_state")
    }
