"""The same six joints must be configured consistently in every package.

A joint missing from a controller, a named pose outside the limits, or a planner allowed to move
faster than the hardware driver lets a servo move all break things only at runtime (in Gazebo or,
worse, on the real arm). These checks catch them on every push.
"""
import pytest

from repo_config import JOINT_LIMITS_YAML, JOINTS, SRC, control_properties, load_yaml, srdf_group_states

PROPS = control_properties()
LIMITS = {j: (PROPS[f"{j}_lower"], PROPS[f"{j}_upper"]) for j in JOINTS}

# Every place that lists the joints a trajectory controller drives.
CONTROLLER_JOINT_LISTS = {
    "Gazebo controllers": (
        SRC / "ava_description/config/gazebo_controllers.yaml",
        ["joint_trajectory_controller", "ros__parameters", "joints"],
    ),
    "MoveIt demo controllers": (
        SRC / "ava_moveit_config/config/ros2_controllers.yaml",
        ["joint_trajectory_controller", "ros__parameters", "joints"],
    ),
    "real-arm controllers": (
        SRC / "ava_hardware/config/real_controllers.yaml",
        ["joint_trajectory_controller", "ros__parameters", "joints"],
    ),
    "MoveIt execution": (
        SRC / "ava_moveit_config/config/moveit_controllers.yaml",
        ["moveit_simple_controller_manager", "joint_trajectory_controller", "joints"],
    ),
}


def test_every_joint_has_an_ordered_range():
    for joint, (lower, upper) in LIMITS.items():
        assert lower < upper, joint


@pytest.mark.parametrize("name", CONTROLLER_JOINT_LISTS)
def test_controller_drives_exactly_the_arm_joints(name):
    path, keys = CONTROLLER_JOINT_LISTS[name]
    node = load_yaml(path)
    for key in keys:
        node = node[key]
    assert sorted(node) == sorted(JOINTS)


def test_moveit_limits_cover_every_joint_and_stay_within_what_the_driver_allows():
    limits = load_yaml(JOINT_LIMITS_YAML)["joint_limits"]
    assert sorted(limits) == sorted(JOINTS)
    for joint, cfg in limits.items():
        # The real-arm driver rate-limits each joint to <joint>_max_velocity (ava.control.xacro);
        # a planner allowed to go faster would plan moves the arm cannot follow.
        assert 0 < cfg["max_velocity"] <= PROPS[f"{joint}_max_velocity"], joint
        assert cfg["max_acceleration"] > 0, joint


@pytest.mark.parametrize("state, joints", srdf_group_states().items())
def test_named_poses_are_inside_the_joint_limits(state, joints):
    for joint, value in joints.items():
        assert joint in LIMITS, f"{state}: unknown joint {joint}"
        lower, upper = LIMITS[joint]
        assert lower <= value <= upper, f"{state}: {joint}={value} outside [{lower}, {upper}]"
