"""tools/torque_monitor.py judges the Gazebo load tests: its servo table must match the effort
limits the simulation uses, and its OK / HOT / FAIL boundaries must be stall / 2 and stall."""
import sys

import pytest

from repo_config import JOINTS, REPO, control_properties

sys.path.insert(0, str(REPO / "tools"))
import torque_monitor as tm  # noqa: E402 - needs the tools/ path

PROPS = control_properties()


def test_maps_every_arm_joint_to_a_known_servo():
    assert sorted(tm.JOINT_SERVO) == sorted(JOINTS)
    assert set(tm.JOINT_SERVO.values()) <= set(tm.SERVOS)


@pytest.mark.parametrize("joint", JOINTS)
def test_stall_torque_matches_the_urdf_effort_limit(joint):
    # The effort limits sit just above the stall values (rounded to 1 mN·m).
    assert tm.SERVOS[tm.JOINT_SERVO[joint]] == pytest.approx(PROPS[f"{joint}_effort"], abs=1e-3)


@pytest.mark.parametrize("servo", ["MG996R", "SG90"])
def test_status_boundaries(servo):
    stall = tm.SERVOS[servo]
    assert tm.status(0.0, servo) == "OK"
    assert tm.status(stall / 2, servo) == "OK"
    assert tm.status(stall / 2 + 1e-6, servo) == "HOT"
    assert tm.status(stall, servo) == "HOT"
    assert tm.status(stall + 1e-6, servo) == "FAIL"
