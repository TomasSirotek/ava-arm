"""The real-arm calibration in ava.control.xacro, checked with the same maths as the driver.

ava_hardware/AvaServoSystem turns a joint angle q into a pulse width:
    zero_us = center_us - direction * (lower + upper) / 2 * us_per_rad   (ava.ros2control.xacro)
    pulse   = round(zero_us + direction * q * us_per_rad), clamped to [servo_min_us, servo_max_us]
If a joint limit maps outside the clamp, the servo stops early while the reported state (which is
only the last command: the servos give no feedback) keeps moving. These checks keep the whole range
reachable while the values are edited during bench calibration.
"""
import pytest

from repo_config import JOINTS, control_properties

PROPS = control_properties()
US_PER_RAD = PROPS["servo_us_per_rad"]
MIN_US, MAX_US = PROPS["servo_min_us"], PROPS["servo_max_us"]


def pulse(joint, q):
    lower, upper = PROPS[f"{joint}_lower"], PROPS[f"{joint}_upper"]
    direction, center = PROPS[f"{joint}_direction"], PROPS[f"{joint}_center_us"]
    zero = center - direction * (lower + upper) / 2 * US_PER_RAD
    return round(zero + direction * q * US_PER_RAD)


def test_pulse_clamp_is_what_the_driver_and_firmware_accept():
    # The driver refuses min_us/max_us outside 500..2500 (kDeviceMinUs/kDeviceMaxUs).
    assert 500 <= MIN_US < MAX_US <= 2500
    assert US_PER_RAD > 0


def test_each_joint_has_its_own_channel_in_joint_order():
    # Channel = J-number - 1, as in the BOM, the electronics design and the firmware guide.
    assert [int(PROPS[f"{j}_channel"]) for j in JOINTS] == list(range(len(JOINTS)))


@pytest.mark.parametrize("joint", JOINTS)
def test_direction_is_plus_or_minus_one(joint):
    assert PROPS[f"{joint}_direction"] in (1.0, -1.0)


@pytest.mark.parametrize("joint", JOINTS)
def test_whole_joint_range_maps_inside_the_pulse_clamp(joint):
    lower, upper = PROPS[f"{joint}_lower"], PROPS[f"{joint}_upper"]
    pulses = sorted((pulse(joint, lower), pulse(joint, upper)))
    assert MIN_US <= pulses[0] and pulses[1] <= MAX_US, f"{joint}: limits map to {pulses} us"


@pytest.mark.parametrize("joint", JOINTS)
def test_middle_of_the_range_is_the_centre_pulse(joint):
    lower, upper = PROPS[f"{joint}_lower"], PROPS[f"{joint}_upper"]
    assert pulse(joint, (lower + upper) / 2) == round(PROPS[f"{joint}_center_us"])
