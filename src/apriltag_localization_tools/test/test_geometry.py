import math

from apriltag_localization_tools.geometry import wrap_angle, yaw_from_quaternion


def test_wrap_angle_folds_into_range():
    assert math.isclose(wrap_angle(2 * math.pi + 0.3), 0.3, abs_tol=1e-9)
    assert math.isclose(wrap_angle(-2 * math.pi - 0.3), -0.3, abs_tol=1e-9)
    assert math.isclose(wrap_angle(0.5), 0.5, abs_tol=1e-9)
    assert abs(wrap_angle(3 * math.pi)) <= math.pi + 1e-9


def test_yaw_from_identity_is_zero():
    assert math.isclose(yaw_from_quaternion(0.0, 0.0, 0.0, 1.0), 0.0, abs_tol=1e-9)


def test_yaw_from_ninety_degrees():
    half = math.sqrt(0.5)
    assert math.isclose(
        yaw_from_quaternion(0.0, 0.0, half, half), math.pi / 2, abs_tol=1e-6)
