"""Quaternion and IMU-metric tests for UT-IMU-01 / UT-IMU-02 scoring.

The load-bearing property proved here is that commanded-rotation scoring uses *body-frame*
relative rotations, which are invariant to whatever world frame the BNO055 fuses into. That
is what makes the shipped `bno055_native` passthrough safe: the axis/sign gates give the same
answer whether the chip's world frame turns out to be NWU, ENU or anything else (BLK-14).
"""

import numpy as np
import pytest

from fixtures.sensor_nano_fixtures import axis_quaternion, quaternion_hold

from billiebot_sensor_tests.sensor_nano.imu_metrics import (
    apply_orientation_convention,
    axis_angle,
    canonicalize,
    dominant_axis,
    finite_fraction,
    mean_quaternion,
    normalize_quaternions,
    quaternion_conjugate,
    quaternion_multiply,
    quaternion_norms,
    relative_rotation_body,
    relative_rotation_world,
    score_commanded_rotation,
    segment_mask,
    stationary_stats,
    vector_magnitudes,
    wrap_angle_rad,
    yaw_from_quaternion,
    yaw_return_error_rad,
)

IDENTITY = np.array([[1.0, 0.0, 0.0, 0.0]])


# --- basics ------------------------------------------------------------------------------

def test_quaternion_norms_of_unit_quaternions_are_one():
    quats = np.vstack([IDENTITY, axis_quaternion(0, 90), axis_quaternion(2, 45)])
    assert quaternion_norms(quats) == pytest.approx(1.0)


def test_quaternion_norms_detects_a_shrunken_quaternion():
    assert quaternion_norms(np.array([[0.5, 0.0, 0.0, 0.0]]))[0] == pytest.approx(0.5)


def test_normalize_leaves_a_zero_quaternion_alone_rather_than_producing_nan():
    # A zero quaternion is the I2C-failure signature the firmware norm-gate rejects; if one
    # ever reached here it must stay visible, not become NaN and poison every later metric.
    result = normalize_quaternions(np.zeros((1, 4)))
    assert np.all(np.isfinite(result))
    assert np.all(result == 0.0)


def test_bad_quaternion_shape_raises():
    with pytest.raises(ValueError):
        quaternion_norms(np.zeros((3, 3)))


def test_canonicalize_resolves_the_double_cover():
    negative = -axis_quaternion(1, 60)
    assert canonicalize(negative)[0, 0] >= 0


def test_conjugate_times_original_is_identity():
    quat = axis_quaternion(1, 37)
    product = quaternion_multiply(quat, quaternion_conjugate(quat))
    assert product[0] == pytest.approx([1.0, 0.0, 0.0, 0.0], abs=1e-12)


def test_quaternion_multiply_is_not_commutative():
    a, b = axis_quaternion(0, 90), axis_quaternion(1, 90)
    assert not np.allclose(quaternion_multiply(a, b), quaternion_multiply(b, a))


# --- axis / angle ------------------------------------------------------------------------

@pytest.mark.parametrize('axis_index', [0, 1, 2])
@pytest.mark.parametrize('angle_deg', [30.0, 90.0, 150.0])
def test_axis_angle_recovers_the_constructed_rotation(axis_index, angle_deg):
    axis, angle = axis_angle(axis_quaternion(axis_index, angle_deg))
    assert np.degrees(angle[0]) == pytest.approx(angle_deg, abs=1e-6)
    index, sign = dominant_axis(axis[0])
    assert (index, sign) == (axis_index, 1.0)


def test_a_negative_rotation_reads_as_a_positive_angle_about_the_negative_axis():
    axis, angle = axis_angle(axis_quaternion(0, -90))
    assert np.degrees(angle[0]) == pytest.approx(90.0)
    assert dominant_axis(axis[0]) == (0, -1.0)


def test_near_zero_rotation_gets_a_zero_axis_instead_of_amplified_noise():
    axis, angle = axis_angle(axis_quaternion(2, 1e-9))
    assert angle[0] == pytest.approx(0.0, abs=1e-7)
    assert np.allclose(axis[0], 0.0)


def test_dominant_axis_rejects_a_non_three_vector():
    with pytest.raises(ValueError):
        dominant_axis([1.0, 0.0])


# --- angle wrapping ----------------------------------------------------------------------

@pytest.mark.parametrize('angle,expected', [
    (0.0, 0.0),
    (np.pi, np.pi),
    (3 * np.pi, np.pi),
    (-3 * np.pi, np.pi),
    (2 * np.pi + 0.5, 0.5),
    (-2 * np.pi - 0.5, -0.5),
])
def test_wrap_angle_maps_into_a_single_turn(angle, expected):
    assert wrap_angle_rad(angle) == pytest.approx(expected, abs=1e-12)


def test_yaw_of_a_pure_z_rotation_matches_the_construction():
    assert np.degrees(yaw_from_quaternion(axis_quaternion(2, 90))[0]) == pytest.approx(90.0)


def test_yaw_difference_across_the_pi_boundary_does_not_blow_up():
    # 170 deg -> -170 deg is a 20 deg turn, not a 340 deg one. Euler subtraction gets this
    # wrong, which is why the scorers use quaternion relative rotations.
    before = axis_quaternion(2, 170)
    after = axis_quaternion(2, -170)
    delta = yaw_from_quaternion(relative_rotation_world(before, after))[0]
    assert abs(np.degrees(wrap_angle_rad(delta))) == pytest.approx(20.0, abs=1e-6)


# --- relative rotation and frame invariance ----------------------------------------------

def test_body_relative_rotation_recovers_the_turn_between_two_orientations():
    start = axis_quaternion(2, 30)
    end = quaternion_multiply(start, axis_quaternion(0, 90))
    axis, angle = axis_angle(relative_rotation_body(start, end))
    assert np.degrees(angle[0]) == pytest.approx(90.0)
    assert dominant_axis(axis[0]) == (0, 1.0)


def test_body_relative_rotation_is_invariant_to_the_world_frame_convention():
    # This is why `bno055_native` passthrough is a safe default: re-expressing both
    # orientations in another world frame cannot change the body-frame turn between them.
    start = axis_quaternion(2, 30)
    end = quaternion_multiply(start, axis_quaternion(1, -75))

    native = relative_rotation_body(start, end)
    converted = relative_rotation_body(
        apply_orientation_convention(start, 'nwu_to_enu'),
        apply_orientation_convention(end, 'nwu_to_enu'),
    )
    assert canonicalize(native) == pytest.approx(canonicalize(converted), abs=1e-12)


def test_yaw_return_error_is_invariant_to_a_pure_yaw_world_change():
    start = axis_quaternion(2, 10)
    end = axis_quaternion(2, 22)
    native = yaw_return_error_rad(start, end)
    converted = yaw_return_error_rad(
        apply_orientation_convention(start, 'nwu_to_enu'),
        apply_orientation_convention(end, 'nwu_to_enu'),
    )
    assert native == pytest.approx(converted, abs=1e-12)


def test_yaw_return_error_is_zero_for_a_perfect_return():
    assert yaw_return_error_rad(IDENTITY, IDENTITY) == pytest.approx(0.0)


def test_yaw_return_error_measures_a_residual_offset():
    error = yaw_return_error_rad(IDENTITY, axis_quaternion(2, 12))
    assert np.degrees(error) == pytest.approx(12.0, abs=1e-6)


# --- orientation conventions -------------------------------------------------------------

def test_native_convention_is_the_identity():
    quat = axis_quaternion(1, 42)
    assert apply_orientation_convention(quat, 'bno055_native') == pytest.approx(quat)


def test_nwu_to_enu_applies_a_plus_90_degree_yaw():
    result = apply_orientation_convention(IDENTITY, 'nwu_to_enu')
    assert np.degrees(yaw_from_quaternion(result)[0]) == pytest.approx(90.0)


def test_unknown_convention_raises_rather_than_silently_passing_through():
    with pytest.raises(ValueError):
        apply_orientation_convention(IDENTITY, 'ned')


# --- averaging ---------------------------------------------------------------------------

def test_mean_quaternion_of_a_steady_hold_is_that_orientation():
    quat = axis_quaternion(2, 45)
    assert canonicalize(mean_quaternion(quaternion_hold(quat[0], 40))) == pytest.approx(
        canonicalize(quat), abs=1e-9
    )


def test_mean_quaternion_handles_the_sign_flip_a_component_mean_would_botch():
    # Half the samples reported as -q. A component-wise mean would average to ~zero here.
    quat = axis_quaternion(2, 45)
    mixed = np.vstack([quaternion_hold(quat[0], 10), -quaternion_hold(quat[0], 10)])
    assert canonicalize(mean_quaternion(mixed)) == pytest.approx(canonicalize(quat), abs=1e-9)


def test_mean_quaternion_is_robust_to_small_sensor_noise():
    quat = axis_quaternion(0, 60)
    noisy = quaternion_hold(quat[0], 200, noise_std=0.01, seed=3)
    assert canonicalize(mean_quaternion(noisy)) == pytest.approx(canonicalize(quat), abs=0.01)


def test_mean_quaternion_of_nothing_raises():
    with pytest.raises(ValueError):
        mean_quaternion(np.zeros((0, 4)))


# --- stationary statistics ---------------------------------------------------------------

def test_stationary_acceleration_of_a_level_board_is_one_g():
    accel = np.tile([0.0, 0.0, 9.81], (50, 1))
    assert stationary_stats(accel)['mean'] == pytest.approx(9.81)


def test_stationary_acceleration_near_zero_is_the_gravity_removed_signature():
    # test plan 20.7: a stationary magnitude near zero means the BNO055 gravity-removed
    # VECTOR_LINEARACCEL is being published, which the EKF would then remove a second time.
    assert stationary_stats(np.zeros((50, 3)))['mean'] == pytest.approx(0.0)


def test_stationary_stats_of_no_samples_is_zeroed_and_flagged_by_count():
    stats = stationary_stats(np.zeros((0, 3)))
    assert stats['count'] == 0 and stats['mean'] == 0.0


def test_vector_magnitudes_rejects_a_wrong_shape():
    with pytest.raises(ValueError):
        vector_magnitudes(np.zeros((4, 2)))


# --- finiteness and segmentation ---------------------------------------------------------

def test_finite_fraction_counts_nan_and_inf_as_non_finite():
    values = np.array([1.0, np.nan, 3.0, np.inf])
    assert finite_fraction(values) == pytest.approx(0.5)


def test_finite_fraction_of_empty_input_is_zero_not_one():
    # "No data" must never score as "perfectly finite data".
    assert finite_fraction([]) == 0.0


def test_segment_mask_is_half_open():
    stamps = np.array([0, 10, 20, 30], dtype=np.int64)
    assert list(segment_mask(stamps, 10, 30)) == [False, True, True, False]


# --- commanded-rotation scoring ----------------------------------------------------------

@pytest.mark.parametrize('axis_index,sign', [(0, 1.0), (0, -1.0), (1, 1.0), (1, -1.0),
                                              (2, 1.0), (2, -1.0)])
def test_a_correct_commanded_rotation_scores_correct(axis_index, sign):
    held = axis_quaternion(axis_index, 90.0 * sign)
    scored = score_commanded_rotation(IDENTITY, held, axis_index, sign, np.radians(45))
    assert scored['axis_correct'] and scored['sign_correct'] and scored['angle_sufficient']


def test_a_rotation_about_the_wrong_axis_fails_the_axis_check():
    scored = score_commanded_rotation(IDENTITY, axis_quaternion(2, 90), 0, 1.0,
                                       np.radians(45))
    assert not scored['axis_correct']


def test_a_rotation_in_the_wrong_direction_fails_the_sign_check():
    scored = score_commanded_rotation(IDENTITY, axis_quaternion(0, -90), 0, 1.0,
                                       np.radians(45))
    assert scored['axis_correct'] and not scored['sign_correct']


def test_a_rotation_too_small_to_be_deliberate_fails_the_angle_check():
    scored = score_commanded_rotation(IDENTITY, axis_quaternion(0, 10), 0, 1.0,
                                       np.radians(45))
    assert not scored['angle_sufficient']


def test_a_hand_rotation_with_realistic_off_axis_slop_still_passes():
    # ~88 deg about X with a few degrees of Y and Z contamination: what a real hand
    # rotation looks like. The gate demands axis dominance, not axis purity.
    sloppy = quaternion_multiply(
        quaternion_multiply(axis_quaternion(0, 88), axis_quaternion(1, 6)),
        axis_quaternion(2, -4),
    )
    scored = score_commanded_rotation(IDENTITY, sloppy, 0, 1.0, np.radians(45))
    assert scored['axis_correct'] and scored['sign_correct'] and scored['angle_sufficient']


def test_axis_dominance_ratio_is_reported_for_the_report_table():
    scored = score_commanded_rotation(IDENTITY, axis_quaternion(0, 90), 0, 1.0,
                                       np.radians(45))
    assert scored['axis_dominance'] == float('inf')  # perfectly on-axis
    assert scored['measured_angle_deg'] == pytest.approx(90.0)


def test_a_stricter_dominance_ratio_rejects_a_sloppier_turn():
    sloppy = quaternion_multiply(axis_quaternion(0, 60), axis_quaternion(1, 45))
    lenient = score_commanded_rotation(IDENTITY, sloppy, 0, 1.0, np.radians(45), 1.1)
    strict = score_commanded_rotation(IDENTITY, sloppy, 0, 1.0, np.radians(45), 5.0)
    assert lenient['axis_correct'] and not strict['axis_correct']
