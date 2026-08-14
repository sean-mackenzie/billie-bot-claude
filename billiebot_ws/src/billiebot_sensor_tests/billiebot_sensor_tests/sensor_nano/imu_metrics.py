"""Pure IMU/quaternion math for UT-IMU-01 and UT-IMU-02. No rclpy, no hardware imports.

Quaternions are `(w, x, y, z)` throughout, stored as `(N, 4)` float arrays, and are always
taken as **sensor-to-world** rotations (`v_world = q * v_sensor * q^-1`) -- the convention
the BNO055 fusion output uses.

Commanded-rotation scoring uses quaternion *relative* rotations, never Euler differences: a
sequence of Euler angles wraps, gimbal-locks near +/-90 degrees pitch, and makes a clean
+90 degrees roll look like a discontinuity. `relative_rotation_body()` plus `axis_angle()`
answers "which body axis did it turn about, by how much, and in which direction" without any
of that.
"""

import numpy as np

#: Named world-frame conventions accepted by apply_orientation_convention() and by
#: sensor_nano_bridge's `orientation_frame_convention` parameter.
ORIENTATION_CONVENTIONS = ('bno055_native', 'nwu_to_enu')

#: Rotation taking NWU world axes to ENU world axes: +90 degrees about Z.
#:
#: Derivation: with q_in mapping sensor->NWU, we want q_out mapping sensor->ENU, so
#: q_out = q_(ENU<-NWU) * q_in. Expressing the NWU basis in ENU coordinates gives
#: North -> (0, 1, 0), West -> (-1, 0, 0), Up -> (0, 0, 1), i.e. R = R_z(+90 degrees),
#: whose quaternion is (cos 45, 0, 0, sin 45).
#:
#: NOTE: that the BNO055's NDOF fusion world frame *is* NWU is a documented hypothesis, not
#: an established fact for this hardware -- which is exactly why `bno055_native` (identity)
#: is the shipped default and why UT-IMU-01/UT-IMU-02 measure the observed axis and sign
#: rather than assuming one. Do not switch the default until a hardware run says so.
_Q_ENU_FROM_NWU = np.array([np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)])


def as_quaternion_array(quats) -> np.ndarray:
    """Coerce to a float64 (N, 4) array, raising on a wrong-shaped input."""
    arr = np.asarray(quats, dtype=np.float64)
    if arr.ndim == 1:
        arr = arr.reshape(1, 4)
    if arr.ndim != 2 or arr.shape[1] != 4:
        raise ValueError(f'expected an (N, 4) quaternion array, got shape {arr.shape}')
    return arr


def quaternion_norms(quats) -> np.ndarray:
    """Euclidean norm of each quaternion. A healthy BNO055 stream sits within 0.98-1.02."""
    return np.linalg.norm(as_quaternion_array(quats), axis=1)


def normalize_quaternions(quats) -> np.ndarray:
    """Unit-normalize. Zero-norm quaternions are left as-is rather than producing NaN --
    callers gate on quaternion_norms() first, so a zero here is a detected fault, not a
    silent divide."""
    arr = as_quaternion_array(quats)
    norms = np.linalg.norm(arr, axis=1, keepdims=True)
    safe = np.where(norms > 0, norms, 1.0)
    return arr / safe


def canonicalize(quats) -> np.ndarray:
    """Force w >= 0, resolving the q/-q double cover so axis_angle() returns an angle in
    [0, pi] and a signed axis rather than an arbitrary hemisphere."""
    arr = as_quaternion_array(quats).copy()
    flip = arr[:, 0] < 0
    arr[flip] *= -1.0
    return arr


def quaternion_conjugate(quats) -> np.ndarray:
    """Conjugate, which is the inverse for unit quaternions."""
    arr = as_quaternion_array(quats).copy()
    arr[:, 1:] *= -1.0
    return arr


def quaternion_multiply(a, b) -> np.ndarray:
    """Hamilton product a*b, broadcasting over (N, 4) inputs."""
    qa = as_quaternion_array(a)
    qb = as_quaternion_array(b)
    aw, ax, ay, az = qa[:, 0], qa[:, 1], qa[:, 2], qa[:, 3]
    bw, bx, by, bz = qb[:, 0], qb[:, 1], qb[:, 2], qb[:, 3]
    return np.stack([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ], axis=1)


def relative_rotation_body(q_from, q_to) -> np.ndarray:
    """Rotation from `q_from` to `q_to` expressed in the **starting body frame**:
    `conj(q_from) * q_to`.

    Body-frame is the right choice for the UT-IMU-01 motion script, whose steps are phrased
    as "rotate about the board's X axis" -- the axis is fixed to the board, not to the world.
    """
    return quaternion_multiply(quaternion_conjugate(q_from), q_to)


def relative_rotation_world(q_from, q_to) -> np.ndarray:
    """Rotation from `q_from` to `q_to` expressed in the **world frame**:
    `q_to * conj(q_from)`. Used for the yaw-sign check, where the axis of interest is
    world-up rather than a board axis."""
    return quaternion_multiply(q_to, quaternion_conjugate(q_from))


def axis_angle(quats):
    """Decompose into (axis (N, 3) unit vectors, angle (N,) radians in [0, pi]).

    Canonicalizes first, so the returned axis carries the direction: a -90 degrees turn about
    +X reads as +90 degrees about -X.  Near-zero rotations get a zero axis rather than an
    amplified-noise direction.
    """
    arr = canonicalize(normalize_quaternions(quats))
    w = np.clip(arr[:, 0], -1.0, 1.0)
    angle = 2.0 * np.arccos(w)
    sin_half = np.sqrt(np.clip(1.0 - w * w, 0.0, 1.0))
    axis = np.zeros((arr.shape[0], 3))
    significant = sin_half > 1e-8
    axis[significant] = arr[significant, 1:] / sin_half[significant, None]
    return axis, angle


def dominant_axis(axis):
    """(index, sign) of the largest-magnitude component: 0=x, 1=y, 2=z.

    This is how "did the +90 degrees X command actually turn about X?" is scored -- it
    answers axis identity and direction without pinning down an exact angle.
    """
    vec = np.asarray(axis, dtype=np.float64).reshape(-1)
    if vec.size != 3:
        raise ValueError(f'expected a 3-vector axis, got size {vec.size}')
    index = int(np.argmax(np.abs(vec)))
    sign = float(np.sign(vec[index])) if vec[index] != 0 else 0.0
    return index, sign


def wrap_angle_rad(angle):
    """Wrap to (-pi, pi]."""
    return -(np.mod(-np.asarray(angle, dtype=np.float64) + np.pi, 2.0 * np.pi) - np.pi)


def yaw_from_quaternion(quats) -> np.ndarray:
    """Z-Y-X (yaw-pitch-roll) yaw in radians, positive counter-clockwise seen from +Z.

    Only meaningful as a *relative* quantity in this campaign -- absolute magnetic heading is
    explicitly non-gating (test plan section 14.8, BLK-13).
    """
    arr = normalize_quaternions(quats)
    w, x, y, z = arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3]
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def mean_quaternion(quats) -> np.ndarray:
    """Markley average: the unit eigenvector of sum(q q^T) for the largest eigenvalue.

    Correct across the q/-q sign ambiguity, unlike a component-wise mean, so a hold segment
    straddling the double cover still averages to the orientation actually held.
    """
    arr = normalize_quaternions(quats)
    if arr.shape[0] == 0:
        raise ValueError('cannot average an empty quaternion set')
    m = arr.T @ arr
    _eigenvalues, eigenvectors = np.linalg.eigh(m)
    mean = eigenvectors[:, -1]  # eigh returns ascending eigenvalues
    if mean[0] < 0:
        mean = -mean
    return mean.reshape(1, 4)


def apply_orientation_convention(quats, convention: str) -> np.ndarray:
    """Re-express sensor-to-world quaternions in the named world frame.

    'bno055_native' is the identity and is the shipped default: the bench deliberately does
    not bake in an unverified frame assumption. See _Q_ENU_FROM_NWU for why 'nwu_to_enu' is
    a hypothesis rather than a fact.
    """
    if convention == 'bno055_native':
        return as_quaternion_array(quats)
    if convention == 'nwu_to_enu':
        return quaternion_multiply(np.tile(_Q_ENU_FROM_NWU, (len(as_quaternion_array(quats)), 1)),
                                    quats)
    raise ValueError(
        f'unknown orientation convention {convention!r}; expected one of {ORIENTATION_CONVENTIONS}'
    )


def finite_fraction(values) -> float:
    """Fraction of entries that are finite. Empty input scores 0.0 -- "no data" must never
    read as "perfectly finite data"."""
    arr = np.asarray(values, dtype=np.float64)
    if arr.size == 0:
        return 0.0
    return float(np.mean(np.isfinite(arr)))


def vector_magnitudes(vectors) -> np.ndarray:
    """Row-wise magnitude of an (N, 3) array."""
    arr = np.asarray(vectors, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] != 3:
        raise ValueError(f'expected an (N, 3) array, got shape {arr.shape}')
    return np.linalg.norm(arr, axis=1)


def stationary_stats(vectors) -> dict:
    """Magnitude statistics over a stationary window.

    For acceleration this is the gravity check: a stationary BNO055 must read ~9.8 m/s^2, and
    a value near zero is the signature of the gravity-removed VECTOR_LINEARACCEL output being
    published by mistake (test plan section 20.7). For gyro it is the bias/noise check.
    """
    magnitudes = vector_magnitudes(vectors)
    if magnitudes.size == 0:
        return {'count': 0, 'mean': 0.0, 'median': 0.0, 'std': 0.0, 'min': 0.0, 'max': 0.0}
    return {
        'count': int(magnitudes.size),
        'mean': float(np.mean(magnitudes)),
        'median': float(np.median(magnitudes)),
        'std': float(np.std(magnitudes)),
        'min': float(np.min(magnitudes)),
        'max': float(np.max(magnitudes)),
    }


def segment_mask(timestamps_ns, t_start_ns, t_end_ns) -> np.ndarray:
    """Boolean mask for samples in [t_start_ns, t_end_ns)."""
    stamps = np.asarray(timestamps_ns, dtype=np.int64)
    return (stamps >= int(t_start_ns)) & (stamps < int(t_end_ns))


def score_commanded_rotation(q_reference, q_held, expected_axis: int, expected_sign: float,
                              min_angle_rad: float, axis_dominance_ratio: float = 2.0) -> dict:
    """Score one commanded hand rotation from a reference hold to a rotated hold.

    Returns the measured axis/angle plus the three booleans that make up the
    "clear correct-axis response with expected sign" gate of test plan section 14.8:
    the turn was large enough to be a deliberate motion, the dominant axis is the commanded
    one by a clear margin, and its sign matches.

    `axis_dominance_ratio` is how many times larger the commanded axis component must be than
    the largest off-axis component -- a hand rotation is never perfectly about one axis, so
    demanding dominance rather than purity is what keeps this gate honest.
    """
    relative = relative_rotation_body(q_reference, q_held)
    axis, angle = axis_angle(relative)
    axis_vec = axis[0]
    measured_angle = float(angle[0])
    index, sign = dominant_axis(axis_vec)

    magnitudes = np.abs(axis_vec)
    on_axis = magnitudes[expected_axis]
    off_axis = float(np.max(np.delete(magnitudes, expected_axis)))
    dominance = float(on_axis / off_axis) if off_axis > 0 else float('inf')

    return {
        'measured_axis': [float(v) for v in axis_vec],
        'measured_angle_rad': measured_angle,
        'measured_angle_deg': float(np.degrees(measured_angle)),
        'dominant_axis_index': index,
        'dominant_axis_sign': sign,
        'axis_dominance': dominance,
        'angle_sufficient': measured_angle >= min_angle_rad,
        'axis_correct': index == expected_axis and dominance >= axis_dominance_ratio,
        'sign_correct': sign == expected_sign,
    }


def yaw_return_error_rad(q_initial, q_final) -> float:
    """Absolute world-frame yaw difference between the opening and closing holds, wrapped.

    Scores the "returns toward its starting orientation without gross discontinuity" criterion
    of test plan section 15.7.
    """
    relative = relative_rotation_world(q_initial, q_final)
    return float(np.abs(wrap_angle_rad(yaw_from_quaternion(relative)[0])))
