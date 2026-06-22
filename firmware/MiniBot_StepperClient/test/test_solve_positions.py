"""
Tests for the solvePositions() function in position_estimator.cpp.

Replicates the C++ solver math in Python, then:
  - Generates ProcessedEmagData from a known ground-truth robot pose
  - Optionally adds noise to magnitude and azimuth
  - Runs the solver and checks that the result is within the expected margin

Config mirrors config.h:
  EMAG_POSITIONS_MM = [(0.0, 0.0), (125.0, 0.0)]
  EMAG_MIN_SIGNAL_GAUSS = 0.5  (unused here; we inject magnitude directly)
  DIPOLE_STRENGTH_K: arbitrary constant controlling simulated field strength
"""

import math
import random

# ── Platform constants (must match config.h) ─────────────────────────────────
EMAG_POSITIONS_MM = [(0.0, 0.0), (125.0, 0.0)]
DIPOLE_STRENGTH_K = 5000.0  # k such that B = k / r^3 Gauss·mm³


# ── Helper: normalize angle to [-π, π] matching the C++ fmod approach ────────
def _normalize_pi(angle):
    a = math.fmod(angle + math.pi, 2.0 * math.pi)
    if a < 0.0:
        a += 2.0 * math.pi
    return a - math.pi


# ── Physics: robot pose → ProcessedEmagData inputs ───────────────────────────
def pose_to_sensor_readings(
    rx, ry, theta, noise_mag_frac=0.0, noise_angle_rad=0.0, rng=None
):
    """
    Given robot pose (rx, ry, theta) compute what processEmagReadings() would
    produce (magnitude_G, azimuth_angle_rad) for each emag.

    Dipole model: B ∝ k / r³  (pure radial dipole in the XY plane).
    The azimuth measured by the robot's magnetometer is the direction from the
    robot to the emag, expressed in the robot's local frame.

    noise_mag_frac   : std-dev of multiplicative magnitude noise (e.g. 0.02 = 2%)
    noise_angle_rad  : std-dev of additive azimuth noise in radians
    rng              : optional random.Random instance for reproducibility
    """
    if rng is None:
        rng = random.Random(0)

    readings = []
    for ex, ey in EMAG_POSITIONS_MM:
        dx = ex - rx
        dy = ey - ry
        d = math.sqrt(dx * dx + dy * dy)
        if d < 1e-3:
            raise ValueError(f"Robot is too close to emag at ({ex},{ey}): d={d:.4f} mm")

        magnitude = DIPOLE_STRENGTH_K / d**3
        if noise_mag_frac > 0.0:
            magnitude *= 1.0 + rng.gauss(0.0, noise_mag_frac)
            magnitude = max(magnitude, 1e-9)

        beta_world = math.atan2(dy, dx)  # direction to emag in world frame
        azimuth = _normalize_pi(beta_world - theta)  # direction in robot frame
        if noise_angle_rad > 0.0:
            azimuth = _normalize_pi(azimuth + rng.gauss(0.0, noise_angle_rad))

        readings.append((magnitude, azimuth))
    return readings


# ── Python replica of solvePositions() for a single emag pair ────────────────
def solve_pair(idx0, idx1, processed_data):
    """
    Mirrors the inner loop body of solvePositions() in position_estimator.cpp.
    processed_data: list of (magnitude_G, azimuth_angle_rad) indexed by emag index.
    Returns (pos_x_mm, pos_y_mm, ang_rad, confidence) or None if degenerate.
    """
    e0x, e0y = EMAG_POSITIONS_MM[idx0]
    e1x, e1y = EMAG_POSITIONS_MM[idx1]

    m0, a0 = processed_data[idx0]
    m1, a1 = processed_data[idx1]

    # Distance ratio from dipole falloff: B ∝ 1/r³  →  d0/d1 = (m1/m0)^(1/3)
    ratio = (m1 / m0) ** (1.0 / 3.0)

    da = a0 - a1
    P = ratio * math.cos(da) - 1.0
    Q = ratio * math.sin(da)
    denom = P * P + Q * Q

    if denom < 1e-6:
        return None  # degenerate geometry

    dex = e0x - e1x
    dey = e0y - e1y
    emag_sep_sq = dex * dex + dey * dey
    d1 = math.sqrt(emag_sep_sq / denom)

    A = dex / d1
    B = dey / d1

    # β1 = direction from robot to emag1 in world frame
    beta1 = math.atan2(P * B - Q * A, P * A + Q * B)

    # θ = β1 - α1  (robot heading)
    theta = _normalize_pi(beta1 - a1)

    rx = e1x - d1 * math.cos(beta1)
    ry = e1y - d1 * math.sin(beta1)

    confidence = min(m0, m1) * math.sqrt(denom)
    return rx, ry, theta, confidence


# ── Test runner helpers ───────────────────────────────────────────────────────
def angle_diff(a, b):
    """Smallest signed angle difference a - b, in [-π, π]."""
    return _normalize_pi(a - b)


def run_test(
    name,
    rx,
    ry,
    theta,
    noise_mag_frac=0.0,
    noise_angle_rad=0.0,
    pos_tol_mm=0.5,
    ang_tol_rad=0.01,
    rng=None,
):
    readings = pose_to_sensor_readings(
        rx, ry, theta, noise_mag_frac, noise_angle_rad, rng
    )

    result = solve_pair(0, 1, readings)

    assert result is not None, f"FAIL [{name}]: solver returned degenerate unexpectedly"
    rx_out, ry_out, theta_out, _ = result

    pos_err = math.sqrt((rx_out - rx) ** 2 + (ry_out - ry) ** 2)
    ang_err = abs(angle_diff(theta_out, theta))

    ok = pos_err <= pos_tol_mm and ang_err <= ang_tol_rad
    status = "OK" if ok else "FAIL"
    print(f"  {status}  [{name}]")
    print(f"       truth : pos=({rx:7.2f}, {ry:7.2f})  theta={theta:+.4f} rad")
    print(
        f"       solved: pos=({rx_out:7.2f}, {ry_out:7.2f})  theta={theta_out:+.4f} rad"
    )
    print(f"       error : pos={pos_err:.4f} mm  ang={math.degrees(ang_err):.4f} deg")

    assert ok, (
        f"FAIL [{name}]: pos_err={pos_err:.4f} mm (tol={pos_tol_mm}), "
        f"ang_err={math.degrees(ang_err):.4f}° (tol={math.degrees(ang_tol_rad):.4f}°)"
    )
    return True


# ── Test cases ────────────────────────────────────────────────────────────────
def main():
    passed = 0
    rng = random.Random(42)

    # --- No-noise exact tests ------------------------------------------------
    print("\n=== Exact (no noise) tests ===")
    exact_cases = [
        # (name,             rx,    ry,   theta)
        ("center, theta=0", 62.5, 50.0, 0.0),
        ("center, theta=pi/2", 62.5, 50.0, math.pi / 2),
        ("center, theta=-pi/4", 62.5, 50.0, -math.pi / 4),
        ("off-center left", 20.0, 40.0, 0.3),
        ("off-center right", 100.0, 60.0, -0.5),
        ("large y offset", 50.0, 120.0, 1.0),
        ("small y offset", 50.0, 5.0, 0.0),
        ("negative y", 50.0, -30.0, -0.8),
        ("theta near +pi", 62.5, 50.0, math.pi - 0.01),
        ("theta near -pi", 62.5, 50.0, -math.pi + 0.01),
        ("theta = pi/2", 30.0, 70.0, math.pi / 2),
        ("theta = -pi/2", 30.0, 70.0, -math.pi / 2),
        ("asymmetric, close to emag0", 10.0, 20.0, 0.2),
        ("asymmetric, close to emag1", 115.0, 15.0, -0.2),
        ("robot on y-axis of emag0", 0.0, 40.0, 0.0),
        ("robot on y-axis of emag1", 125.0, 40.0, math.pi),
    ]
    for args in exact_cases:
        passed += run_test(*args, pos_tol_mm=0.01, ang_tol_rad=1e-4)

    # --- Special geometry tests -----------------------------------------------
    print("\n=== Special geometry tests ===")
    # Robot on the perpendicular bisector (equidistant, but azimuths are ±π/2
    # in the robot frame when theta=0 — well-conditioned, not degenerate)
    passed += run_test(
        "perp bisector, theta=0", 62.5, 80.0, 0.0, pos_tol_mm=0.01, ang_tol_rad=1e-4
    )
    passed += run_test(
        "perp bisector, theta=pi/2",
        62.5,
        100.0,
        math.pi / 2,
        pos_tol_mm=0.01,
        ang_tol_rad=1e-4,
    )
    # Robot collinear (between the two emags on the x-axis)
    passed += run_test(
        "collinear between emags", 40.0, 0.01, 0.0, pos_tol_mm=0.1, ang_tol_rad=0.01
    )
    # Robot far away — very weak signals but ratio stays well conditioned
    passed += run_test("far away", 62.5, 400.0, 0.5, pos_tol_mm=0.01, ang_tol_rad=1e-4)
    # Robot very close to one emag (strong signal asymmetry)
    passed += run_test(
        "very close to emag0", 3.0, 12.0, 0.1, pos_tol_mm=0.01, ang_tol_rad=1e-4
    )
    passed += run_test(
        "very close to emag1", 122.0, 8.0, -0.1, pos_tol_mm=0.01, ang_tol_rad=1e-4
    )
    # Negative x (robot to the left of emag0)
    passed += run_test(
        "robot left of emag0", -40.0, 30.0, 0.7, pos_tol_mm=0.01, ang_tol_rad=1e-4
    )
    # Robot to the right of emag1
    passed += run_test(
        "robot right of emag1", 175.0, 45.0, -1.0, pos_tol_mm=0.01, ang_tol_rad=1e-4
    )

    # --- Low noise tests (1% mag, 0.005 rad angle) ---------------------------
    print("\n=== Low noise tests (s_mag=1%, s_angle=0.005 rad) ===")
    low_noise_cases = [
        ("low noise, center", 62.5, 50.0, 0.0),
        ("low noise, off-center", 20.0, 40.0, 0.3),
        ("low noise, large y", 50.0, 120.0, 1.0),
        ("low noise, theta=pi/2", 30.0, 70.0, math.pi / 2),
        ("low noise, neg y", 50.0, -30.0, -0.8),
        ("low noise, close emag0", 10.0, 20.0, 0.2),
    ]
    for args in low_noise_cases:
        name, rx, ry, theta = args
        passed += run_test(
            name,
            rx,
            ry,
            theta,
            noise_mag_frac=0.01,
            noise_angle_rad=0.005,
            pos_tol_mm=2.0,
            ang_tol_rad=0.02,
            rng=rng,
        )

    # --- Moderate noise tests (3% mag, 0.02 rad angle) -----------------------
    print("\n=== Moderate noise tests (s_mag=3%, s_angle=0.02 rad) ===")
    moderate_noise_cases = [
        ("mod noise, center", 62.5, 50.0, 0.0),
        ("mod noise, off-center", 20.0, 40.0, 0.3),
        ("mod noise, theta=pi/4", 80.0, 60.0, math.pi / 4),
        ("mod noise, neg theta", 40.0, 55.0, -1.2),
    ]
    for args in moderate_noise_cases:
        name, rx, ry, theta = args
        passed += run_test(
            name,
            rx,
            ry,
            theta,
            noise_mag_frac=0.03,
            noise_angle_rad=0.02,
            pos_tol_mm=5.0,
            ang_tol_rad=0.05,
            rng=rng,
        )

    # --- Random pose tests ---------------------------------------------------
    print("\n=== Random pose tests (no noise, pos_tol=0.1 mm, ang_tol=0.001 rad) ===")
    rand_rng = random.Random(1337)
    for i in range(20):
        # Keep robot well away from both emags (d > 10 mm)
        while True:
            rx = rand_rng.uniform(-50.0, 175.0)
            ry = rand_rng.uniform(10.0, 150.0)  # keep y > 0 to avoid equidistant line
            d0 = math.sqrt(rx**2 + ry**2)
            d1 = math.sqrt((rx - 125.0) ** 2 + ry**2)
            if d0 > 15.0 and d1 > 15.0 and abs(d0 - d1) > 5.0:
                break
        theta = rand_rng.uniform(-math.pi, math.pi)
        passed += run_test(
            f"random[{i}]", rx, ry, theta, pos_tol_mm=0.1, ang_tol_rad=1e-3
        )

    print(f"\nAll {passed} tests passed.")


if __name__ == "__main__":
    main()
