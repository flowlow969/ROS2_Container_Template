"""
delta_ik.py — Delta robot inverse and forward kinematics for DeltaFlex.

Coordinate convention
---------------------
- Z is positive **downward** (robot hangs from ceiling mount).
- Home position: EE directly below the base centre at z = RF + RE (approx).
- All angles in radians; positive angle = arm rotates downward.

Parameters (measure from STEP files in the DeltaFlex repository):
  https://github.com/made-iit/deltaflex

Paper reference: Parmiggiani et al. — confirmed values shown in CONFIRMED section.
"""

import numpy as np
from typing import Tuple

# ──────────────────────────────────────────────────────────────────────────────
# Robot geometry parameters — from Parmiggiani et al.
# Update F, E, RF, RE once exact STEP-file measurements are available.
# ──────────────────────────────────────────────────────────────────────────────
F  = 0.100   # base circumradius [m]   (centre → vertex of base equilateral triangle)
E  = 0.040   # EE circumradius [m]     (centre → vertex of EE triangle)
RF = 0.120   # upper arm length [m]
RE = 0.200   # forearm length [m]

# Derived constants (precomputed at import time)
_SQRT3 = np.sqrt(3.0)
_F_OVER_2SQRT3 = F / (2.0 * _SQRT3)   # = F*sqrt(3)/6
_E_OVER_2SQRT3 = E / (2.0 * _SQRT3)

# ──────────────────────────────────────────────────────────────────────────────
# Workspace limits confirmed from paper (repeatability test, Fig. 7, page 8/12)
# ──────────────────────────────────────────────────────────────────────────────
WORKSPACE_XY_RADIUS = 0.050   # m  (±50 mm radial)
WORKSPACE_Z_MIN     = -0.060  # m  (±60 mm, negative = upward in Z-down convention)
WORKSPACE_Z_MAX     =  0.060  # m


def _rotate_xy(x: float, y: float, angle_deg: float) -> Tuple[float, float]:
    """Rotate point (x, y) in the horizontal plane by angle_deg degrees."""
    a = np.radians(angle_deg)
    return x * np.cos(a) - y * np.sin(a), x * np.sin(a) + y * np.cos(a)


def _ik_single_arm(x0: float, y0: float, z0: float) -> float:
    """
    Analytical IK for arm 1 (Y-Z plane canonical frame).

    Parameters
    ----------
    x0, y0, z0 :
        EE position already rotated into this arm's canonical frame [m].
        z0 must be positive (Z-positive-downward convention).

    Returns
    -------
    theta : float
        Motor shaft angle in radians (0 = upper arm horizontal).

    Raises
    ------
    ValueError
        If the target is geometrically unreachable.
    """
    y1  = -_F_OVER_2SQRT3
    y0p = y0 - _E_OVER_2SQRT3     # shift by EE triangle offset

    a = (x0**2 + y0p**2 + z0**2 + RF**2 - RE**2 - y1**2) / (2.0 * z0)
    b = (y1 - y0p) / z0

    discriminant = -(a + b * y1)**2 + RF**2 * (b**2 + 1.0)

    if discriminant < 0:
        raise ValueError(
            f'IK no solution: EE ({x0:.4f}, {y0:.4f}, {z0:.4f}) unreachable '
            f'(discriminant={discriminant:.6f})'
        )

    y_j = (y1 - a * b - np.sqrt(discriminant)) / (b**2 + 1.0)
    z_j = a + b * y_j

    return np.arctan2(-z_j, y_j - y1)


def inverse_kinematics(x: float, y: float, z: float) -> np.ndarray:
    """
    Compute all three motor joint angles for a target EE position.

    Parameters
    ----------
    x, y, z : float
        Desired EE position in the robot base frame [m].
        z is positive-downward; typical working range z ∈ [0.15, 0.35].

    Returns
    -------
    np.ndarray, shape (3,)
        [theta_1, theta_2, theta_3] in radians.

    Raises
    ------
    ValueError
        If the target is unreachable for any arm.
    """
    # Arm 1 — canonical frame (no rotation needed)
    theta_1 = _ik_single_arm(x, y, z)

    # Arm 2 — rotate EE target by +120° into arm 2's canonical frame
    x2, y2 = _rotate_xy(x, y, 120.0)
    theta_2 = _ik_single_arm(x2, y2, z)

    # Arm 3 — rotate EE target by +240° into arm 3's canonical frame
    x3, y3 = _rotate_xy(x, y, 240.0)
    theta_3 = _ik_single_arm(x3, y3, z)

    return np.array([theta_1, theta_2, theta_3])


def forward_kinematics(
    theta: np.ndarray,
    tol: float = 1e-6,
    max_iter: int = 100,
) -> np.ndarray:
    """
    Numerical forward kinematics via Newton-Raphson iteration.

    Finds EE position (x, y, z) such that inverse_kinematics(x, y, z) ≈ theta.

    Parameters
    ----------
    theta : np.ndarray, shape (3,)
        Motor joint angles in radians.
    tol :
        Convergence tolerance [m].
    max_iter :
        Maximum Newton-Raphson iterations.

    Returns
    -------
    np.ndarray, shape (3,)
        EE position [x, y, z] in metres.

    Raises
    ------
    RuntimeError
        If Newton-Raphson does not converge.
    """
    # Initial guess: approximate home position
    pos = np.array([0.0, 0.0, (RF + RE) * 0.85])

    for _ in range(max_iter):
        try:
            theta_current = inverse_kinematics(*pos)
        except ValueError:
            pos *= 0.95   # pull back toward base if out of workspace
            continue

        error = theta_current - theta
        if np.linalg.norm(error) < tol:
            return pos

        # Numerical Jacobian (finite differences)
        eps = 1e-5
        J = np.zeros((3, 3))
        for j in range(3):
            dp = pos.copy()
            dp[j] += eps
            try:
                J[:, j] = (inverse_kinematics(*dp) - theta_current) / eps
            except ValueError:
                J[:, j] = 0.0

        try:
            delta = np.linalg.solve(J, -error)
        except np.linalg.LinAlgError:
            break

        pos += delta

    raise RuntimeError(
        f'FK: Newton-Raphson did not converge after {max_iter} iterations'
    )


def workspace_check(
    x: float,
    y: float,
    z: float,
    r_max: float = WORKSPACE_XY_RADIUS,
    z_min: float = WORKSPACE_Z_MIN,
    z_max: float = WORKSPACE_Z_MAX,
) -> bool:
    """
    Fast workspace pre-check before calling IK.

    Limits are from Parmiggiani et al. (Fig. 7, repeatability test):
      XY: ±50 mm radial, Z: ±60 mm (Z-positive-downward convention).
    """
    r = np.hypot(x, y)
    return (r <= r_max) and (z_min <= z <= z_max)
