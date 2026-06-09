"""
cartesian_controller_node.py — Cartesian-space control for DeltaFlex.

Subscribes
----------
/delta/target_pose  (geometry_msgs/msg/Point)
    Desired EE position in the robot base frame [m].
    z is positive-downward; typical range z ∈ [0.15, 0.35].

Publishes
---------
/deltaflex_position_controller/commands  (std_msgs/msg/Float64MultiArray)
    Motor joint angles [theta_1, theta_2, theta_3] in radians.

ROS2 parameters
---------------
interpolation_steps  (int,   default 50)   waypoints between current and target
interpolation_dt     (float, default 0.02) seconds between waypoints
workspace_r_max      (float, default 0.05) max XY radial offset [m]
workspace_z_min      (float, default 0.15) minimum Z [m]
workspace_z_max      (float, default 0.35) maximum Z [m]

Design notes
------------
- Linear Cartesian interpolation: each waypoint is solved with IK.
- Motion runs in a daemon thread so the subscriber callback returns immediately.
- A non-blocking lock rejects overlapping trajectory requests.
- On startup the robot is commanded to the home position.
"""

import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from deltaflex_kinematics.delta_ik import (
    WORKSPACE_XY_RADIUS,
    WORKSPACE_Z_MAX,
    WORKSPACE_Z_MIN,
    inverse_kinematics,
    workspace_check,
)


class CartesianControllerNode(Node):

    # Default home position — EE directly below base centre [m]
    _HOME = np.array([0.0, 0.0, 0.25])

    def __init__(self) -> None:
        super().__init__('deltaflex_cartesian_controller')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('interpolation_steps', 50)
        self.declare_parameter('interpolation_dt', 0.02)
        self.declare_parameter('workspace_r_max', WORKSPACE_XY_RADIUS)
        self.declare_parameter('workspace_z_min', WORKSPACE_Z_MIN)
        self.declare_parameter('workspace_z_max', WORKSPACE_Z_MAX)

        self._steps    = self.get_parameter('interpolation_steps').value
        self._dt       = self.get_parameter('interpolation_dt').value
        self._r_max    = self.get_parameter('workspace_r_max').value
        self._z_min    = self.get_parameter('workspace_z_min').value
        self._z_max    = self.get_parameter('workspace_z_max').value

        # ── Current EE state ────────────────────────────────────────────
        self._current_pos = self._HOME.copy()
        try:
            self._current_joints = inverse_kinematics(*self._current_pos)
        except ValueError:
            self._current_joints = np.zeros(3)

        # ── Publisher ───────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/deltaflex_position_controller/commands',
            10,
        )

        # ── Subscriber ──────────────────────────────────────────────────
        self._target_sub = self.create_subscription(
            Point,
            '/delta/target_pose',
            self._target_callback,
            10,
        )

        # Motion guard — one trajectory at a time
        self._motion_lock = threading.Lock()

        # Send robot to home on startup
        self._publish_joints(self._current_joints)
        self.get_logger().info(
            'Cartesian controller ready. '
            f'Home position: {self._HOME.tolist()}'
        )

    # ── Subscriber callback ──────────────────────────────────────────────
    def _target_callback(self, msg: Point) -> None:
        target = np.array([msg.x, msg.y, msg.z])

        if not workspace_check(
            target[0], target[1], target[2],
            r_max=self._r_max,
            z_min=self._z_min,
            z_max=self._z_max,
        ):
            self.get_logger().warn(
                f'Target {target.tolist()} outside workspace — ignored. '
                f'Limits: r≤{self._r_max}, z∈[{self._z_min}, {self._z_max}]'
            )
            return

        try:
            target_joints = inverse_kinematics(*target)
        except ValueError as exc:
            self.get_logger().warn(f'IK failed for target {target.tolist()}: {exc}')
            return

        thread = threading.Thread(
            target=self._execute_linear_move,
            args=(self._current_pos.copy(), target, target_joints),
            daemon=True,
        )
        thread.start()

    # ── Motion execution ─────────────────────────────────────────────────
    def _execute_linear_move(
        self,
        start: np.ndarray,
        target: np.ndarray,
        target_joints: np.ndarray,
    ) -> None:
        """Cartesian linear interpolation from start to target."""
        if not self._motion_lock.acquire(blocking=False):
            self.get_logger().warn(
                'Motion already in progress — ignoring command.'
            )
            return

        try:
            for i in range(1, self._steps + 1):
                alpha    = i / self._steps
                waypoint = start + alpha * (target - start)

                try:
                    joints = inverse_kinematics(*waypoint)
                except ValueError:
                    self.get_logger().warn(
                        f'IK failed at step {i}/{self._steps} — stopping.'
                    )
                    break

                self._publish_joints(joints)
                time.sleep(self._dt)

            self._current_pos    = target.copy()
            self._current_joints = target_joints.copy()

        finally:
            self._motion_lock.release()

    # ── Helper ───────────────────────────────────────────────────────────
    def _publish_joints(self, joints: np.ndarray) -> None:
        msg = Float64MultiArray()
        msg.data = [float(j) for j in joints]
        self._cmd_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CartesianControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
