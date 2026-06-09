"""
ik_service_node.py — ROS2 IK/FK service node for DeltaFlex.

Services provided
-----------------
/deltaflex/ik  (deltaflex_msgs/srv/IKSolve)
    Given EE position (x, y, z), return motor joint angles.

/deltaflex/fk  (deltaflex_msgs/srv/FKSolve)
    Given motor joint angles, return EE position via numerical FK.
"""

import numpy as np
import rclpy
from rclpy.node import Node

from deltaflex_msgs.srv import FKSolve, IKSolve
from deltaflex_kinematics.delta_ik import (
    forward_kinematics,
    inverse_kinematics,
    workspace_check,
)


class IKServiceNode(Node):

    def __init__(self) -> None:
        super().__init__('deltaflex_ik_service')

        self._ik_srv = self.create_service(
            IKSolve, '/deltaflex/ik', self._handle_ik
        )
        self._fk_srv = self.create_service(
            FKSolve, '/deltaflex/fk', self._handle_fk
        )

        self.get_logger().info(
            'DeltaFlex IK/FK services ready on '
            '/deltaflex/ik and /deltaflex/fk'
        )

    # ------------------------------------------------------------------
    def _handle_ik(
        self,
        request: IKSolve.Request,
        response: IKSolve.Response,
    ) -> IKSolve.Response:

        x, y, z = request.x, request.y, request.z

        if not workspace_check(x, y, z):
            response.success = False
            response.message = (
                f'Position ({x:.4f}, {y:.4f}, {z:.4f}) '
                'failed workspace pre-check.'
            )
            response.joint_angles = [0.0, 0.0, 0.0]
            self.get_logger().warn(response.message)
            return response

        try:
            angles = inverse_kinematics(x, y, z)
            response.joint_angles = angles.tolist()
            response.success = True
            response.message = 'OK'
        except ValueError as exc:
            response.success = False
            response.message = str(exc)
            response.joint_angles = [0.0, 0.0, 0.0]
            self.get_logger().warn(f'IK failed: {exc}')

        return response

    # ------------------------------------------------------------------
    def _handle_fk(
        self,
        request: FKSolve.Request,
        response: FKSolve.Response,
    ) -> FKSolve.Response:

        theta = np.array(request.joint_angles, dtype=float)

        if len(theta) != 3:
            response.success = False
            response.message = f'Expected 3 joint angles, got {len(theta)}'
            response.x = response.y = response.z = 0.0
            return response

        try:
            pos = forward_kinematics(theta)
            response.x, response.y, response.z = float(pos[0]), float(pos[1]), float(pos[2])
            response.success = True
            response.message = 'OK'
        except (ValueError, RuntimeError) as exc:
            response.success = False
            response.message = str(exc)
            response.x = response.y = response.z = 0.0
            self.get_logger().warn(f'FK failed: {exc}')

        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IKServiceNode()
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
