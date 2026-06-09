"""
sim.launch.py — Full Gazebo Harmonic simulation for DeltaFlex.

Sequence
--------
1. Gazebo Harmonic  (gz sim -r empty.sdf)
2. robot_state_publisher  (publishes /robot_description + TF from URDF)
3. ros_gz_sim/create  (spawns the robot URDF into Gazebo)
4. ros_gz_bridge  (Gazebo clock → ROS2 /clock)
5. joint_state_broadcaster spawner  (waits for controller_manager from gz_ros2_control)
6. deltaflex_position_controller spawner  (activated after JSB)
7. deltaflex_ik_service  (IK/FK ROS2 services)
8. deltaflex_cartesian_controller  (Cartesian pose subscriber → joint commands)

Arguments
---------
gui           true|false  (default true)   Show Gazebo GUI
with_rviz     true|false  (default false)  Also launch RViz2
with_cartesian true|false (default true)   Start the Cartesian control node

Usage
-----
ros2 launch deltaflex_bringup sim.launch.py
ros2 launch deltaflex_bringup sim.launch.py gui:=false
ros2 launch deltaflex_bringup sim.launch.py with_rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Package paths ─────────────────────────────────────────────────────────
    description_pkg_share = get_package_share_directory('deltaflex_description')
    bringup_pkg_share     = get_package_share_directory('deltaflex_bringup')

    xacro_file   = os.path.join(description_pkg_share, 'urdf', 'deltaflex.urdf.xacro')
    world_file   = os.path.join(bringup_pkg_share, 'worlds', 'empty.sdf')

    # ── Launch arguments ──────────────────────────────────────────────────────
    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo with GUI (true) or headless (false)',
    )
    declare_with_rviz = DeclareLaunchArgument(
        'with_rviz',
        default_value='false',
        description='Also launch RViz2 for joint state visualisation',
    )
    declare_with_cartesian = DeclareLaunchArgument(
        'with_cartesian',
        default_value='true',
        description='Start the Cartesian controller node',
    )

    gui             = LaunchConfiguration('gui')
    with_rviz       = LaunchConfiguration('with_rviz')
    with_cartesian  = LaunchConfiguration('with_cartesian')

    # ── Robot description (xacro → URDF string) ───────────────────────────────
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            xacro_file,
            ' use_sim:=true',
        ]),
        value_type=str,
    )
    robot_description = {'robot_description': robot_description_content}

    # ── 1. Gazebo Harmonic ────────────────────────────────────────────────────
    # Uses the ros_gz_sim launch wrapper so ROS_DOMAIN_ID and env vars are set.
    # When gui:=false, '-s' (server-only / headless) is appended to gz_args.
    gz_args = PythonExpression([
        '"', world_file, ' -r"',
        ' + (" -s" if "', gui, '" != "true" else "")',
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': gz_args,
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ── 2. robot_state_publisher ──────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # ── 3. Spawn robot into Gazebo ────────────────────────────────────────────
    # gz_ros2_control plugin inside the URDF starts controller_manager once loaded
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name',  'deltaflex',
            '-z',     '0.3',   # spawn 0.3 m above ground
        ],
        output='screen',
    )

    # ── 4. Clock bridge (Gazebo sim time → ROS2 /clock) ──────────────────────
    gz_clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # ── 5. Joint state broadcaster spawner ───────────────────────────────────
    # The spawner waits automatically for controller_manager to become active.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    # ── 6. Position controller spawner (after JSB is active) ─────────────────
    position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'deltaflex_position_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    # Activate position controller only after JSB finishes spawning
    delayed_position_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[position_controller_spawner],
        )
    )

    # ── 7. IK / FK service node ───────────────────────────────────────────────
    ik_node = Node(
        package='deltaflex_kinematics',
        executable='ik_service_node',
        name='deltaflex_ik_service',
        output='screen',
    )

    # ── 8. Cartesian controller node ──────────────────────────────────────────
    cartesian_node = Node(
        package='deltaflex_controllers',
        executable='cartesian_controller',
        name='deltaflex_cartesian_controller',
        output='screen',
        condition=IfCondition(with_cartesian),
    )

    # ── Optional: RViz2 ──────────────────────────────────────────────────────
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        condition=IfCondition(with_rviz),
    )

    return LaunchDescription([
        # Arguments
        declare_gui,
        declare_with_rviz,
        declare_with_cartesian,

        # Core sim
        gz_sim,
        robot_state_publisher,
        gz_clock_bridge,
        spawn_robot,

        # Controllers (ordered via event handler)
        joint_state_broadcaster_spawner,
        delayed_position_controller,

        # Application nodes
        ik_node,
        cartesian_node,

        # Optional
        rviz2,
    ])
