from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    TextSubstitution,
    FindExecutable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('41068_ignition_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    rviz_flag = LaunchConfiguration('rviz')
    nav2_flag = LaunchConfiguration('nav2')
    slam_flag = LaunchConfiguration('slam')

    # ---- Args ----
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use /clock from simulation'
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([pkg_share, 'worlds', 'large_demo.sdf']),
        description='Absolute path to the world SDF file',
    )
    rviz_arg = DeclareLaunchArgument('rviz', default_value='False', description='Launch RViz2')
    nav2_arg = DeclareLaunchArgument('nav2', default_value='False', description='Launch Nav2')
    slam_arg = DeclareLaunchArgument('slam', default_value='True', description='Run slam_toolbox (map->odom)')

    # ---- Model/resource paths for Ignition/GZ ----
    set_ign_res = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[pkg_share, TextSubstitution(text=':'), PathJoinSubstitution([pkg_share, 'models'])],
    )
    set_gz_res = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[pkg_share, TextSubstitution(text=':'), PathJoinSubstitution([pkg_share, 'models'])],
    )

    # ---- robot_description from xacro ----
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            TextSubstitution(text=' '),
            PathJoinSubstitution([pkg_share, 'urdf_drone', 'parrot.urdf.xacro']),
        ]),
        value_type=str,
    )
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ---- Start Ignition Gazebo ----
    gz = ExecuteProcess(cmd=['ign', 'gazebo', world, '-r'], output='screen')

    # ---- Spawn the drone from /robot_description ----
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic', '/robot_description', '-name', 'parrot', '-z', '2.0'],
    )

    # ---- Bridge topics as per YAML (pass as parameters, not --params-file) ----
    bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    name='parameter_bridge',
    output='screen',
    parameters=[{
        'config_file': PathJoinSubstitution([
            pkg_share, 'config', 'gazebo_bridge.yaml'
        ])
    }],
)


    # ---- EKF (robot_localization): consumes /odometry + /imu, publishes odom->base_link ----
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'robot_localization.yaml']),
            {'use_sim_time': use_sim_time},
        ],
    )

    # ---- SLAM (map->odom). If off, publish identity map->odom so TF tree is complete. ----
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'slam_params.yaml']),
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(slam_flag),
    )

    map_to_odom_identity = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
        condition=UnlessCondition(slam_flag),
    )

    # ---- RViz (optional) ----
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([pkg_share, 'config', '41068.rviz'])],
        condition=IfCondition(rviz_flag),
    )

    # ---- Nav2 (optional) ----
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_share, 'launch', '41068_navigation.launch.py'])]
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(nav2_flag),
    )

    return LaunchDescription([
        use_sim_time_arg, world_arg, rviz_arg, nav2_arg, slam_arg,
        set_ign_res, set_gz_res,
        rsp, gz, spawn, bridge,
        ekf, slam_node, map_to_odom_identity,
        rviz, nav2,
    ])
