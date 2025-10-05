from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, FindExecutable, Command, TextSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ---- args
    gui_arg         = DeclareLaunchArgument('gui', default_value='true')
    renderer_arg    = DeclareLaunchArgument('renderer', default_value='ogre2')
    world_arg       = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('thermal_detection'), 'worlds', 'demo.sdf'
        ])
    )
    bridge_arg      = DeclareLaunchArgument('bridge',   default_value='true')
    detector_arg    = DeclareLaunchArgument('detector', default_value='true')

    gui      = LaunchConfiguration('gui')
    renderer = LaunchConfiguration('renderer')
    world    = LaunchConfiguration('world')
    bridge   = LaunchConfiguration('bridge')
    detector = LaunchConfiguration('detector')

    pkg_share = FindPackageShare('thermal_detection')

    # ---- env
    set_renderer = SetEnvironmentVariable('IGN_RENDERING_ENGINE', renderer)
    set_res_path = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        [pkg_share, ':', PathJoinSubstitution([pkg_share, 'models'])]
    )
    set_gz_res = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        [pkg_share, ':', PathJoinSubstitution([pkg_share, 'models'])]
    )

    # ---- Gazebo (GUI or headless)
    ign_gui = ExecuteProcess(
        cmd=[FindExecutable(name='ign'), 'gazebo', world],
        output='screen',
        condition=IfCondition(gui),
    )
    ign_headless = ExecuteProcess(
        cmd=[FindExecutable(name='ign'), 'gazebo', '-r', world],
        output='screen',
        condition=UnlessCondition(gui),
    )

    # ---- robot_description (xacro -> urdf)
    parrot_xacro = PathJoinSubstitution([pkg_share, 'urdf_drone', 'parrot.urdf.xacro'])
    robot_description = {
        'robot_description': ParameterValue(
            Command([FindExecutable(name='xacro'), TextSubstitution(text=' '), parrot_xacro]),
            value_type=str
        )
    }
    state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen',
    )

    # ---- spawn the drone
    spawn_parrot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description', '-name', 'parrot', '-z', '2.0'],
        output='screen',
    )

    # ---- ROS ↔︎ Ignition bridges (Fortress uses 'ignition.msgs.*')
    bridge_topics = [
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock]',
        '/parrot/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image]',
        '/parrot/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo]',
        '/parrot/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image]',
        '/parrot/thermal/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image]',
        '/parrot/thermal/image_mono8@sensor_msgs/msg/Image[ignition.msgs.Image]',
        '/parrot/thermal/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo]',
        # optional: points -> PointCloud2 if you need it
        # '/parrot/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked]',
    ]
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_topics,
        output='screen',
        condition=IfCondition(bridge),
    )

    # ---- thermal detector (threshold & other params adjustable here)
    detector_node = Node(
        package='thermal_detection',
        executable='thermal_detector.py',
        name='thermal_detector',
        output='screen',
        parameters=[{
            'source_topic': '/parrot/thermal/image_raw',   # L16 via bridge
            'threshold_K':  310.0,                         # ~36.85°C
            'min_hot_px':   50,
            'publish_debug': True,                          # publishes /parrot/thermal/hot_mask
            'alert_topic':  '/alerts/thermal_hotspot'
        }],
        condition=IfCondition(detector),
    )

    return LaunchDescription([
        gui_arg, renderer_arg, world_arg, bridge_arg, detector_arg,
        set_renderer, set_res_path, set_gz_res,
        ign_gui, ign_headless,
        state_pub, spawn_parrot,
        bridge_node, detector_node
    ])

