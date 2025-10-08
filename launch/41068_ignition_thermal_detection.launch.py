# ros2_ws/src/thermal_detection/launch/all_in_one.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, FindExecutable, Command, TextSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # ---- args
    gui_arg          = DeclareLaunchArgument('gui', default_value='true')
    renderer_arg     = DeclareLaunchArgument('renderer', default_value='ogre2')
    world_arg        = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('thermal_detection'), 'worlds', 'demo.sdf'
        ])
    )
    auto_unpause_arg = DeclareLaunchArgument('auto_unpause', default_value='true')
    bridge_arg       = DeclareLaunchArgument('bridge',   default_value='true')
    detector_arg     = DeclareLaunchArgument('detector', default_value='true')
    rqt_arg          = DeclareLaunchArgument('launch_rqt', default_value='false')

    # detector params (tweak at launch time)
    det_src_arg      = DeclareLaunchArgument('source_topic', default_value='/parrot/thermal/image_raw')
    det_thr_arg      = DeclareLaunchArgument('threshold_K',  default_value='305.0')
    det_minpx_arg    = DeclareLaunchArgument('min_hot_px',   default_value='10')
    det_dbg_arg      = DeclareLaunchArgument('publish_debug',default_value='true')
    det_alert_arg    = DeclareLaunchArgument('alert_topic',  default_value='/hotspot_detected')
    det_alert_txt    = DeclareLaunchArgument('alert_text_topic', default_value='/alerts/thermal_hotspot_text')
    det_cppk_arg     = DeclareLaunchArgument('counts_per_kelvin', default_value='0.0')  # 0 => auto

    gui          = LaunchConfiguration('gui')
    renderer     = LaunchConfiguration('renderer')
    world        = LaunchConfiguration('world')
    auto_unpause = LaunchConfiguration('auto_unpause')
    bridge       = LaunchConfiguration('bridge')
    detector     = LaunchConfiguration('detector')
    launch_rqt   = LaunchConfiguration('launch_rqt')

    # ---- env
    pkg_share = FindPackageShare('thermal_detection')
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

    # ---- auto unpause after server is up
    unpause = TimerAction(
        period=2.0,
        actions=[ExecuteProcess(
            cmd=[
                FindExecutable(name='ign'), 'service',
                '-s', '/world/demo/control',
                '--reqtype', 'ignition.msgs.WorldControl',
                '--reptype', 'ignition.msgs.Boolean',
                '--req', 'pause:false'
            ],
            output='screen'
        )],
        condition=IfCondition(auto_unpause),
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
        arguments=['-topic', '/robot_description', '-name', 'parrot', '-z', '2.0', '-allow_renaming', 'true'],
        output='screen',
    )

    # ---- ROS ↔︎ Ignition bridges
    bridge_topics = [
        '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
        '/parrot/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
        '/parrot/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
        '/parrot/camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
        '/parrot/thermal/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
        '/parrot/thermal/image_mono8@sensor_msgs/msg/Image@ignition.msgs.Image',
        # Add IMU/LiDAR if desired:
        # '/parrot/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
        # '/parrot/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
    ]
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_topics,
        output='screen',
        condition=IfCondition(bridge),
    )

    # ---- thermal detector (merged version)
    detector_node = Node(
        package='thermal_detection',
        executable='thermal_detector.py',
        name='thermal_detector',
        output='screen',
        parameters=[{
            'source_topic': det_src_arg.default_value,
            'threshold_K':  det_thr_arg.default_value,
            'min_hot_px':   det_minpx_arg.default_value,
            'publish_debug':det_dbg_arg.default_value,
            'alert_topic':  det_alert_arg.default_value,
            'alert_text_topic': det_alert_txt.default_value,
            'counts_per_kelvin': det_cppk_arg.default_value,  # 0 => auto infer
            # Optional fine-tuning (leave default):
            # 'center_radius_frac': 0.25,
            # 'min_center_px': 200,
        }],
        condition=IfCondition(detector),
    )

    # ---- optional: rqt image view on the debug mask
    rqt_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        arguments=['/parrot/thermal/hot_mask'],
        condition=IfCondition(launch_rqt),
        output='screen',
    )

    return LaunchDescription([
        # args
        gui_arg, renderer_arg, world_arg, auto_unpause_arg,
        bridge_arg, detector_arg, rqt_arg,
        det_src_arg, det_thr_arg, det_minpx_arg, det_dbg_arg, det_alert_arg, det_alert_txt, det_cppk_arg,
        # env + sim
        set_renderer, set_res_path, set_gz_res,
        ign_gui, ign_headless, unpause,
        # robot + tools
        state_pub, spawn_parrot,
        bridge_node, detector_node, rqt_view
    ])

