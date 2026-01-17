import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, TimerAction
#v1.1.0 - Added image transport node and camera_info topic remapping
#v1.2.0 - In progress, added laser scan QoS settings, static transform for lidar, and tf2 buffer server
def generate_launch_description():

    # Package name
    package_name='ardi'

    # Launch configurations
    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')
    enable_depth_compression = LaunchConfiguration('enable_depth_compression')

    # Path to default world 
    world_path = os.path.join(get_package_share_directory(package_name),'worlds', 'obstacles.world')

    # Launch Arguments
    declare_world = DeclareLaunchArgument(
        name='world', default_value=world_path,
        description='Full path to the world model file to load')
    
    declare_rviz = DeclareLaunchArgument(
        name='rviz', default_value='True',
        description='Opens rviz is set to True')

    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value='false',
        description='Use sim time if true')
    
    declare_enable_depth_compression = DeclareLaunchArgument(
        name='enable_depth_compression', default_value='true',
        description='Enable depth image compression for /camera/depth_image topic') 

    # Launch Robot State Publisher Node
    urdf_path = os.path.join(get_package_share_directory(package_name),'urdf','robot_model.urdf')
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','robotstate.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'urdf': urdf_path}.items()
    )

    # Launch the gazebo server to initialize the simulation
    gazebo_server = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                    )]), launch_arguments={'gz_args': ['-r -s -v1 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    # Always launch the gazebo client to visualize the simulation
    gazebo_client = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                    )]), launch_arguments={'gz_args': '-g '}.items()
    )

    # Run the spawner node from the gazebo_ros package. 
    spawn_robot = Node(
                        package='ros_gz_sim', 
                        executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'ardi',
                                   '-z', '0.2'],
                        output='screen'
    )

    # Launch the Gazebo-ROS bridge
    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gazebo_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',]
    )
    
    # Launch Rviz with rviz config file
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'bot.rviz')
    rviz2 = GroupAction(
        condition=IfCondition(rviz),
        actions=[Node(
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', rviz_config_file],
                    output='screen',)]
    )
    
    # Node to bridge camera image with image_transport and compressed_image_transport
    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/camera/image",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time'),
             'camera.image.format': 'png'},
        ],
    )

    # Relay node to republish /camera/camera_info to /camera/image/camera_info
    relay_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        output='screen',
        arguments=['camera/camera_info', 'camera/image/camera_info'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    depth_image_republish = Node(
        package="image_transport",
        executable="republish",
        name="depth_image_republish",
        namespace="camera/depth_image",
        arguments=["raw", "compressedDepth"],
        remappings=[
            ("in", "/camera/depth_image"),
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(enable_depth_compression),
    )

    depth_compressed_relay = Node(
        package="topic_tools",
        executable="relay",
        name="depth_compressed_relay",
        arguments=[
            "/camera/depth_image/out/compressedDepth",
            "/camera/depth_image/compressedDepth",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_depth_compression")),
    )

    # Static transform publisher for GPU Lidar
    static_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_gpu_lidar',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '--frame-id', 'base_link',
            '--child-frame-id', 'ardi/base_link/gpu_lidar',
            '--x', '0.05', '--y', '0', '--z', '0.23',
            '--roll', '0', '--pitch', '0', '--yaw', '0']
    )

    # Wrap it in a TimerAction to delay startup by 2 seconds
    static_lidar_tf_delayed = TimerAction(
        period=2.0,  # seconds
        actions=[static_lidar_tf]
    )

    # Increase the tf2 buffer size by launching a tf2 buffer server
    tf_buffer_server = Node(
        package='tf2_ros',
        executable='buffer_server',
        name='tf_buffer_server',
        parameters=[{'use_sim_time': True}],
    )
    
    # Launch them all!
    return LaunchDescription([
        # Declare launch arguments
        declare_rviz,
        declare_world,
        declare_use_sim_time,
        declare_enable_depth_compression,

        # Launch the nodes
        gz_image_bridge_node,
        relay_camera_info_node,
        depth_image_republish,
        depth_compressed_relay,
        static_lidar_tf_delayed,
        tf_buffer_server,
        rviz2,
        rsp,
        gazebo_server,
        gazebo_client,
        ros_gz_bridge,
        spawn_robot
    ])