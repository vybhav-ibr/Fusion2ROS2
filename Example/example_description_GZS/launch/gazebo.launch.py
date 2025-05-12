
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    package_name='example_description'
    robot_name='example'
    share_dir = get_package_share_directory(package_name)
    xacro_file = os.path.join(share_dir, 'urdf', f"{robot_name}.xacro")

    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf},
            {'use_sim_time':True}
        ]
    )

    # joint_state_publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
        parameters=[
            {'use_sim_time':True}
        ]
    )

    # Launch Gazebo Sim (gz sim)
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': '-r --verbose','use_sim_time': 'True'}.items()
    )

    # Spawn entity in Gazebo Sim
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-string', robot_urdf,
            '-allow_renaming', 'true'
        ],
        output='screen'
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            'config',
            'ros2_controller.yaml',
        ]
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--param-file',
            robot_controllers,
        ],
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--param-file',
            robot_controllers,
        ],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock', '/world/default/model/example/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model', '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V', '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist', '/kinect/image@sensor_msgs/msg/Image[gz.msgs.Image', '/kinect/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked', '/kinect/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo', '/zed/image_raw@sensor_msgs/msg/Image[gz.msgs.Image', '/zed/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo', '/gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat', '/bosch/imu@sensor_msgs/msg/Imu[gz.msgs.IMU', '/hokuyo/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan', '/velodyne/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
        remappings=[]
    )

    delay_rqt = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=[
                Node(
                    package="rqt_gui",
                    executable="rqt_gui",
                    name="rqt_joint_trajectory",
                    arguments=["--force-discover"],
                    output="screen",
                )
            ]
        )
    )


    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_sim,
        spawn_entity,
        gz_ros2_bridge,
        joint_trajectory_controller_spawner,
        joint_state_broadcaster,
        delay_rqt
    ])
