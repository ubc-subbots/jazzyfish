import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='cube.world',
        description='Gazebo world file'
    )

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_spiderfish_gazebo = get_package_share_directory('spiderfish_gazebo')

    gmp = 'GZ_SIM_RESOURCE_PATH'
    add_model_path = SetEnvironmentVariable(
        name=gmp, 
        value=[
            EnvironmentVariable(gmp), 
            os.pathsep + os.path.join(pkg_spiderfish_gazebo, 'gazebo', 'models')
        ]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r ', PathJoinSubstitution([
            pkg_spiderfish_gazebo,
            'gazebo',
            'worlds',
            LaunchConfiguration('world')
        ])]}.items(),
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/spiderfish/drivers/front_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',

        ],
        remappings=[
            ('/imu', '/spiderfish/drivers/imu') 
        ],
        output='screen'
    )

    ros_gz_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[
            '/spiderfish/drivers/front_camera/image',
            '/spiderfish/drivers/front_camera/depth_image',
            '/spiderfish/drivers/down_camera',
        ],
        remappings=[
            ('/spiderfish/drivers/front_camera/image', '/spiderfish/drivers/front_camera/image_raw'),
            ('/spiderfish/drivers/front_camera/depth_image', '/spiderfish/drivers/front_camera/depth/image_raw')
        ],
        output='screen'
    )

    ld.add_action(world_arg)
    ld.add_action(add_model_path)
    ld.add_action(gz_sim)
    ld.add_action(ros_gz_bridge)
    ld.add_action(ros_gz_image_bridge)
    return ld