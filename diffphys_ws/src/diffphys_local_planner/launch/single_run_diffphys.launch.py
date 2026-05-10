import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import UnlessCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    obj_num = LaunchConfiguration('obj_num', default=10)
    drone_id = LaunchConfiguration('drone_id', default=0)

    map_size_x = LaunchConfiguration('map_size_x', default=50.0)
    map_size_y = LaunchConfiguration('map_size_y', default=25.0)
    map_size_z = LaunchConfiguration('map_size_z', default=2.0)
    odom_topic = LaunchConfiguration('odom_topic', default='visual_slam/odom')

    use_mockamap = LaunchConfiguration('use_mockamap', default=False)
    use_dynamic = LaunchConfiguration('use_dynamic', default=True)

    checkpoint = LaunchConfiguration('checkpoint', default='/workspace/DiffPhysDrone/output/checkpoint0004.pth')

    map_generator_node = Node(
        package='map_generator',
        executable='random_forest',
        name='random_forest',
        output='screen',
        parameters=[
            {'map/x_size': 26.0},
            {'map/y_size': 20.0},
            {'map/z_size': 3.0},
            {'map/resolution': 0.1},
            {'ObstacleShape/seed': 1.0},
            {'map/obs_num': 0},
            {'ObstacleShape/lower_rad': 0.5},
            {'ObstacleShape/upper_rad': 1.0},
            {'ObstacleShape/lower_hei': 0.0},
            {'ObstacleShape/upper_hei': 3.0},
            {'map/circle_num': 125},
            {'ObstacleShape/radius_l': 1.0},
            {'ObstacleShape/radius_h': 1.5},
            {'ObstacleShape/z_l': 1.0},
            {'ObstacleShape/z_h': 1.2},
            {'ObstacleShape/theta': 0.5},
            {'min_distance': 0.8}
        ],
        condition=UnlessCondition(use_mockamap)
    )

    diffphys_node = Node(
        package='diffphys_local_planner',
        executable='local_planner_node',
        name=['drone_', drone_id, '_diffphys_planner'],
        output='screen',
        parameters=[{
            'checkpoint': checkpoint,
            'drone_id': drone_id,
            'max_vel': 2.0,
            'time_forward': 1.0,
        }],
        remappings=[
            ('drone_0_visual_slam/odom', ['drone_', drone_id, '_visual_slam/odom']),
            ('drone_0_pcl_render_node/depth', ['drone_', drone_id, '_pcl_render_node/depth']),
            ('drone_0_planning/bspline', ['drone_', drone_id, '_planning/bspline']),
            ('drone_0_planning/pos_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
        ]
    )

    simulator_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'simulator.launch.py')),
        launch_arguments={
            'use_dynamic': use_dynamic,
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'init_x_': str(0.0),
            'init_y_': str(0.0),
            'init_z_': str(1.0),
            'odometry_topic': odom_topic,
            'use_mockamap': str(False)
        }.items()
    )
    
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('obj_num', default_value=obj_num))
    ld.add_action(DeclareLaunchArgument('drone_id', default_value=drone_id))
    ld.add_action(DeclareLaunchArgument('map_size_x', default_value=map_size_x))
    ld.add_action(DeclareLaunchArgument('map_size_y', default_value=map_size_y))
    ld.add_action(DeclareLaunchArgument('map_size_z', default_value=map_size_z))
    ld.add_action(DeclareLaunchArgument('odom_topic', default_value=odom_topic))
    ld.add_action(DeclareLaunchArgument('use_mockamap', default_value=use_mockamap))
    ld.add_action(DeclareLaunchArgument('use_dynamic', default_value=use_dynamic))
    ld.add_action(DeclareLaunchArgument('checkpoint', default_value=checkpoint))

    ld.add_action(map_generator_node)
    ld.add_action(diffphys_node)
    ld.add_action(simulator_include)

    return ld
