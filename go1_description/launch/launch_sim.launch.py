import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node



def generate_launch_description():

    package_name='go1_description'

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )
    params_file = os.path.join(get_package_share_directory(package_name), 'config', 'nav2_params.yaml')
    map_dir = os.path.join(get_package_share_directory(package_name), 'maps', 'warehouse_map.yaml')
    default_world = os.path.join(
        get_package_share_directory(package_name),
        'world',
        'empty.world'
        )       
    pkg_bot = get_package_share_directory(package_name)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Использовать симуляционное время'
    )
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
             )

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'my_bot',
                                   '-z', '0.4'],
                        output='screen')



    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )
        # Запуск контроллеров
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    joint_group_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_group_controller"],
        output="screen",
    )

    # Ноды для управления роботом
    controller = Node(
        package='go1_controller',
        executable='robot_controller_gazebo.py',
        name='quadruped_controller',
        output='screen',
    )


    cmd_vel_pub = Node(
        package='go1_controller',
        executable='cmd_vel_pub.py',
        name='cmd_vel_pub',
        output='screen',
    )

    odom =Node(
        package='go1_controller',
        executable='QuadrupedOdometryNode.py',
        name='odom',
        output='screen'
    )
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bot, 'launch', 'nav2', 'bringup_launch.py')),
        launch_arguments={
            'map': map_dir,
            'use_namespace': 'False',
            'params_file': params_file,
            'autostart': 'true',
            'use_sim_time': 'true',
            'log_level': 'warn',
            'map_server': 'True'
        }.items()
    )




    # Launch them all!
    return LaunchDescription([
        declare_use_sim_time,
        rsp,
        world_arg,
        gazebo,
        spawn_entity,
        ros_gz_bridge,
        joint_state_broadcaster,
        joint_group_controller,
        controller,
        # cmd_vel_pub,
        odom,
        # bringup_cmd
    ])
