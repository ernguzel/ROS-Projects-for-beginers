import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription ,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration ,Command

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node



def generate_launch_description():



    package_name='kasva_sekli' 
    
    xacro_file = os.path.join(
        get_package_share_directory(package_name),  
        'description',
        'robot.urdf.xacro'
    )

    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'lines.world'
        )    

    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
             )

    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'robot',
                                   '-z', '0.1'],
                        output='screen')
    

    robot_state_publisher = Node(  
                        package='robot_state_publisher',
                        executable='robot_state_publisher',
                        parameters=[{
                            'robot_description': Command(['xacro', ' ', xacro_file])
                                    }],
                        output='screen'
        )


 

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

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )

    rviz2_node = Node(
        package= "rviz2",
        executable="rviz2",
        output="screen"
    )




    # Launch them all!
    return LaunchDescription([
        world_arg,
        gazebo,
        spawn_entity,
        rviz2_node,
        robot_state_publisher,
        ros_gz_bridge,
        ros_gz_image_bridge
    ])
