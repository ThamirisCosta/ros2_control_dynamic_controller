from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from yaml import safe_load

def generate_launch_description():
    # Configurações
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    description_pkg = get_package_share_directory('manipulator_description_pkg')
    
    # Caminhos dos arquivos
    urdf_path = os.path.join(description_pkg, 'urdf', 'manipulator_description.urdf')
    gazebo_urdf_path = os.path.join(description_pkg, 'urdf', 'manipulator_description_gazebo.urdf')
    controller_config_path = os.path.join(description_pkg, 'controllers', 'controllers.yaml')
    
    # Carregar conteúdo dos arquivos
    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    
    with open(controller_config_path, 'r') as f:
        controller_config = safe_load(f)['controller_manager']['ros__parameters']

    # Nós
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'source_list': ['joint_states'],
            'use_sim_time': use_sim_time
        }]
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time},
            controller_config
        ]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', gazebo_urdf_path, '-name', 'manipulator'],
        output='screen'
    )

    # Controladores
    joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    dynamic_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'computed_torque_controller'],
        output='screen'
    )
    #dynamic_controller = Node(
    #	package="controller_manager",
    #	executable="spawner.py",
    #	arguments=["computed_torque_controller"],
    #	output='screen'
    #)
    

    # Nó de publicação de posição desejada
    desired_position_publisher = Node(
        package='manipulator_move_pkg',
        executable='desired_position',
        parameters=[{
            'initial_delay': 5.0,
            'publish_rate': 10.0,
            'x_position': 1.391,
            'y_position': 0.01,
            'z_position': -0.004
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
        # Simulação
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('ros_gz_sim'),
                '/launch/gz_sim.launch.py'
            ]),
            launch_arguments={
            	'gz_args': '-r empty.sdf'
            }.items()
        ),
        
        # Nós do robô
        robot_state_publisher,
        joint_state_publisher,
        controller_manager,
        gz_spawn_entity,
        
        # Sequência de inicialização
        RegisterEventHandler(
            OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[dynamic_controller]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=dynamic_controller,
                on_start=[desired_position_publisher]
            )
        )
    ])
