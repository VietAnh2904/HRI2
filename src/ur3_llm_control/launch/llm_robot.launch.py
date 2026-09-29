from pathlib import Path
import os
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    own = Path(get_package_share_directory('ur3_llm_control'))
    ur = Path(get_package_share_directory('ur_moveit_config'))
    desc = xacro.process_file(str(own/'urdf/ur3e_suction.urdf.xacro'), mappings={
        'name': 'ur', 'ur_type': 'ur3e', 'sim_gazebo': 'true',
        'safety_limits': 'true', 'simulation_controllers': str(own/'config/controllers.yaml')}).toxml()
    srdf = xacro.process_file(str(ur/'srdf/ur.srdf.xacro'), mappings={'name': 'ur', 'prefix': ''}).toxml()
    # The suction cup is fixed to the wrist; its contact with the wrist is intentional.
    srdf = srdf.replace('</robot>', '<disable_collisions link1="suction_tip" link2="wrist_3_link" reason="Adjacent"/></robot>')
    def load(name):
        return yaml.safe_load((ur/'config'/name).read_text())
    kin = load('kinematics.yaml')
    if '/**' in kin:
        kin = kin['/**']['ros__parameters']['robot_description_kinematics']
    robot = {'robot_description': desc, 'robot_description_semantic': srdf,
             'robot_description_kinematics': kin,
             'robot_description_planning': load('joint_limits.yaml'), 'use_sim_time': True}
    ompl = load('ompl_planning.yaml')
    ompl.update({'planning_plugin': 'ompl_interface/OMPLPlanner',
                 'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
                 'start_state_max_bounds_error': 0.05, 'path_tolerance': 0.001, 'resample_dt': 0.02})
    controllers = load('controllers.yaml')
    controllers['scaled_joint_trajectory_controller']['default'] = False
    controllers['joint_trajectory_controller']['default'] = True
    moveit = Node(package='moveit_ros_move_group', executable='move_group', output='log', parameters=[robot, {
        'move_group': ompl, 'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': controllers, 'moveit_manage_controllers': False,
        'trajectory_execution.allowed_start_tolerance': 0.02,
        'trajectory_execution.allowed_execution_duration_scaling': 2.0,
        'trajectory_execution.allowed_goal_duration_margin': 3.0,
        'publish_robot_description_semantic': True, 'publish_planning_scene': True,
        'publish_geometry_updates': True, 'publish_state_updates': True, 'publish_transforms_updates': True}])
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('gazebo_ros'))/'launch/gazebo.launch.py')),
        launch_arguments={'world': str(own/'worlds/table.world'), 'gui': LaunchConfiguration('gui'), 'verbose': 'false'}.items())
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', str(Path(get_package_prefix('ur3_llm_control'))/'lib') + ':' + os.environ.get('GAZEBO_PLUGIN_PATH','')),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        gazebo,
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[robot], output='log'),
        Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-entity','ur','-topic','robot_description'], output='screen'),
        Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster','--controller-manager-timeout','120'], output='screen'),
        Node(package='controller_manager', executable='spawner', arguments=['joint_trajectory_controller','--controller-manager-timeout','120'], output='screen'),
        moveit,
    ])
