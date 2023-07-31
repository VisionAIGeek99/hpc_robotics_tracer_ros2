from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    urdf_tutorial_path = get_package_share_path('hpc_robotics_tracer_ros2')
    #default_model_path = urdf_tutorial_path / 'urdf/tracer_v1.urdf.xacro'
    default_rviz_config_path = urdf_tutorial_path / 'rviz/model_diaplay.rviz'

    path_to_urdf = get_package_share_path('hpc_robotics_tracer_ros2') / 'urdf' / 'tracer_v1.urdf.xacro'
    # path_to_urdf = get_package_share_path('hpc_robotics_tracer_ros2') / 'urdf' / '08-macroed.urdf.xacro'

    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    # model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
    #                                   description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', str(path_to_urdf)]), value_type=str
            )
        }]
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        gui_arg,
        # model_arg,
        # rviz_arg,
        # joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        robot_state_publisher_node,
        # rviz_node
    ])