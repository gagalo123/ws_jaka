from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('py_test')
    urdf_share = get_package_share_directory('dual_arm')
    urdf_name = 'dual_arm.urdf'

    urdf_model_path = os.path.join(urdf_share, f'urdf/{urdf_name}')
    rviz2_config_path = os.path.join(pkg_share, 'config/test_urdf.rviz')
    # test2_node = Node(
    #     package='robot_tool',
    #     executable='test2',
    #     name='test2',
    #     output='screen',
    # )
    # print("rviz2_config_path : ", rviz2_config_path)
    robot_state_publisher_node = Node(package='robot_state_publisher',
                                    executable='robot_state_publisher',
                                    output='screen',
                                    arguments=[urdf_model_path])
    test_cllision_rviz2_node = Node(package='py_test',
                      executable='test_collision_rviz2',
                      emulate_tty=True,
                      name='test_collision_rviz2',
                      output='screen',
                      arguments=[
                                 '--sleep', '0.05', 
                                 '--stop-at-first-collision', 
                                 '--steps', '50', 
                                 '--interp-steps', '20',
                                #  '--ros-args', '--log-level', 'DEBUG'  # Set logging verbosity to DEBUG
                                 ])
    return LaunchDescription([
        robot_state_publisher_node,
        test_cllision_rviz2_node
        # Start rviz with the provided configuration file
    ])
