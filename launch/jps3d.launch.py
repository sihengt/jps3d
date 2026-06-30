from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # --------------------------------------------------------------------------
    # Launch arguments
    # --------------------------------------------------------------------------
    config_arg = DeclareLaunchArgument(
        "config",
        default_value="jps3d.yaml",
        description="Params file to load: jps3d.yaml",
    )

    params_file = PathJoinSubstitution([
        FindPackageShare("jps3d"),
        "config",
        LaunchConfiguration("config"),
    ])

    # --------------------------------------------------------------------------
    # jps3d_node
    # --------------------------------------------------------------------------
    jps3d_node = Node(
        package="jps3d",
        executable="jps3d_node",
        name="jps3d_node",
        output="screen",
        parameters=[
            params_file,
        ],
    )

    # --------------------------------------------------------------------------
    return LaunchDescription(
        [
            config_arg,
            jps3d_node,
        ]
    )
