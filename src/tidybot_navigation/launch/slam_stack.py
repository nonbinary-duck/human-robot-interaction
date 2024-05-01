import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
#
# Launch file based on the substitutions tutorial: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html
#

def generate_launch_description():

    # Line of code adapted from the nav2_bringup slam_launch.py file
    tidybot_dir = get_package_share_directory('tidybot_navigation');

    
    # Initialise a launch description object to add to
    ld = LaunchDescription();


    ld.add_action(
        # Include our Nav2 launch description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "navigation_launch.py"
                ])
            ]),
            launch_arguments={
                "use_sim_time": "True",
                "params_file": os.path.join( tidybot_dir, "params", "limo_nav2_params.yaml" )
            }.items()
        )
    );
    

    ld.add_action(
        # Include our SLAM Toolbox launch description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("slam_toolbox"),
                    "launch",
                    "online_async_launch.py"
                ])
            ]),
            launch_arguments={
                "use_sim_time": "True",
                "slam_params_file": os.path.join( tidybot_dir, "params", "limo_mapper_params_online_sync.yaml" )
            }.items()
        )
    );

    # Also launch rviz with our config
    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=[
                "-d",
                os.path.join( tidybot_dir, "rviz", "tidybot_nav.rviz" )
            ]
        )
    );


    # Return our launch description we've generated
    return ld;

