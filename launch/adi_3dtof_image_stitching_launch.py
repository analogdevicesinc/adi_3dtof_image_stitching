import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ParameterFile


def generate_launch_description():

    # Get the launch directory
    bringup_dir = get_package_share_directory('adi_3dtof_image_stitching') 
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')

    #Parameters
    profiling_envvar = SetEnvironmentVariable(
        'ENABLE_PROFILING', '0')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    # Specify the actions
    bringup_cam1 = GroupAction([            
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'adi_3dtof_adtf31xx_cam1_launch.py')),
            launch_arguments={'namespace': namespace}.items()),
    ])

    bringup_cam2 = GroupAction([            
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'adi_3dtof_adtf31xx_cam2_launch.py')),
            launch_arguments={'namespace': namespace}.items()),
    ])

    bringup_cam3 = GroupAction([            

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'adi_3dtof_adtf31xx_cam3_launch.py')),
            launch_arguments={'namespace': namespace}.items()),
    ])

    bringup_cam4 = GroupAction([            
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'adi_3dtof_adtf31xx_cam4_launch.py')),
            launch_arguments={'namespace': namespace}.items()),
    ])

    bringup_stitch = GroupAction([            
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir,'adi_3dtof_image_stitching_host_only_launch.py')),
            launch_arguments={'namespace': namespace}.items()),
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(profiling_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)

    # Add the actions to launch all of the nodes
    ld.add_action(bringup_cam1)
    ld.add_action(bringup_cam2)
    ld.add_action(bringup_cam3)
    ld.add_action(bringup_cam4)
    ld.add_action(bringup_stitch)

    return ld
