import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():


    #Arguments 
    arg_camera_prefixes_desc = DeclareLaunchArgument('arg_camera_prefixes', default_value="[cam1,cam2,cam3,cam4]")
    arg_output_mode_desc = DeclareLaunchArgument('arg_output_mode', default_value="0")
    arg_out_file_name_desc = DeclareLaunchArgument('arg_out_file_name', default_value="no filename")
    arg_enable_depth_ir_compression_desc = DeclareLaunchArgument('arg_enable_depth_ir_compression', default_value="True")

    # Parameters for TF
    var_base_frame_optical = "stitch_frame_link_optical"
    var_base_frame = "stitch_frame_link"
    """
         map
          ^
          |
        camera_device
          ^
          |
        camera_optical
    """
    var_parent_frame = "map"
    var_child_frame = var_base_frame_optical

    adi_3dtof_image_stitching_node_desc = Node(
                                        package='adi_3dtof_image_stitching',
                                        namespace='image_stitching',
                                        executable='adi_3dtof_image_stitching_node',
                                        name='adi_3dtof_image_stitching_node',
                                        output="screen",
                                        parameters=[{
                                            'param_camera_link': var_base_frame,
                                            'param_camera_prefixes': LaunchConfiguration('arg_camera_prefixes'),
                                            'param_output_mode': LaunchConfiguration('arg_output_mode'),
                                            'param_out_file_name': LaunchConfiguration('arg_out_file_name'),
                                            'param_enable_depth_ir_compression': LaunchConfiguration('arg_enable_depth_ir_compression'),
                                        }],
                                        on_exit=launch.actions.Shutdown()
                                    )

    map_to_base_tf_desc = Node(
                                   package='tf2_ros',
                                   namespace='image_stitching',
                                   executable='static_transform_publisher',
                                   name=f'{var_base_frame}_tf',
                                   output="screen",
                                   arguments=["--x", "0", 
                                              "--y", "0", 
                                              "--z", "0", 
                                              "--roll", "0", 
                                              "--pitch", "0", 
                                              "--yaw", "0", 
                                              "--frame-id", f"{var_parent_frame}", 
                                              "--child-frame-id", f"{var_base_frame}"]
                               )

    rviz_desc = Node(
                    package='rviz2',
                    namespace='image_stitching',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', [PathJoinSubstitution(([
                                        FindPackageShare('adi_3dtof_image_stitching'),
                                        'rviz',
                                        'adi_3dtof_image_stitching.rviz'
                                        ]))]]
                    )

    return LaunchDescription([
        arg_camera_prefixes_desc,
        arg_output_mode_desc,
        arg_out_file_name_desc,
        arg_enable_depth_ir_compression_desc,
        adi_3dtof_image_stitching_node_desc,
        map_to_base_tf_desc,
        rviz_desc
    ])
