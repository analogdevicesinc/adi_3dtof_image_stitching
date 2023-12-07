import os
import launch
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.substitutions import LaunchConfiguration, PythonExpression

# configuration file names
config_json_file_names = ['config_crosby_old_modes.json',
                          'config_crosby_adsd3500_new_modes.json']
config_ini_file_names = ['RawToDepthAdsd3500_qmp.ini',
                         'RawToDepthAdsd3500_lr-qnative.ini']

package_dir = get_package_share_directory('adi_3dtof_adtf31xx') + "/../../../../src/adi_3dtof_adtf31xx/"

def modify_ini_path_in_json_file(config_file_name_of_tof_sdk):

    with open(config_file_name_of_tof_sdk, 'r+') as file:
        data = json.load(file)

    for i in range(len(config_json_file_names)):
        if (config_file_name_of_tof_sdk.rsplit("/", 1)[1] == config_json_file_names[i]):
            modified_file_path = package_dir + "config/" + config_ini_file_names[i] 

    data["DEPTH_INI"] = modified_file_path

    with open(config_file_name_of_tof_sdk, 'w') as file:
        json.dump(data, file, indent=4)


def generate_launch_description():

    # Height of the sensor from camera in meters
    arg_camera_height_from_ground_in_mtr_desc = DeclareLaunchArgument(
        'arg_camera_height_from_ground_in_mtr', default_value="0.15")

    # Input mode => 0:Real Time Sensor, 2:Rosbag bin
    arg_input_sensor_mode_desc = DeclareLaunchArgument(
        'arg_input_sensor_mode', default_value="2")

    # Input filename : Applicable only if the input mode is 2
    # Relative path to the launch file which gets executed. Here, launch file from install folder is considered
    arg_in_file_name_desc = DeclareLaunchArgument('arg_in_file_name', default_value= package_dir + "../adi_3dtof_input_video_files/adi_3dtof_height_170mm_yaw_0degrees_cam3.bin")

    # Enable RVL compression for depth and ir images
    arg_enable_depth_ir_compression_desc = DeclareLaunchArgument(
        'arg_enable_depth_ir_compression', default_value="True")

    # abThreshold
    arg_ab_threshold_desc = DeclareLaunchArgument(
        'arg_ab_threshold', default_value="10")

    # confidenceThreshold
    arg_confidence_threshold_desc = DeclareLaunchArgument(
        'arg_confidence_threshold', default_value="10")

    # Configuration fie name of ToF SDK
    #    "config_crosby_old_modes.json" - Sensor serial number starting with CR/DV - config_json_file_names[0]
    #    "config_crosby_adsd3500_new_modes.json" - Sensor serial number starting with AM - config_json_file_names[1]
    arg_config_file_name_of_tof_sdk_desc = DeclareLaunchArgument(
        'arg_config_file_name_of_tof_sdk', default_value= package_dir + "config/" + config_json_file_names[0])

    # Frame Type
    #    "qmp" - Sensor serial number starting with CR/DV
    #    "lr-qnative" - Sensor serial number starting with AM
    arg_frame_type_desc = DeclareLaunchArgument(
        'arg_frame_type', default_value="qmp")

    # Modifying the path to ini file in json file.
    if arg_input_sensor_mode_desc.default_value[0].text == '0':
        modify_ini_path_in_json_file(arg_config_file_name_of_tof_sdk_desc.default_value[0].text)

    # Parameters for TF
    var_ns_prefix_cam1 = "cam3"
    var_cam1_base_frame_optical = f"{var_ns_prefix_cam1}_adtf31xx_optical"
    var_cam1_base_frame = f"{var_ns_prefix_cam1}_adtf31xx"
    """
         map
          ^
          |
        camera_device
          ^
          |
        camera_optical
    """
    var_cam1_parent_frame = "map"
    var_cam1_child_frame = var_cam1_base_frame_optical

    # Optical to Device rotation, these are standard values, would never change
    var_cam_optical_to_base_roll = "-1.57"
    var_cam_optical_to_base_pitch = "0"
    var_cam_optical_to_base_yaw = "-1.57"

    # Camera position wrt map
    var_cam1_pos_x = "0.111"
    var_cam1_pos_y = "0.0"
    var_cam1_roll = "0.0"
    var_cam1_pitch = "0.0"
    var_cam1_yaw = "0.0"

    # adi_3dtof_adtf31xx_node Node description
    adi_3dtof_adtf31xx_node_desc = Node(
        package='adi_3dtof_adtf31xx',
        namespace=var_ns_prefix_cam1,
        executable='adi_3dtof_adtf31xx_node',
        name='adi_3dtof_adtf31xx_node',
        output="screen",
        parameters=[{
            'param_camera_link': var_cam1_base_frame_optical,
            'param_input_sensor_mode': LaunchConfiguration('arg_input_sensor_mode'),
            'param_input_file_name': LaunchConfiguration('arg_in_file_name'),
            'param_config_file_name_of_tof_sdk': LaunchConfiguration('arg_config_file_name_of_tof_sdk'),
            'param_frame_type': LaunchConfiguration('arg_frame_type'),
            'param_enable_depth_ir_compression': LaunchConfiguration('arg_enable_depth_ir_compression'),
            'param_ab_threshold': LaunchConfiguration('arg_ab_threshold'),
            'param_confidence_threshold': LaunchConfiguration('arg_confidence_threshold')
        }],
        on_exit=launch.actions.Shutdown()
    )

    # cam1_base_to_optical TF description
    cam1_base_to_optical_tf_desc = Node(
        package='tf2_ros',
        namespace=var_ns_prefix_cam1,
        executable='static_transform_publisher',
        name=f'{var_cam1_base_frame_optical}_tf',
        output="screen",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                                                  "--roll", f"{var_cam_optical_to_base_roll}",
                                                  "--pitch", f"{var_cam_optical_to_base_pitch}",
                                                  "--yaw", f"{var_cam_optical_to_base_yaw}",
                                                  "--frame-id", f"{var_cam1_base_frame}",
                                                  "--child-frame-id", f"{var_cam1_child_frame}"]
    )

    # map_to_cam1_base TF description
    map_to_cam1_base_tf_desc = Node(
        package='tf2_ros',
        namespace=var_ns_prefix_cam1,
        executable='static_transform_publisher',
        name=f'{var_cam1_base_frame}_tf',
        output="screen",
        arguments=["--x", f"{var_cam1_pos_x}",
                   "--y", f"{var_cam1_pos_y}",
                   "--z", PythonExpression(LaunchConfiguration(
                       'arg_camera_height_from_ground_in_mtr')),
                   "--roll", f"{var_cam1_roll}",
                   "--pitch", f"{var_cam1_pitch}",
                   "--yaw", f"{var_cam1_yaw}",
                   "--frame-id", f"{var_cam1_parent_frame}",
                   "--child-frame-id", f"{var_cam1_base_frame}"]
    )

    # RVIZ description
    rviz_desc = Node(
        package='rviz2',
        namespace=var_ns_prefix_cam1,
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [PathJoinSubstitution(([
            FindPackageShare('adi_3dtof_adtf31xx'),
            'rviz',
            'adi_3dtof_adtf31xx.rviz'
        ]))]]
    )

    # Launch
    return LaunchDescription([
        arg_camera_height_from_ground_in_mtr_desc,
        arg_input_sensor_mode_desc,
        arg_in_file_name_desc,
        arg_enable_depth_ir_compression_desc,
        arg_ab_threshold_desc,
        arg_confidence_threshold_desc,
        arg_config_file_name_of_tof_sdk_desc,
        arg_frame_type_desc,
        adi_3dtof_adtf31xx_node_desc,
        cam1_base_to_optical_tf_desc,
        map_to_cam1_base_tf_desc,
        # rviz_desc
    ])
