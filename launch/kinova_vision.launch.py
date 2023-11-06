from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml

configurable_parameters = [
    {"name": "device", "default": "192.168.1.10", "description": "Device IPv4 address"},
    {"name": "camera", "default": "camera", "description": "'camera' should uniquely identify the device. All topics are pushed down into the 'camera' namespace."},
    {"name": "camera_link_frame_id", "default": "camera_link", "description": "Camera link frame identifier"},
    {"name": "color_frame_id", "default": "camera_color_frame", "description": "Color camera frame identifier"},
    {"name": "depth_frame_id", "default": "camera_depth_frame", "description": "Depth camera frame identifier"},
    {"name": "color_camera_info_url", "default": "", "description": "URL of custom calibration file for color camera. See camera_info_manager docs for calibration URL details"},
    {"name": "depth_camera_info_url", "default": "", "description": "URL of custom calibration file for depth camera. See camera_info_manager docs for calibration URL details"},
    {"name": "depth_registration", "default": "false", "description": "Hardware depth registration"},
    {"name": "depth_rtsp_element_config", "default": "depth latency=30", "description": "RTSP element configuration for depth stream"},
    {"name": "depth_rtp_depay_element_config", "default": "rtpgstdepay", "description": "RTP element configuration for depth stream"},
    {"name": "color_rtsp_element_config", "default": "color latency=30", "description": "RTSP element configuration for color stream"},
    {"name": "color_rtp_depay_element_config", "default": "rtph264depay", "description": "RTP element configuration for color stream"},
    {"name": "respawn", "default": "false", "description": ""},
]


def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param["name"], default_value=param["default"], description=param["description"]) for param in parameters]


def set_configurable_parameters(parameters):
    return dict([(param["name"], LaunchConfiguration(param["name"])) for param in parameters])


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)


def launch_setup(context, *args, **kwargs):

    # Depth Node Configuration
    depth_node = Node(
        package="kinova_vision",
        namespace=LaunchConfiguration('camera'),
        executable="kinova_vision_node",
        name="kinova_vision_depth",
        output="both",
        parameters=[{
            "camera_type": "depth",
            "camera_name": "depth",
            "camera_info_url_default": "package://kinova_vision/launch/calibration/default_depth_calib_%ux%u.ini",
            "camera_info_url_user": LaunchConfiguration("depth_camera_info_url").perform(context),
            "stream_config": "rtspsrc location=rtsp://" + LaunchConfiguration("device").perform(context) + "/" + LaunchConfiguration("depth_rtsp_element_config").perform(context) + " ! " + LaunchConfiguration("depth_rtp_depay_element_config").perform(context),
            "frame_id": LaunchConfiguration("depth_frame_id").perform(context),
        }],
        remappings=[
            ("camera_info", "depth/camera_info"),
            ("image_raw", "depth/image_raw"),
            ("image_raw/compressed", "depth/image_raw/compressed"),
            ("image_raw/compressedDepth", "depth/image_raw/compressedDepth"),
            ("image_raw/theora", "depth/image_raw/theora"),
        ],
    )

    color_node = Node(
        package="kinova_vision",
        namespace=LaunchConfiguration('camera'),
        executable="kinova_vision_node",
        name="kinova_vision_color",
        output="both",
        parameters=[{
            "camera_type": "color",
            "camera_name": "color",
            "camera_info_url_default": "package://kinova_vision/launch/calibration/default_color_calib_%ux%u.ini",
            "camera_info_url_user": LaunchConfiguration("color_camera_info_url").perform(context),
            "stream_config": "rtspsrc location=rtsp://" + LaunchConfiguration("device").perform(context) + "/" + LaunchConfiguration("color_rtsp_element_config").perform(context) + " ! " + LaunchConfiguration("color_rtp_depay_element_config").perform(context) + " ! avdec_h264 ! videoconvert",
            "frame_id": LaunchConfiguration("color_frame_id").perform(context),
        }],
        remappings=[
            ("camera_info", "color/camera_info"),
            ("camera_info", "color/camera_info"),
            ("image_raw", "color/image_raw"),
            ("image_raw/compressed", "color/image_raw/compressed"),
            ("image_raw/compressedDepth", "color/image_raw/compressedDepth"),
            ("image_raw/theora", "color/image_raw/theora"),
        ],
    )

    # Static Transformation Publishers
    camera_depth_tf_publisher = Node(
        package='tf2_ros',
        namespace=LaunchConfiguration('camera'),
        executable='static_transform_publisher',
        name='camera_depth_tf_publisher',
        output="both",
        arguments=['-0.0195', '-0.005', '0', '0', '0', '0',
                    LaunchConfiguration('camera_link_frame_id'),
                    LaunchConfiguration('depth_frame_id')],
    )

    camera_color_tf_publisher = Node(
        package='tf2_ros',
        namespace=LaunchConfiguration('camera'),
        executable='static_transform_publisher',
        name='camera_color_tf_publisher',
        output="both",
        arguments=['0', '0', '0', '0', '0', '0',
                    LaunchConfiguration('camera_link_frame_id'),
                    LaunchConfiguration('color_frame_id')],
    )
    
    return [depth_node, color_node, camera_depth_tf_publisher, camera_color_tf_publisher]


def generate_launch_description():
    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        OpaqueFunction(function=launch_setup)
    ])
