from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='192.168.1.10', description='Device IPv4 address'),
        DeclareLaunchArgument('camera', default_value='camera', description="'camera' should uniquely identify the device. All topics are pushed down into the 'camera' namespace."),
        DeclareLaunchArgument('camera_link_frame_id', default_value='camera_link', description='Camera link frame identifier'),
        DeclareLaunchArgument('color_frame_id', default_value='camera_color_frame', description='Color camera frame identifier'),
        DeclareLaunchArgument('depth_frame_id', default_value='camera_depth_frame', description='Depth camera frame identifier'),
        DeclareLaunchArgument('color_camera_info_url', default_value='', description='URL of custom calibration file for color camera. See camera_info_manager docs for calibration URL details'),
        DeclareLaunchArgument('depth_camera_info_url', default_value='', description='URL of custom calibration file for depth camera. See camera_info_manager docs for calibration URL details'),
        DeclareLaunchArgument('depth_registration', default_value='false', description='Hardware depth registration'),
        DeclareLaunchArgument('rgb', default_value='color', description='Argument for remapping all device namespaces'),
        DeclareLaunchArgument('depth', default_value='depth', description='Argument for remapping all device namespaces'),
        
        # Depth stream configs
        DeclareLaunchArgument('depth_rtsp_media_url', description='RTSP media URL for depth stream', default_value=['rtsp://', LaunchConfiguration('device'), '/depth']),
        DeclareLaunchArgument('depth_rtp_depay_element_config', default_value='rtpgstdepay', description='RTP element configuration for depth stream'),
        DeclareLaunchArgument('depth_rtsp_element_config', description='RTSP element configuration for depth stream', default_value=['rtspsrc location=', LaunchConfiguration('depth_rtsp_media_url'),' latency=30']),
        DeclareLaunchArgument('depth_stream_config', default_value=[LaunchConfiguration('depth_rtsp_element_config'), ' ! ', LaunchConfiguration('depth_rtp_depay_element_config')]),

        # Color stream configs
        DeclareLaunchArgument('color_rtsp_media_url', description='RTSP media URL for color stream', default_value=['rtsp://', LaunchConfiguration('device'), '/color']),
        DeclareLaunchArgument('color_rtp_depay_element_config', default_value='rtph264depay', description='RTP element configuration for color stream'),
        DeclareLaunchArgument('color_rtsp_element_config', description='RTSP element configuration for color stream', default_value=['rtspsrc location=', LaunchConfiguration('color_rtsp_media_url'),' latency=30']),
        DeclareLaunchArgument('color_stream_config', default_value=[LaunchConfiguration('color_rtsp_element_config'), ' ! ', LaunchConfiguration('color_rtp_depay_element_config'),' ! avdec_h264 ! videoconvert']),

        # Depth Node Configuration
        Node(
          package='kinova_vision',
          executable='kinova_vision_node',
          namespace=LaunchConfiguration('camera'),
          name='kinova_vision_depth',
          output='both',
          parameters=[
            {'camera_type': 'depth'},
            {'camera_name': 'depth'},
            {'camera_info_url_default': 'package://kinova_vision/launch/calibration//default_depth_calib_%ux%u.ini'},
            {'camera_info_url_user': LaunchConfiguration('depth_camera_info_url')},
            {'frame_id': LaunchConfiguration('depth_frame_id')},
            {'stream_config': LaunchConfiguration('depth_stream_config')},
          ]
        ),

        # Color Node Configuration
        Node(
          package='kinova_vision',
          executable='kinova_vision_node',
          namespace=LaunchConfiguration('camera'),
          name='kinova_vision_color',
          output='both',
          parameters=[
            {'camera_type': 'color'},
            {'camera_name': 'color'},
            {'camera_info_url_default': 'package://kinova_vision/launch/calibration/default_color_calib_%ux%u.ini'},
            {'camera_info_url_user': LaunchConfiguration('color_camera_info_url')},
            {'frame_id': LaunchConfiguration('color_frame_id')},
            {'stream_config': LaunchConfiguration('color_stream_config')},
          ]
        ),

        # Static Transformation Publishers
        Node(
          package='tf2_ros',
          executable='static_transform_publisher',
          name='camera_depth_tf_publisher',
          arguments=['-0.0195', '-0.005', '0', '0', '0', '0',
                      LaunchConfiguration('camera_link_frame_id'),
                      LaunchConfiguration('depth_frame_id')],
        ),
        Node(
          package='tf2_ros',
          executable='static_transform_publisher',
          name='camera_color_tf_publisher',
          arguments=['0', '0', '0', '0', '0', '0',
                      LaunchConfiguration('camera_link_frame_id'),
                      LaunchConfiguration('color_frame_id')],
        ),
    ])
