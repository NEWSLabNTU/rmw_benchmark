#!/usr/bin/env python3
"""
RMW Stress Test Launch File

Launches a component container with:
- gscam publisher (high resolution, high FPS video)
- Image subscriber (measures latency and throughput)

Both nodes are loaded as composable nodes for efficient communication.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare video parameters
    declare_width = DeclareLaunchArgument(
        'width',
        default_value='1920',
        description='Camera width'
    )

    declare_height = DeclareLaunchArgument(
        'height',
        default_value='1080',
        description='Camera height'
    )

    declare_fps = DeclareLaunchArgument(
        'fps',
        default_value='30',
        description='Frame rate'
    )

    declare_camera_name = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Camera name'
    )

    declare_video_device = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='Video device path (e.g., /dev/video0 or v4l2src or videotestsrc)'
    )

    # Declare GStreamer pipeline (optional, overrides width/height/fps if provided)
    declare_gstreamer_pipeline = DeclareLaunchArgument(
        'gstreamer_pipeline',
        default_value='',
        description='Complete GStreamer pipeline (if empty, uses videotestsrc with width/height/fps)'
    )

    # Declare QoS parameters
    declare_qos_reliability = DeclareLaunchArgument(
        'qos_reliability',
        default_value='RELIABLE',
        description='QoS Reliability: RELIABLE or BEST_EFFORT'
    )

    declare_qos_durability = DeclareLaunchArgument(
        'qos_durability',
        default_value='VOLATILE',
        description='QoS Durability: VOLATILE or TRANSIENT_LOCAL'
    )

    declare_qos_history = DeclareLaunchArgument(
        'qos_history',
        default_value='KEEP_LAST',
        description='QoS History: KEEP_LAST or KEEP_ALL'
    )

    declare_qos_history_depth = DeclareLaunchArgument(
        'qos_history_depth',
        default_value='10',
        description='QoS History Depth (for KEEP_LAST)'
    )

    declare_qos_deadline_ms = DeclareLaunchArgument(
        'qos_deadline_ms',
        default_value='0',
        description='QoS Deadline in milliseconds (0 = disabled)'
    )

    declare_qos_liveliness = DeclareLaunchArgument(
        'qos_liveliness',
        default_value='AUTOMATIC',
        description='QoS Liveliness: AUTOMATIC, MANUAL_BY_TOPIC, or SYSTEM_DEFAULT'
    )

    declare_qos_liveliness_lease_ms = DeclareLaunchArgument(
        'qos_liveliness_lease_ms',
        default_value='0',
        description='QoS Liveliness Lease Duration in milliseconds (0 = infinite)'
    )

    # Component container
    container = ComposableNodeContainer(
        name='stress_test_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # Image subscriber node
            ComposableNode(
                package='rmw_stress_test',
                plugin='stress_test::ImageSubscriberNode',
                name='image_subscriber',
                parameters=[{
                    'topic_name': '/camera/image_raw',
                    'show_stats_interval': 2.0,
                    'qos_reliability': LaunchConfiguration('qos_reliability'),
                    'qos_durability': LaunchConfiguration('qos_durability'),
                    'qos_history': LaunchConfiguration('qos_history'),
                    'qos_history_depth': LaunchConfiguration('qos_history_depth'),
                    'qos_deadline_ms': LaunchConfiguration('qos_deadline_ms'),
                    'qos_liveliness': LaunchConfiguration('qos_liveliness'),
                    'qos_liveliness_lease_ms': LaunchConfiguration('qos_liveliness_lease_ms'),
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='both',
    )

    # Note: gscam needs to be loaded separately as it may not be a composable node
    # If gscam2 is available, it supports composition. Otherwise, run as separate node.

    def launch_setup(context, *args, **kwargs):
        width = LaunchConfiguration('width').perform(context)
        height = LaunchConfiguration('height').perform(context)
        fps = LaunchConfiguration('fps').perform(context)
        camera_name = LaunchConfiguration('camera_name').perform(context)
        video_device = LaunchConfiguration('video_device').perform(context)
        gstreamer_pipeline = LaunchConfiguration('gstreamer_pipeline').perform(context)

        # Create GSCAM configuration string
        if gstreamer_pipeline:
            # Use provided pipeline
            gscam_config = gstreamer_pipeline
            print(f"[stress_test.launch] Using custom GStreamer pipeline: {gscam_config}")
        else:
            # Default: videotestsrc for testing (works without camera)
            gscam_config = f"videotestsrc pattern=smpte ! video/x-raw,width={width},height={height},framerate={fps}/1 ! videoconvert"
            print(f"[stress_test.launch] Using default pipeline: {width}x{height}@{fps}")
            # For real camera, use:
            # gscam_config = f"v4l2src device={video_device} ! video/x-raw,width={width},height={height},framerate={fps}/1 ! videoconvert"

        actions = []

        # Check if gscam2 is available (composable version)
        try:
            from launch_ros.actions import Node

            # Launch gscam as a regular node (gscam is not always composable)
            gscam_node = Node(
                package='gscam',
                executable='gscam_node',
                name=camera_name,
                parameters=[{
                    'camera_name': camera_name,
                    'camera_info_url': '',
                    'gscam_config': gscam_config,
                    'frame_id': f'{camera_name}_optical_frame',
                    'sync_sink': True,
                    'use_gst_timestamps': False,
                }],
                remappings=[
                    ('camera/image_raw', f'/{camera_name}/image_raw'),
                ],
                output='both',
            )
            actions.append(gscam_node)
        except Exception as e:
            print(f"Warning: Could not configure gscam: {e}")

        return actions

    return LaunchDescription([
        # Video parameters
        declare_width,
        declare_height,
        declare_fps,
        declare_camera_name,
        declare_video_device,
        declare_gstreamer_pipeline,
        # QoS parameters
        declare_qos_reliability,
        declare_qos_durability,
        declare_qos_history,
        declare_qos_history_depth,
        declare_qos_deadline_ms,
        declare_qos_liveliness,
        declare_qos_liveliness_lease_ms,
        # Container and nodes
        container,
        OpaqueFunction(function=launch_setup),
    ])
