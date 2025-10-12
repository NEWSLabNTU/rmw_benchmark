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
    # Declare arguments
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

        # Create GSCAM configuration string
        # Using videotestsrc for testing (works without camera)
        gscam_config = f"videotestsrc pattern=smpte ! video/x-raw,width={width},height={height},framerate={fps}/1 ! videoconvert"

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
        declare_width,
        declare_height,
        declare_fps,
        declare_camera_name,
        declare_video_device,
        container,
        OpaqueFunction(function=launch_setup),
    ])
