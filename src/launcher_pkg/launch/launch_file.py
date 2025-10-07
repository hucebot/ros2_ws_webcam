from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
import os

def generate_launch_description():

    # params
    n_camera = 3

    # params list
    camera_devices = [
        DeclareLaunchArgument('video_device1', default_value=''),
        DeclareLaunchArgument('video_device2', default_value=''),
        DeclareLaunchArgument('video_device3', default_value='')
    ]

    declare_width = DeclareLaunchArgument('image_width', default_value='640')
    declare_height = DeclareLaunchArgument('image_height', default_value='480')
    declare_fps = DeclareLaunchArgument('framerate', default_value='30')
    declare_transport = DeclareLaunchArgument('transport_on', default_value='false') # when active, I publish Image type to be visualized also on RViz

    def create_nodes(context, *args, **kwargs):
        
        nodes = []

        transport_enabled = LaunchConfiguration('transport_on').perform(context) == 'true'

        for i in range(1, n_camera+1):

            device = LaunchConfiguration(f'video_device{i}').perform(context)

            if device:  # create node only if device is not empty
                
                nodes.append(
                    Node(
                        package='webcam_pkg',
                        executable='webcam_node',
                        name=f'webcam{i}',
                        output='screen',
                        parameters=[{
                            'video_device': device,
                            'image_width': LaunchConfiguration('image_width'),
                            'image_height': LaunchConfiguration('image_height'),
                            'framerate': LaunchConfiguration('framerate'),
                            'cam_frame_id': f'/webcam{i}'
                        }]
                    )
                )

                if transport_enabled:

                    nodes.append(
                        Node(
                            package='image_transport',
                            executable='republish',
                            name=f'webcam{i}_republish',
                            output='screen',
                            arguments=[
                                'compressed', 'raw',
                                '--ros-args',
                                '-r', f'in/compressed:=/webcam{i}/image_raw/compressed',
                                '-r', f'out:=/webcam{i}/image_raw'
                            ]
                        )
                    )

        return nodes

    return LaunchDescription(
        camera_devices + [declare_width, declare_height, declare_fps, declare_transport] + [
            OpaqueFunction(function=create_nodes)
        ]
    )