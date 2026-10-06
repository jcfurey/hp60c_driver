# Copyright 2026 jcfurey
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Start the HP60C driver as a component in its own container."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument('namespace', default_value='hp60c'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('hp60c_driver'), 'config', 'hp60c.yaml']),
            description='Parameter file (exposure, gain, filter, ...). '
                        'The arguments below override it.'),
        DeclareLaunchArgument(
            'device', default_value='',
            description='/dev/videoN; empty = find the camera by USB id 3482:6723'),
        DeclareLaunchArgument('color_frame_id', default_value='hp60c_color_optical_frame'),
        DeclareLaunchArgument('depth_frame_id', default_value='hp60c_depth_optical_frame'),
        DeclareLaunchArgument(
            'base_frame_id', default_value='',
            description='If set, publish a body frame (x forward, z up) at the colour '
                        'camera. Leave empty when a URDF places the optical frame.'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('best_effort', default_value='false'),
        DeclareLaunchArgument('use_cuda', default_value='true'),
        DeclareLaunchArgument('filter', default_value='true',
                              description='Publish */image_filtered depth too'),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]
    container = ComposableNodeContainer(
        name='hp60c_container',
        namespace=LaunchConfiguration('namespace'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='hp60c_driver',
                plugin='hp60c_driver::Hp60cNode',
                name='hp60c',
                namespace=LaunchConfiguration('namespace'),
                parameters=[
                    LaunchConfiguration('params_file'),
                    {
                        'device': LaunchConfiguration('device'),
                        'color_frame_id': LaunchConfiguration('color_frame_id'),
                        'depth_frame_id': LaunchConfiguration('depth_frame_id'),
                        'base_frame_id': LaunchConfiguration('base_frame_id'),
                        'publish_tf': LaunchConfiguration('publish_tf'),
                        'best_effort': LaunchConfiguration('best_effort'),
                        'use_cuda': LaunchConfiguration('use_cuda'),
                        'filter.enabled': LaunchConfiguration('filter'),
                    },
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        ros_arguments=['--log-level', LaunchConfiguration('log_level')],
        output='screen',
    )
    return LaunchDescription(args + [container])
