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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    args = [
        DeclareLaunchArgument('namespace', default_value='hp60c'),
        DeclareLaunchArgument(
            'device', default_value='',
            description='/dev/videoN; empty = find the camera by USB id 3482:6723'),
        DeclareLaunchArgument('color_frame_id', default_value='hp60c_color_optical_frame'),
        DeclareLaunchArgument('depth_frame_id', default_value='hp60c_depth_optical_frame'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('best_effort', default_value='false'),
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
                parameters=[{
                    'device': LaunchConfiguration('device'),
                    'color_frame_id': LaunchConfiguration('color_frame_id'),
                    'depth_frame_id': LaunchConfiguration('depth_frame_id'),
                    'publish_tf': LaunchConfiguration('publish_tf'),
                    'best_effort': LaunchConfiguration('best_effort'),
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )
    return LaunchDescription(args + [container])
