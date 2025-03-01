#  Copyright 2025 Walter Lucetti
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http:#www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction,
    LogInfo
)
from launch.substitutions import (
    TextSubstitution
)
from launch_ros.actions import (
    ComposableNodeContainer,
    LoadComposableNodes
)
from launch_ros.descriptions import ComposableNode

def launch_setup(context, *args, **kwargs):

    # List of actions to be launched
    actions = []

    namespace_val = 'test_composition'
    
    # ROS 2 Component Container
    container_name = 'test_container'
    distro = os.environ['ROS_DISTRO']
    
    info = '* Starting Composable node container: /' + namespace_val + '/' + container_name
    actions.append(LogInfo(msg=TextSubstitution(text=info)))

    test_container = ComposableNodeContainer(
        name=container_name,
        namespace=namespace_val,
        package='rclcpp_components',
        executable='component_container_isolated',
        arguments=['--use_multi_threaded_executor --ros-args', '--log-level', 'info'],
        output='screen',
    )
    actions.append(test_container)

    # Timer Component
    info = '* Starting a Timer Component'
    actions.append(LogInfo(msg=TextSubstitution(text=info)))

    timer_component = ComposableNode(
            package='ros2_test_composition_components',
            namespace=namespace_val,
            plugin='tc::TimerComponent',
            name='timer_component'
        )

    # Thread Component
    info = '* Starting a Thread Component'
    actions.append(LogInfo(msg=TextSubstitution(text=info)))

    thread_component = ComposableNode(
        package='ros2_test_composition_components',
        namespace=namespace_val,
        plugin='tc::ThreadComponent',
        name='thread_component'
        )
    
    # Load components
    full_container_name = '/' + namespace_val + '/' + container_name
    load_composable_nodes = LoadComposableNodes(
        composable_node_descriptions=[timer_component, thread_component],
        target_container=full_container_name
    )
    actions.append(load_composable_nodes)

    return actions

def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup)
        ]
    )