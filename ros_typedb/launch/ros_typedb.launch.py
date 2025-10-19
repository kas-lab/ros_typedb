# Copyright 2024 Gustavo Rezende Silva
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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs


def generate_launch_description():
    schema_path = LaunchConfiguration('schema_path')
    data_path = LaunchConfiguration('data_path')
    database_name = LaunchConfiguration('database_name')
    address = LaunchConfiguration('address')
    force_data = LaunchConfiguration('force_data')
    force_database = LaunchConfiguration('force_database')
    infer = LaunchConfiguration('infer')

    schema_path_arg = DeclareLaunchArgument(
        'schema_path',
        default_value="['']",
        description='path for KB schema'
    )

    data_path_arg = DeclareLaunchArgument(
        'data_path',
        default_value="['']",
        description='path for KB data'
    )

    database_name_arg = DeclareLaunchArgument(
        'database_name',
        default_value='ros_typedb',
        description='database name'
    )

    address_arg = DeclareLaunchArgument(
        'address',
        default_value='localhost:1729',
        description='typedb server address'
    )

    force_data_arg = DeclareLaunchArgument(
        'force_data',
        default_value='False',
        description='force data'
    )

    force_database_arg = DeclareLaunchArgument(
        'force_database',
        default_value='False',
        description='force database'
    )

    infer_arg = DeclareLaunchArgument(
        'infer',
        default_value='True',
        description='use inference engine'
    )

    ros_typedb_node = LifecycleNode(
        package='ros_typedb',
        executable='ros_typedb',
        name='ros_typedb',
        namespace='',
        output='screen',
        parameters=[{
            'schema_path': schema_path,
            'data_path': data_path,
            'database_name': database_name,
            'address': address,
            'force_data': force_data,
            'force_database': force_database,
            'infer': infer,
        }]
    )

    ros_typedb_node_config_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(ros_typedb_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    ros_typedb_node_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=ros_typedb_node,
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(ros_typedb_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    return LaunchDescription([
        schema_path_arg,
        data_path_arg,
        database_name_arg,
        address_arg,
        force_data_arg,
        force_database_arg,
        infer_arg,
        ros_typedb_node,
        ros_typedb_node_config_event,
        ros_typedb_node_activate_event
    ])
