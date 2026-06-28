"""Launch ros_typedb with the bundled test schema and data."""

# Copyright 2026 Gustavo Rezende Silva
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
import os

from ament_index_python.packages import get_package_share_directory

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
    """Launch ros_typedb with the bundled test schema and data."""
    package_share = get_package_share_directory('ros_typedb_examples')
    default_schema_path = os.path.join(
        package_share,
        'data',
        'typedb_test_data',
        'schema.tql',
    )
    default_data_path = os.path.join(
        package_share,
        'data',
        'typedb_test_data',
        'data.tql',
    )

    database_name = LaunchConfiguration('database_name')
    address = LaunchConfiguration('address')
    force_data = LaunchConfiguration('force_data')
    force_database = LaunchConfiguration('force_database')
    reload_schema = LaunchConfiguration('reload_schema')
    infer = LaunchConfiguration('infer')
    query_timeout_s = LaunchConfiguration('query_timeout_s')

    database_name_arg = DeclareLaunchArgument(
        'database_name',
        default_value='ros_typedb_test_data_example',
        description='database name',
    )

    address_arg = DeclareLaunchArgument(
        'address',
        default_value='localhost:1729',
        description='TypeDB server address',
    )

    force_data_arg = DeclareLaunchArgument(
        'force_data',
        default_value='True',
        description='reload test data on startup',
    )

    force_database_arg = DeclareLaunchArgument(
        'force_database',
        default_value='True',
        description='recreate the test database on startup',
    )

    reload_schema_arg = DeclareLaunchArgument(
        'reload_schema',
        default_value='True',
        description='reload test schema on startup',
    )

    infer_arg = DeclareLaunchArgument(
        'infer',
        default_value='True',
        description='use inference engine',
    )

    query_timeout_s_arg = DeclareLaunchArgument(
        'query_timeout_s',
        default_value='5.0',
        description='default TypeDB query timeout in seconds',
    )

    ros_typedb_node = LifecycleNode(
        package='ros_typedb',
        executable='ros_typedb',
        name='ros_typedb_interface',
        namespace='',
        output='screen',
        parameters=[{
            'schema_path': [default_schema_path],
            'data_path': [default_data_path],
            'database_name': database_name,
            'address': address,
            'force_data': force_data,
            'force_database': force_database,
            'reload_schema': reload_schema,
            'infer': infer,
            'query_timeout_s': query_timeout_s,
        }],
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(ros_typedb_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        ),
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=ros_typedb_node,
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(ros_typedb_node),
                    transition_id=(
                        lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
                    ),
                )),
            ],
        ),
    )

    return LaunchDescription([
        database_name_arg,
        address_arg,
        force_data_arg,
        force_database_arg,
        reload_schema_arg,
        infer_arg,
        query_timeout_s_arg,
        ros_typedb_node,
        configure_event,
        activate_event,
    ])
