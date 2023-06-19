# Copyright 2023 Gustavo Rezende Silva
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

from pathlib import Path
import sys
from threading import Thread

import launch
import launch_pytest
import launch_ros

import pytest

import rclpy
from rclpy.node import Node

from ros_typedb.ros_typedb_interface import ROSTypeDBInterface

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.srv import GetState
from ros_typedb_msgs.srv import Query


@launch_pytest.fixture
def generate_test_description():
    path_to_test = Path(__file__).parents[1]

    ros_typedb_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[
            str(path_to_test / 'ros_typedb' / 'ros_typedb_interface.py')],
        additional_env={'PYTHONUNBUFFERED': '1'},
        name='ros_typedb_interface',
        output='screen',
    )

    return launch.LaunchDescription([
        # launch.actions.ExecuteProcess(
        #     cmd=['typedb server'],
        #     shell=True,
        #     cached_output=True,
        # ),
        ros_typedb_node,
    ])


@pytest.mark.launch(fixture=generate_test_description)
def test_ros_typedb_lc_states():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()

        configure_res = node.change_ros_typedb_state(1)
        get_inactive_state_res = node.get_ros_typedb_state()

        activate_res = node.change_ros_typedb_state(3)
        get_active_state_res = node.get_ros_typedb_state()

        assert configure_res is True and \
               get_inactive_state_res == 2 and \
               activate_res is True and \
               get_active_state_res == 3
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_ros_typedb_insert_query():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_ros_typedb()

        query_req = Query.Request()
        query_req.query = \
            'insert $entity isa person, has email \"test@test.com\";'
        while not node.insert_query_srv.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('service not available, waiting again...')
        query_res = node.insert_query_srv.call(query_req)
        assert query_res.success is True
    finally:
        rclpy.shutdown()


class MakeTestNode(Node):

    def __init__(self, name='test_node'):
        super().__init__(name)

        self.change_state_srv = self.create_client(
            ChangeState, '/ros_typedb_interface/change_state')

        self.get_state_srv = self.create_client(
            GetState, '/ros_typedb_interface/get_state')

        self.insert_query_srv = self.create_client(
            Query, '/typedb/insert_query')

    def start_node(self):
        self.ros_spin_thread = Thread(
            target=lambda node: rclpy.spin(node),
            args=(self,))
        self.ros_spin_thread.start()

    def change_ros_typedb_state(self, transition_id):
        change_state_req = ChangeState.Request()
        change_state_req.transition.id = transition_id
        while not self.change_state_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        change_state_res = self.change_state_srv.call(change_state_req)

        return change_state_res.success

    def get_ros_typedb_state(self):
        get_state_req = GetState.Request()
        while not self.get_state_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        get_state_res = self.get_state_srv.call(get_state_req)

        return get_state_res.current_state.id

    def activate_ros_typedb(self):
        self.change_ros_typedb_state(1)
        self.change_ros_typedb_state(3)
