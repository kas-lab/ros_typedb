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
from threading import Event
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
from std_msgs.msg import String


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

        assert configure_res.success is True and \
               get_inactive_state_res.current_state.id == 2 and \
               activate_res.success is True and \
               get_active_state_res.current_state.id == 3
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
        query_req.query_type = 'insert'
        query_req.query = \
            'insert $entity isa person, has email \"test@test.com\";'
        query_res = node.call_service(node.query_srv, query_req)
        assert query_res.success is True
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_ros_typedb_insert_event():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_ros_typedb()

        query_req = Query.Request()
        query_req.query_type = 'insert'
        query_req.query = \
            'insert $entity isa person, has email \"test@test.com\";'
        query_res = node.call_service(node.query_srv, query_req)
        event_flag = node.typedb_event.wait(timeout=5.0)
        assert event_flag and node.typedb_event_data == 'insert'
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_ros_typedb_delete_query():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_ros_typedb()

        query_req = Query.Request()
        query_req.query_type = 'insert'
        query_req.query = \
            'insert $entity isa person, has email \"test@test.com\";'
        query_res = node.call_service(node.query_srv, query_req)

        query_req = Query.Request()
        query_req.query_type = 'delete'
        query_req.query = \
            'match $entity isa person, has email \"test@test.com\";' + \
            ' delete $entity isa person;'
        query_res = node.call_service(node.query_srv, query_req)

        assert query_res.success is True
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_ros_typedb_delete_event():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_ros_typedb()

        query_req = Query.Request()
        query_req.query_type = 'insert'
        query_req.query = \
            'insert $entity isa person, has email \"test@test.com\";'
        query_res = node.call_service(node.query_srv, query_req)

        query_req = Query.Request()
        query_req.query_type = 'delete'
        query_req.query = \
            'match $entity isa person, has email \"test@test.com\";' + \
            ' delete $entity isa person;'
        query_res = node.call_service(node.query_srv, query_req)
        event_flag = node.typedb_event.wait(timeout=5.0)
        assert event_flag and node.typedb_event_data == 'delete'

        assert query_res.success is True
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_ros_typedb_wrong_query():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_ros_typedb()

        query_req = Query.Request()
        query_req.query_type = 'wrong'
        query_req.query = \
            'insert $entity isa person, has email \"test@test.com\";'
        query_res = node.call_service(node.query_srv, query_req)
        assert query_res.success is False
    finally:
        rclpy.shutdown()


class MakeTestNode(Node):

    def __init__(self, name='test_node'):
        super().__init__(name)
        self.typedb_event = Event()
        self.typedb_event_data = None

        self.change_state_srv = self.create_client(
            ChangeState, '/ros_typedb_interface/change_state')

        self.get_state_srv = self.create_client(
            GetState, '/ros_typedb_interface/get_state')

        self.query_srv = self.create_client(
            Query, '/typedb/query')

        self.event_sub = self.create_subscription(
            String, '/typedb/events', self.event_sub_cb, 10)

    def start_node(self):
        self.ros_spin_thread = Thread(
            target=lambda node: rclpy.spin(node),
            args=(self,))
        self.ros_spin_thread.start()

    def change_ros_typedb_state(self, transition_id):
        change_state_req = ChangeState.Request()
        change_state_req.transition.id = transition_id
        return self.call_service(self.change_state_srv, change_state_req)

    def get_ros_typedb_state(self):
        get_state_req = GetState.Request()
        return self.call_service(self.get_state_srv, get_state_req)

    def call_service(self, cli, request):
        if cli.wait_for_service(timeout_sec=5.0) is False:
            self.get_logger().error(
                'service not available {}'.format(cli.srv_name))
            return None
        future = cli.call_async(request)
        if self.executor.spin_until_future_complete(
                future, timeout_sec=5.0) is False:
            self.get_logger().error(
                "Future not completed {}".format(cli.srv_name))
            return None

        self.get_logger().error("Future completed {}".format(cli.srv_name))
        self.get_logger().error("Future result {}".format(future.result()))
        return future.result()

    def activate_ros_typedb(self):
        self.change_ros_typedb_state(1)
        self.change_ros_typedb_state(3)

    def event_sub_cb(self, msg):
        self.typedb_event_data = msg.data
        self.typedb_event.set()
