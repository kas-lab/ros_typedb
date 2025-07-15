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

import launch
import launch_pytest
import launch_ros

from pathlib import Path

import pytest
import rclpy

from threading import Event
from threading import Thread

import sys

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.srv import GetState
from rclpy.node import Node
from ros_typedb_msgs.msg import Attribute
from ros_typedb_msgs.msg import QueryResult
from ros_typedb_msgs.msg import ResultTree
from ros_typedb_msgs.msg import Thing
from ros_typedb_msgs.srv import Query
from std_msgs.msg import String


@pytest.fixture
def insert_query():
    query_req = Query.Request()
    query_req.query_type = 'insert'
    query_req.query = """
        insert
            $person isa person,
                has email "test@test.com",
                has nickname "test",
                has age 33,
                has height 1.00,
                has alive true,
                has birth-date 1990-06-01;
            $person2 isa person,
                has email "test2@test.com",
                has nickname "employer";
            $person3 isa person,
                has email "test3@test.com",
                has nickname "employer2";
            (employee:$person, employer:$person2) isa employment,
                has salary 1000;
            (employee:$person, employer:$person3) isa employment,
                has salary 500;
    """
    return query_req


@launch_pytest.fixture
def generate_test_description():
    path_to_test = Path(__file__).parents[1]
    path_tql = path_to_test / 'test' / 'typedb_test_data'

    ros_typedb_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[
            str(path_to_test / 'ros_typedb' / 'ros_typedb_node.py')],
        additional_env={'PYTHONUNBUFFERED': '1'},
        name='ros_typedb',
        output='screen',
        parameters=[{
            'schema_path': [str(path_tql / 'schema.tql')],
            'data_path': [str(path_tql / 'data.tql')]
        }]
    )

    return launch.LaunchDescription([
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
def test_ros_typedb_insert_query(insert_query):
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_ros_typedb()

        query_res = node.call_service(node.query_srv, insert_query)
        assert query_res.success is True
    finally:
        rclpy.shutdown()


@pytest.mark.skip(
    reason='Events deactivated')
@pytest.mark.launch(fixture=generate_test_description)
def test_ros_typedb_insert_event(insert_query):
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_ros_typedb()

        node.call_service(node.query_srv, insert_query)
        event_flag = node.typedb_event.wait(timeout=5.0)
        assert event_flag and node.typedb_event_data == 'insert'
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_ros_typedb_delete_query(insert_query):
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_ros_typedb()

        node.call_service(node.query_srv, insert_query)

        query_req = Query.Request()
        query_req.query_type = 'delete'
        query_req.query = """
            match $entity isa person, has email "test@test.com";
            delete $entity isa person;
        """
        query_res = node.call_service(node.query_srv, query_req)

        match_query_req = Query.Request()
        match_query_req.query_type = 'get_aggregate'
        match_query_req.query = """
            match $entity isa person, has email "test@test.com";
            get $entity;
            count;
        """
        match_query_res = node.call_service(node.query_srv, match_query_req)

        assert query_res.success and match_query_res.success and \
            match_query_res.results[0].attributes[0].value.type == 2 and \
            match_query_res.results[0].attributes[0].value.integer_value == 0
    finally:
        rclpy.shutdown()


@pytest.mark.skip(
    reason='Events deactivated')
@pytest.mark.launch(fixture=generate_test_description)
def test_ros_typedb_delete_event(insert_query):
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_ros_typedb()

        node.call_service(node.query_srv, insert_query)

        query_req = Query.Request()
        query_req.query_type = 'delete'
        query_req.query = """
            match $entity isa person, has email "test@test.com";
            delete $entity isa person;
        """
        query_res = node.call_service(node.query_srv, query_req)
        event_flag = node.typedb_event.wait(timeout=5.0)
        assert event_flag and node.typedb_event_data == 'delete'

        assert query_res.success is True
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_ros_typedb_wrong_query(insert_query):
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_ros_typedb()

        insert_query_req = insert_query
        insert_query_req.query_type = 'wrong'
        query_res = node.call_service(node.query_srv, insert_query_req)
        assert query_res.success is False
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_ros_typedb_fetch_query_attribute(insert_query):
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_ros_typedb()

        node.call_service(node.query_srv, insert_query)

        # query_req = Query.Request()
        # query_req.query_type = 'fetch'
        # query_req.query = """
        #     match $entity isa person,
        #         has email "test@test.com",
        #         has nickname $nick,
        #         has age $age,
        #         has height $height,
        #         has alive $alive,
        #         has birth-date $date;
        #     fetch $nick; $age; $height; $alive; $date;
        # """
        # query_res = node.call_service(node.query_srv, query_req)

        # correct_nick = False
        # correct_age = False
        # correct_height = False
        # correct_alive = False
        # correct_date = False
        # for result in query_res.results:
        #     for r in result.attributes:
        #         if r.name == 'nick' and r.label == 'nickname' and \
        #            r.value.string_value == 'test':
        #             correct_nick = True
        #         if r.name == 'age' and r.label == 'age' and \
        #            r.value.integer_value == 33:
        #             correct_age = True
        #         if r.name == 'height' and r.label == 'height' and \
        #            r.value.double_value == 1.0:
        #             correct_height = True
        #         if r.name == 'alive' and r.label == 'alive' and \
        #            r.value.bool_value is True:
        #             correct_alive = True
        #         if r.name == 'date' and r.label == 'birth-date' and \
        #            r.value.string_value == '1990-06-01T00:00:00.000':
        #             correct_date = True

        # assert query_res.success is True and correct_nick and correct_age and \
        #     correct_height and correct_alive and correct_date
        
        query_req = Query.Request()
        query_req.query_type = 'fetch'
        query_req.query = """
            match
                $company_var isa company, has name "TU Delft";
            fetch
                $company_var: name, address;
                employee_names: {
                    match
                        $employment_var (employer: $company_var, employee: $employee_var) isa employment;
                    fetch
                        $employee_var: full-name, email;
                        $employment_var: salary;
                };
        """
        expected_response = Query.Response()

        expected_query_result = QueryResult()
        
        company_var_tree = ResultTree()
        
        company_var_node = Thing()
        company_var_node.thing_type = Thing.ENTITY
        company_var_node.thing_id = 0
        company_var_node.variable_name = 'company_var'
        company_var_node.type_name = 'company'

        address_attr = Attribute()
        address_attr.label = 'address'
        address_attr.value.type = 3
        address_attr.value.string_value = 'street 1'
        company_var_node.attributes.append(address_attr)

        address_attr2 = Attribute()
        address_attr2.label = 'address'
        address_attr2.value.type = 3
        address_attr2.value.string_value = 'street 2'
        company_var_node.attributes.append(address_attr2)

        name_attr = Attribute()
        name_attr.label = 'name'
        name_attr.value.type = 3
        name_attr.value.string_value = 'TU Delft'
        company_var_node.attributes.append(name_attr)

        company_var_tree.things.append(company_var_node)
        expected_query_result.nodes.append(company_var_tree)

        employee_names_tree1 = ResultTree()
        employee_names_tree1.sub_query_name = 'employee_names'

        employee_var_node = Thing()
        employee_var_node.thing_type = Thing.ENTITY
        employee_var_node.thing_id = 1
        employee_var_node.variable_name = 'employee_var'
        employee_var_node.type_name = 'person'

        email_attr = Attribute()
        email_attr.label = 'email'
        email_attr.value.type = 3
        email_attr.value.string_value = 'phd@tudelft.nl'
        employee_var_node.attributes.append(email_attr)

        full_name_attr = Attribute()
        full_name_attr.label = 'full-name'
        full_name_attr.value.type = 3
        full_name_attr.value.string_value = 'PhD candidate 1'
        employee_var_node.attributes.append(full_name_attr)
        
        employee_names_tree1.things.append(employee_var_node)
        
        employment_var_node = Thing()
        employment_var_node.thing_type = Thing.RELATION
        employment_var_node.thing_id = 2
        employment_var_node.variable_name = 'employment_var'
        employment_var_node.type_name = 'employment'

        salary_attr = Attribute()
        salary_attr.label = 'salary'
        salary_attr.value.type = 1
        salary_attr.value.integer_value = 30000
        employment_var_node.attributes.append(salary_attr)

        employee_names_tree1.things.append(employment_var_node)
        expected_query_result.sub_query_nodes.append(employee_names_tree1)

        employee_names_tree2 = ResultTree()
        employee_names_tree2.sub_query_name = 'employee_names'

        employee_var_node = Thing()
        employee_var_node.thing_type = Thing.ENTITY
        employee_var_node.thing_id = 2
        employee_var_node.variable_name = 'employee_var'
        employee_var_node.type_name = 'person'

        email_attr = Attribute()
        email_attr.label = 'email'
        email_attr.value.type = 3
        email_attr.value.string_value = 'boss@tudelft.nl'
        employee_var_node.attributes.append(email_attr)

        full_name_attr = Attribute()
        full_name_attr.label = 'full-name'
        full_name_attr.value.type = 3
        full_name_attr.value.string_value = 'Big Boss'
        employee_var_node.attributes.append(full_name_attr)
        
        employee_names_tree2.things.append(employee_var_node)
        
        employment_var_node = Thing()
        employment_var_node.thing_type = Thing.RELATION
        employment_var_node.thing_id = 3
        employment_var_node.variable_name = 'employment_var'
        employment_var_node.type_name = 'employment'

        salary_attr = Attribute()
        salary_attr.label = 'salary'
        salary_attr.value.type = 1
        salary_attr.value.integer_value = 999999
        employment_var_node.attributes.append(salary_attr)

        employee_names_tree2.things.append(employment_var_node)
        expected_query_result.sub_query_nodes.append(employee_names_tree2)

        employee_names_tree3 = ResultTree()
        employee_names_tree3.sub_query_name = 'employee_names'

        employee_var_node = Thing()
        employee_var_node.thing_type = Thing.ENTITY
        employee_var_node.thing_id = 4
        employee_var_node.variable_name = 'employee_var'
        employee_var_node.type_name = 'person'

        email_attr = Attribute()
        email_attr.label = 'email'
        email_attr.value.type = 3
        email_attr.value.string_value = 'guest@tudelft.nl'
        employee_var_node.attributes.append(email_attr)

        full_name_attr = Attribute()
        full_name_attr.label = 'full-name'
        full_name_attr.value.type = 3
        full_name_attr.value.string_value = 'Random guest'
        employee_var_node.attributes.append(full_name_attr)
        
        employee_names_tree3.things.append(employee_var_node)
        
        employment_var_node = Thing()
        employment_var_node.thing_type = Thing.RELATION
        employment_var_node.thing_id = 5
        employment_var_node.variable_name = 'employment_var'
        employment_var_node.type_name = 'employment'

        salary_attr = Attribute()
        salary_attr.label = 'salary'
        salary_attr.value.type = 1
        salary_attr.value.integer_value = 0
        employment_var_node.attributes.append(salary_attr)

        employee_names_tree3.things.append(employment_var_node)
        expected_query_result.sub_query_nodes.append(employee_names_tree3)

        expected_response.results.append(expected_query_result)
        expected_response.success = True

        query_res = node.call_service(node.query_srv, query_req)
        assert query_res == expected_response
        # assert query_res.success
        # assert len(query_res.results) == 1
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_ros_typedb_get_query(insert_query):
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_ros_typedb()

        node.call_service(node.query_srv, insert_query)

        query_req = Query.Request()
        query_req.query_type = 'get'
        query_req.query = """
            match
                $p isa person, has full-name $name, has email $email;
            get $name, $email;
            sort $name asc; limit 3;
        """
        query_res = node.call_service(node.query_srv, query_req)

        correct_name = False
        correct_email = False
        for r in query_res.results[0].attributes:
            if r.name == 'name' and r.label == 'full-name' \
               and r.value.string_value == 'Ahmed Frazier':
                correct_name = True
            if r.name == 'email' and r.label == 'email' and \
               r.value.string_value == 'ahmed.frazier@gmail.com':
                correct_email = True

        assert query_res.success is True and \
            len(query_res.results) == 3 and correct_name and correct_email
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_ros_typedb_get_aggregate_query(insert_query):
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_ros_typedb()

        node.call_service(node.query_srv, insert_query)

        query_req = Query.Request()
        query_req.query_type = 'get_aggregate'
        query_req.query = """
            match
                $person isa person,
                    has email "test@test.com";
                (employee:$person) isa employment, has salary $s;
            get $s; sum $s;
        """
        query_res = node.call_service(node.query_srv, query_req)

        assert query_res.success is True and \
            query_res.results[0].attributes[0].value.integer_value == 1500
    finally:
        rclpy.shutdown()


class MakeTestNode(Node):

    def __init__(self, name='test_node'):
        super().__init__(name)
        self.typedb_event = Event()
        self.typedb_event_data = None

        self.change_state_srv = self.create_client(
            ChangeState, '/ros_typedb/change_state')

        self.get_state_srv = self.create_client(
            GetState, '/ros_typedb/get_state')

        self.query_srv = self.create_client(
            Query, '/ros_typedb/query')

        self.event_sub = self.create_subscription(
            String, '/ros_typedb/events', self.event_sub_cb, 10)

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
                'Future not completed {}'.format(cli.srv_name))
            return None

        return future.result()

    def activate_ros_typedb(self):
        self.change_ros_typedb_state(1)
        self.change_ros_typedb_state(3)

    def event_sub_cb(self, msg):
        self.typedb_event_data = msg.data
        self.typedb_event.set()
