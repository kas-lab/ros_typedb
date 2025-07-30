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

import sys

from threading import Event
from threading import Thread

from ros_typedb.ros_typedb_interface import convert_attribute_dict_to_ros_msg
from ros_typedb.ros_typedb_interface import fetch_result_to_ros_result_tree

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.srv import GetState
from rclpy.node import Node

from rcl_interfaces.msg import ParameterType

from ros_typedb_msgs.msg import Attribute
from ros_typedb_msgs.msg import IndexList
from ros_typedb_msgs.msg import QueryResult
from ros_typedb_msgs.msg import ResultTree
from ros_typedb_msgs.msg import Thing
from ros_typedb_msgs.srv import Query

from std_msgs.msg import String

@pytest.fixture
def insert_query():
    query_req = Query.Request()
    query_req.query_type = query_req.INSERT
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
        query_req.query_type = query_req.DELETE
        query_req.query = """
            match $entity isa person, has email "test@test.com";
            delete $entity isa person;
        """
        query_res = node.call_service(node.query_srv, query_req)

        match_query_req = Query.Request()
        match_query_req.query_type = match_query_req.GET_AGGREGATE
        match_query_req.query = """
            match $entity isa person, has email "test@test.com";
            get $entity;
            count;
        """
        match_query_res = node.call_service(node.query_srv, match_query_req)

        assert query_res.success
        assert match_query_res.success
        assert match_query_res.results[0].results[0].attribute.value.type == ParameterType.PARAMETER_INTEGER
        assert match_query_res.results[0].results[0].attribute.value.integer_value == 0
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
        query_req.query_type = query_req.DELETE
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
        insert_query_req.query_type = 100
        query_res = node.call_service(node.query_srv, insert_query_req)
        assert query_res.success is False
    finally:
        rclpy.shutdown()

def test_convert_attribute_dict_to_ros_msg():
    expected_address_attr = Attribute()
    expected_address_attr.variable_name = 'company_var'
    expected_address_attr.label = 'address'
    expected_address_attr.value.type = ParameterType.PARAMETER_STRING_ARRAY
    expected_address_attr.value.string_array_value = ['street 1', 'street 2']

    expected_name_attr = Attribute()
    expected_name_attr.variable_name = 'company_var'
    expected_name_attr.label = 'name'
    expected_name_attr.value.type = ParameterType.PARAMETER_STRING_ARRAY
    expected_name_attr.value.string_array_value = ['TU Delft']

    test_dict = {
        "company_var": {
            "address": [
                { "value": "street 1", "type": { "label": "address", "root": "attribute", "value_type": "string" } },
                { "value": "street 2", "type": { "label": "address", "root": "attribute", "value_type": "string" } }
            ],
            "name": [ { "value": "TU Delft", "type": { "label": "name", "root": "attribute", "value_type": "string" } } ],
            "type": { "label": "company", "root": "entity" }
        }
    }
    address_attr = convert_attribute_dict_to_ros_msg('company_var', test_dict['company_var']['address'])
    assert expected_address_attr == address_attr

    name_attr = convert_attribute_dict_to_ros_msg('company_var', test_dict['company_var']['name'])
    assert expected_name_attr == name_attr 

    test_dict = {
        "age": { "value": 33, "type": { "label": "age", "root": "attribute", "value_type": "long" } },
        "alive": { "value": True, "type": { "label": "alive", "root": "attribute", "value_type": "boolean" } },
        "date": { "value": "1990-06-01T00:00:00.000", "type": { "label": "birth-date", "root": "attribute", "value_type": "datetime" } },
        "height": { "value": 1, "type": { "label": "height", "root": "attribute", "value_type": "double" } },
        "nick": { "value": "test", "type": { "label": "nickname", "root": "attribute", "value_type": "string" } }
    }
    expected_age_attr = Attribute()
    expected_age_attr.variable_name = 'age'
    expected_age_attr.label = 'age'
    expected_age_attr.value.type = ParameterType.PARAMETER_INTEGER
    expected_age_attr.value.integer_value = 33

    age_attr = convert_attribute_dict_to_ros_msg('age', test_dict['age'])
    assert expected_age_attr == age_attr

    expected_alive_attr = Attribute()
    expected_alive_attr.variable_name = 'alive'
    expected_alive_attr.label = 'alive'
    expected_alive_attr.value.type = ParameterType.PARAMETER_BOOL
    expected_alive_attr.value.bool_value = True

    alive_attr = convert_attribute_dict_to_ros_msg('alive', test_dict['alive'])
    assert expected_alive_attr == alive_attr

    expected_date_attr = Attribute()
    expected_date_attr.variable_name = 'date'
    expected_date_attr.label = 'birth-date'
    expected_date_attr.value.type = ParameterType.PARAMETER_STRING
    expected_date_attr.value.string_value = '1990-06-01T00:00:00.000'

    date_attr = convert_attribute_dict_to_ros_msg('date', test_dict['date'])
    assert expected_date_attr == date_attr

    expected_height_attr = Attribute()
    expected_height_attr.variable_name = 'height'
    expected_height_attr.label = 'height'
    expected_height_attr.value.type = ParameterType.PARAMETER_DOUBLE
    expected_height_attr.value.double_value = 1.0

    height_attr = convert_attribute_dict_to_ros_msg('height', test_dict['height'])
    assert expected_height_attr == height_attr

    expected_nick_attr = Attribute()
    expected_nick_attr.variable_name = 'nick'
    expected_nick_attr.label = 'nickname'
    expected_nick_attr.value.type = ParameterType.PARAMETER_STRING
    expected_nick_attr.value.string_value = 'test'

    nick_attr = convert_attribute_dict_to_ros_msg('nick', test_dict['nick'])
    assert expected_nick_attr == nick_attr

def test_fetch_result_to_ros_result_tree():
    json_test = {
        "age": { "value": 33, "type": { "label": "age", "root": "attribute", "value_type": "long" } },
        "alive": { "value": True, "type": { "label": "alive", "root": "attribute", "value_type": "boolean" } },
        "date": { "value": "1990-06-01T00:00:00.000", "type": { "label": "birth-date", "root": "attribute", "value_type": "datetime" } },
        "height": { "value": 1, "type": { "label": "height", "root": "attribute", "value_type": "double" } },
        "nick": { "value": "test", "type": { "label": "nickname", "root": "attribute", "value_type": "string" } }
    }

    expected_age_attr = Attribute()
    expected_age_attr.variable_name = 'age'
    expected_age_attr.label = 'age'
    expected_age_attr.value.type = ParameterType.PARAMETER_INTEGER
    expected_age_attr.value.integer_value = 33

    expected_alive_attr = Attribute()
    expected_alive_attr.variable_name = 'alive'
    expected_alive_attr.label = 'alive'
    expected_alive_attr.value.type = ParameterType.PARAMETER_BOOL
    expected_alive_attr.value.bool_value = True

    expected_date_attr = Attribute()
    expected_date_attr.variable_name = 'date'
    expected_date_attr.label = 'birth-date'
    expected_date_attr.value.type = ParameterType.PARAMETER_STRING
    expected_date_attr.value.string_value = '1990-06-01T00:00:00.000'

    expected_height_attr = Attribute()
    expected_height_attr.variable_name = 'height'
    expected_height_attr.label = 'height'
    expected_height_attr.value.type = ParameterType.PARAMETER_DOUBLE
    expected_height_attr.value.double_value = 1.0

    expected_nick_attr = Attribute()
    expected_nick_attr.variable_name = 'nick'
    expected_nick_attr.label = 'nickname'
    expected_nick_attr.value.type = ParameterType.PARAMETER_STRING
    expected_nick_attr.value.string_value = 'test'

    query_result_age = QueryResult()
    query_result_age.type = QueryResult.ATTRIBUTE
    query_result_age.result_index = 0
    query_result_age.attribute = expected_age_attr

    query_result_alive = QueryResult()
    query_result_alive.type = QueryResult.ATTRIBUTE
    query_result_alive.result_index = 1
    query_result_alive.attribute = expected_alive_attr

    query_result_date = QueryResult()
    query_result_date.type = QueryResult.ATTRIBUTE
    query_result_date.result_index = 2
    query_result_date.attribute = expected_date_attr

    query_result_height = QueryResult()
    query_result_height.type = QueryResult.ATTRIBUTE
    query_result_height.result_index = 3
    query_result_height.attribute = expected_height_attr

    query_result_nick = QueryResult()
    query_result_nick.type = QueryResult.ATTRIBUTE
    query_result_nick.result_index = 4
    query_result_nick.attribute = expected_nick_attr

    expected_tree = ResultTree()
    expected_tree.results.append(query_result_age)
    expected_tree.results.append(query_result_alive)
    expected_tree.results.append(query_result_date)
    expected_tree.results.append(query_result_height)
    expected_tree.results.append(query_result_nick)

    result_tree, tree_index = fetch_result_to_ros_result_tree(json_test)
    assert len(result_tree.results) == 5
    assert tree_index == 5
    assert len(expected_tree.results) == len(result_tree.results)
    
    assert result_tree.results[0] == query_result_age
    assert result_tree.results[1] == query_result_alive
    assert result_tree.results[2] == query_result_date
    assert result_tree.results[3] == query_result_height
    assert result_tree.results[4] == query_result_nick
    
    assert expected_tree == result_tree

    json_test = {
        "company_var": {
            "address": [
                { "value": "street 1", "type": { "label": "address", "root": "attribute", "value_type": "string" } },
                { "value": "street 2", "type": { "label": "address", "root": "attribute", "value_type": "string" } }
            ],
            "name": [ { "value": "TU Delft", "type": { "label": "name", "root": "attribute", "value_type": "string" } } ],
            "type": { "label": "company", "root": "entity" }
        },
        "employee_names": [
            {
                "employee_var": {
                    "email": [ { "value": "phd@tudelft.nl", "type": { "label": "email", "root": "attribute", "value_type": "string" } } ],
                    "full-name": [ { "value": "PhD candidate 1", "type": { "label": "full-name", "root": "attribute", "value_type": "string" } } ],
                    "type": { "label": "person", "root": "entity" }
                },
                "employment_var": {
                    "salary": [ { "value": 30000, "type": { "label": "salary", "root": "attribute", "value_type": "long" } } ],
                    "type": { "label": "employment", "root": "relation" }
                }
            },
            {
                "employee_var": {
                    "email": [ { "value": "boss@tudelft.nl", "type": { "label": "email", "root": "attribute", "value_type": "string" } } ],
                    "full-name": [ { "value": "Big Boss", "type": { "label": "full-name", "root": "attribute", "value_type": "string" } } ],
                    "type": { "label": "person", "root": "entity" }
                },
                "employment_var": {
                    "salary": [ { "value": 999999, "type": { "label": "salary", "root": "attribute", "value_type": "long" } } ],
                    "type": { "label": "employment", "root": "relation" }
                }
            },
            {
                "employee_var": {
                    "email": [ { "value": "guest@tudelft.nl", "type": { "label": "email", "root": "attribute", "value_type": "string" } } ],
                    "full-name": [ { "value": "Random guest", "type": { "label": "full-name", "root": "attribute", "value_type": "string" } } ],
                    "type": { "label": "person", "root": "entity" }
                },
                "employment_var": {
                    "salary": [ { "value": 0, "type": { "label": "salary", "root": "attribute", "value_type": "long" } } ],
                    "type": { "label": "employment", "root": "relation" }
                }
            }
        ]
    }
    expected_tree = ResultTree()
        
    company_var_thing = Thing()
    company_var_thing.type = Thing.ENTITY
    company_var_thing.variable_name = 'company_var'
    company_var_thing.type_name = 'company'

    address_attr = Attribute()
    address_attr.label = 'address'
    address_attr.variable_name = 'address'
    address_attr.value.type = ParameterType.PARAMETER_STRING_ARRAY
    address_attr.value.string_array_value = ['street 1', 'street 2']
    company_var_thing.attributes.append(address_attr)

    name_attr = Attribute()
    name_attr.label = 'name'
    name_attr.variable_name = 'name'
    name_attr.value.type = ParameterType.PARAMETER_STRING_ARRAY
    name_attr.value.string_array_value = ['TU Delft']
    company_var_thing.attributes.append(name_attr)

    company_var_query_result = QueryResult()
    company_var_query_result.type = QueryResult.THING
    company_var_query_result.result_index = 0
    company_var_query_result.thing = company_var_thing

    employee_names_subquery = QueryResult()
    employee_names_subquery.type = QueryResult.SUB_QUERY
    employee_names_subquery.result_index = 1
    employee_names_subquery.sub_query_name = 'employee_names'
    
    employee_var_thing1 = Thing()
    employee_var_thing1.type = Thing.ENTITY
    employee_var_thing1.variable_name = 'employee_var'
    employee_var_thing1.type_name = 'person'

    email_attr = Attribute()
    email_attr.label = 'email'
    email_attr.variable_name = 'email'
    email_attr.value.type = ParameterType.PARAMETER_STRING_ARRAY
    email_attr.value.string_array_value = ['phd@tudelft.nl']
    employee_var_thing1.attributes.append(email_attr)

    full_name_attr = Attribute()
    full_name_attr.label = 'full-name'
    full_name_attr.variable_name = 'full-name'
    full_name_attr.value.type = ParameterType.PARAMETER_STRING_ARRAY
    full_name_attr.value.string_array_value = ['PhD candidate 1']
    employee_var_thing1.attributes.append(full_name_attr)
    
    employee_var_thing1_result = QueryResult()
    employee_var_thing1_result.type = QueryResult.THING
    employee_var_thing1_result.result_index = 2
    employee_var_thing1_result.thing = employee_var_thing1
    
    employment_var_thing1 = Thing()
    employment_var_thing1.type = Thing.RELATION
    employment_var_thing1.variable_name = 'employment_var'
    employment_var_thing1.type_name = 'employment'

    salary_attr = Attribute()
    salary_attr.label = 'salary'
    salary_attr.variable_name = 'salary'
    salary_attr.value.type = ParameterType.PARAMETER_INTEGER_ARRAY
    salary_attr.value.integer_array_value = [30000]
    employment_var_thing1.attributes.append(salary_attr)

    employment_var_thing1_result = QueryResult()
    employment_var_thing1_result.type = QueryResult.THING
    employment_var_thing1_result.result_index = 3
    employment_var_thing1_result.thing = employment_var_thing1
    
    list_index = IndexList()
    list_index.index.append(2)
    list_index.index.append(3)
    employee_names_subquery.children_index.append(list_index)
    
    ## Subtree 2
    employee_var_thing2 = Thing()
    employee_var_thing2.type = Thing.ENTITY
    employee_var_thing2.variable_name = 'employee_var'
    employee_var_thing2.type_name = 'person'

    email_attr = Attribute()
    email_attr.label = 'email'
    email_attr.variable_name = 'email'
    email_attr.value.type = ParameterType.PARAMETER_STRING_ARRAY
    email_attr.value.string_array_value = ['boss@tudelft.nl']
    employee_var_thing2.attributes.append(email_attr)

    full_name_attr = Attribute()
    full_name_attr.label = 'full-name'
    full_name_attr.variable_name = 'full-name'
    full_name_attr.value.type = ParameterType.PARAMETER_STRING_ARRAY
    full_name_attr.value.string_array_value = ['Big Boss']
    employee_var_thing2.attributes.append(full_name_attr)
    
    employee_var_thing2_result = QueryResult()
    employee_var_thing2_result.type = QueryResult.THING
    employee_var_thing2_result.result_index = 4
    employee_var_thing2_result.thing = employee_var_thing2

    employment_var_thing2 = Thing()
    employment_var_thing2.type = Thing.RELATION
    employment_var_thing2.variable_name = 'employment_var'
    employment_var_thing2.type_name = 'employment'

    salary_attr = Attribute()
    salary_attr.label = 'salary'
    salary_attr.variable_name = 'salary'
    salary_attr.value.type = ParameterType.PARAMETER_INTEGER_ARRAY
    salary_attr.value.integer_array_value = [999999]
    employment_var_thing2.attributes.append(salary_attr)

    employment_var_thing2_result = QueryResult()
    employment_var_thing2_result.type = QueryResult.THING
    employment_var_thing2_result.result_index = 5
    employment_var_thing2_result.thing = employment_var_thing2

    list_index = IndexList()
    list_index.index.append(4)
    list_index.index.append(5)
    employee_names_subquery.children_index.append(list_index)

    ## Subtree 3
    employee_var_thing3 = Thing()
    employee_var_thing3.type = Thing.ENTITY
    employee_var_thing3.variable_name = 'employee_var'
    employee_var_thing3.type_name = 'person'

    email_attr = Attribute()
    email_attr.label = 'email'
    email_attr.variable_name = 'email'
    email_attr.value.type = ParameterType.PARAMETER_STRING_ARRAY
    email_attr.value.string_array_value = ['guest@tudelft.nl']
    employee_var_thing3.attributes.append(email_attr)

    full_name_attr = Attribute()
    full_name_attr.label = 'full-name'
    full_name_attr.variable_name = 'full-name'
    full_name_attr.value.type = ParameterType.PARAMETER_STRING_ARRAY
    full_name_attr.value.string_array_value = ['Random guest']
    employee_var_thing3.attributes.append(full_name_attr)
    
    employee_var_thing3_result = QueryResult()
    employee_var_thing3_result.type = QueryResult.THING
    employee_var_thing3_result.result_index = 6
    employee_var_thing3_result.thing = employee_var_thing3
    
    employment_var_thing3 = Thing()
    employment_var_thing3.type = Thing.RELATION
    employment_var_thing3.variable_name = 'employment_var'
    employment_var_thing3.type_name = 'employment'

    salary_attr = Attribute()
    salary_attr.label = 'salary'
    salary_attr.variable_name = 'salary'
    salary_attr.value.type = ParameterType.PARAMETER_INTEGER_ARRAY
    salary_attr.value.integer_array_value = [0]
    employment_var_thing3.attributes.append(salary_attr)

    employment_var_thing3_result = QueryResult()
    employment_var_thing3_result.type = QueryResult.THING
    employment_var_thing3_result.result_index = 7
    employment_var_thing3_result.thing = employment_var_thing3

    list_index = IndexList()
    list_index.index.append(6)
    list_index.index.append(7)
    employee_names_subquery.children_index.append(list_index)

    expected_tree.results.append(company_var_query_result)
    
    expected_tree.results.append(employee_names_subquery)
    
    expected_tree.results.append(employee_var_thing1_result)
    expected_tree.results.append(employment_var_thing1_result)

    expected_tree.results.append(employee_var_thing2_result)
    expected_tree.results.append(employment_var_thing2_result)

    expected_tree.results.append(employee_var_thing3_result)
    expected_tree.results.append(employment_var_thing3_result)

    result_tree, _ = fetch_result_to_ros_result_tree(json_test)
    assert len(expected_tree.results) == len(result_tree.results)
    
    assert result_tree.results[0] == company_var_query_result
    assert result_tree.results[1] == employee_names_subquery
    assert result_tree.results[2] == employee_var_thing1_result
    assert result_tree.results[3] == employment_var_thing1_result
    assert result_tree.results[4] == employee_var_thing2_result
    assert result_tree.results[5] == employment_var_thing2_result
    assert result_tree.results[6] == employee_var_thing3_result
    assert result_tree.results[7] == employment_var_thing3_result
    assert expected_tree == result_tree

@pytest.mark.launch(fixture=generate_test_description)
def test_ros_typedb_fetch_query_attribute(insert_query):
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_ros_typedb()

        node.call_service(node.query_srv, insert_query)

        query_req = Query.Request()
        query_req.query_type = query_req.FETCH
        query_req.query = """
            match $entity isa person,
                has email "test@test.com",
                has nickname $nick,
                has age $age,
                has height $height,
                has alive $alive,
                has birth-date $date;
            fetch $age; $alive; $date; $height; $nick;
        """
        query_res = node.call_service(node.query_srv, query_req)

        expected_age_attr = Attribute()
        expected_age_attr.variable_name = 'age'
        expected_age_attr.label = 'age'
        expected_age_attr.value.type = ParameterType.PARAMETER_INTEGER
        expected_age_attr.value.integer_value = 33

        expected_alive_attr = Attribute()
        expected_alive_attr.variable_name = 'alive'
        expected_alive_attr.label = 'alive'
        expected_alive_attr.value.type = ParameterType.PARAMETER_BOOL
        expected_alive_attr.value.bool_value = True

        expected_date_attr = Attribute()
        expected_date_attr.variable_name = 'date'
        expected_date_attr.label = 'birth-date'
        expected_date_attr.value.type = ParameterType.PARAMETER_STRING
        expected_date_attr.value.string_value = '1990-06-01T00:00:00.000'

        expected_height_attr = Attribute()
        expected_height_attr.variable_name = 'height'
        expected_height_attr.label = 'height'
        expected_height_attr.value.type = ParameterType.PARAMETER_DOUBLE
        expected_height_attr.value.double_value = 1.0

        expected_nick_attr = Attribute()
        expected_nick_attr.variable_name = 'nick'
        expected_nick_attr.label = 'nickname'
        expected_nick_attr.value.type = ParameterType.PARAMETER_STRING
        expected_nick_attr.value.string_value = 'test'

        query_result_age = QueryResult()
        query_result_age.type = QueryResult.ATTRIBUTE
        query_result_age.result_index = 0
        query_result_age.attribute = expected_age_attr

        query_result_alive = QueryResult()
        query_result_alive.type = QueryResult.ATTRIBUTE
        query_result_alive.result_index = 1
        query_result_alive.attribute = expected_alive_attr

        query_result_date = QueryResult()
        query_result_date.type = QueryResult.ATTRIBUTE
        query_result_date.result_index = 2
        query_result_date.attribute = expected_date_attr

        query_result_height = QueryResult()
        query_result_height.type = QueryResult.ATTRIBUTE
        query_result_height.result_index = 3
        query_result_height.attribute = expected_height_attr

        query_result_nick = QueryResult()
        query_result_nick.type = QueryResult.ATTRIBUTE
        query_result_nick.result_index = 4
        query_result_nick.attribute = expected_nick_attr

        expected_tree = ResultTree()
        expected_tree.results.append(query_result_age)
        expected_tree.results.append(query_result_alive)
        expected_tree.results.append(query_result_date)
        expected_tree.results.append(query_result_height)
        expected_tree.results.append(query_result_nick)

        assert query_res.success is True
        assert len(query_res.results) == 1
        
        assert query_result_age in query_res.results[0].results 
        assert query_result_alive in query_res.results[0].results 
        assert query_result_date in query_res.results[0].results 
        assert query_result_height in query_res.results[0].results 
        assert query_result_nick in query_res.results[0].results 

        query_req = Query.Request()
        query_req.query_type = query_req.FETCH
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
        
        expected_tree = ResultTree()

        company_var_thing = Thing()
        company_var_thing.type = Thing.ENTITY
        company_var_thing.variable_name = 'company_var'
        company_var_thing.type_name = 'company'

        address_attr = Attribute()
        address_attr.label = 'address'
        address_attr.variable_name = 'address'
        address_attr.value.type = ParameterType.PARAMETER_STRING_ARRAY
        address_attr.value.string_array_value = ['street 1', 'street 2']
        company_var_thing.attributes.append(address_attr)

        name_attr = Attribute()
        name_attr.label = 'name'
        name_attr.variable_name = 'name'
        name_attr.value.type = ParameterType.PARAMETER_STRING_ARRAY
        name_attr.value.string_array_value = ['TU Delft']
        company_var_thing.attributes.append(name_attr)

        company_var_query_result = QueryResult()
        company_var_query_result.type = QueryResult.THING
        company_var_query_result.result_index = 0
        company_var_query_result.thing = company_var_thing

        employee_names_subquery = QueryResult()
        employee_names_subquery.type = QueryResult.SUB_QUERY
        employee_names_subquery.result_index = 1
        employee_names_subquery.sub_query_name = 'employee_names'
        
        employee_var_thing1 = Thing()
        employee_var_thing1.type = Thing.ENTITY
        employee_var_thing1.variable_name = 'employee_var'
        employee_var_thing1.type_name = 'person'

        email_attr = Attribute()
        email_attr.label = 'email'
        email_attr.variable_name = 'email'
        email_attr.value.type = ParameterType.PARAMETER_STRING_ARRAY
        email_attr.value.string_array_value = ['phd@tudelft.nl']
        employee_var_thing1.attributes.append(email_attr)

        full_name_attr = Attribute()
        full_name_attr.label = 'full-name'
        full_name_attr.variable_name = 'full-name'
        full_name_attr.value.type = ParameterType.PARAMETER_STRING_ARRAY
        full_name_attr.value.string_array_value = ['PhD candidate 1']
        employee_var_thing1.attributes.append(full_name_attr)

        employee_var_thing1_result = QueryResult()
        employee_var_thing1_result.type = QueryResult.THING
        employee_var_thing1_result.result_index = 2
        employee_var_thing1_result.thing = employee_var_thing1
        
        employment_var_thing1 = Thing()
        employment_var_thing1.type = Thing.RELATION
        employment_var_thing1.variable_name = 'employment_var'
        employment_var_thing1.type_name = 'employment'

        salary_attr = Attribute()
        salary_attr.label = 'salary'
        salary_attr.variable_name = 'salary'
        salary_attr.value.type = ParameterType.PARAMETER_INTEGER_ARRAY
        salary_attr.value.integer_array_value = [30000]
        employment_var_thing1.attributes.append(salary_attr)

        employment_var_thing1_result = QueryResult()
        employment_var_thing1_result.type = QueryResult.THING
        employment_var_thing1_result.result_index = 3
        employment_var_thing1_result.thing = employment_var_thing1
        
        list_index = IndexList()
        list_index.index.append(2)
        list_index.index.append(3)
        employee_names_subquery.children_index.append(list_index)
        
        ## Subtree 2
        employee_var_thing2 = Thing()
        employee_var_thing2.type = Thing.ENTITY
        employee_var_thing2.variable_name = 'employee_var'
        employee_var_thing2.type_name = 'person'

        email_attr = Attribute()
        email_attr.label = 'email'
        email_attr.variable_name = 'email'
        email_attr.value.type = ParameterType.PARAMETER_STRING_ARRAY
        email_attr.value.string_array_value = ['boss@tudelft.nl']
        employee_var_thing2.attributes.append(email_attr)

        full_name_attr = Attribute()
        full_name_attr.label = 'full-name'
        full_name_attr.variable_name = 'full-name'
        full_name_attr.value.type = ParameterType.PARAMETER_STRING_ARRAY
        full_name_attr.value.string_array_value = ['Big Boss']
        employee_var_thing2.attributes.append(full_name_attr)

        employee_var_thing2_result = QueryResult()
        employee_var_thing2_result.type = QueryResult.THING
        employee_var_thing2_result.result_index = 4
        employee_var_thing2_result.thing = employee_var_thing2

        employment_var_thing2 = Thing()
        employment_var_thing2.type = Thing.RELATION
        employment_var_thing2.variable_name = 'employment_var'
        employment_var_thing2.type_name = 'employment'

        salary_attr = Attribute()
        salary_attr.label = 'salary'
        salary_attr.variable_name = 'salary'
        salary_attr.value.type = ParameterType.PARAMETER_INTEGER_ARRAY
        salary_attr.value.integer_array_value = [999999]
        employment_var_thing2.attributes.append(salary_attr)

        employment_var_thing2_result = QueryResult()
        employment_var_thing2_result.type = QueryResult.THING
        employment_var_thing2_result.result_index = 5
        employment_var_thing2_result.thing = employment_var_thing2

        list_index = IndexList()
        list_index.index.append(4)
        list_index.index.append(5)
        employee_names_subquery.children_index.append(list_index)

        ## Subtree 3
        employee_var_thing3 = Thing()
        employee_var_thing3.type = Thing.ENTITY
        employee_var_thing3.variable_name = 'employee_var'
        employee_var_thing3.type_name = 'person'

        email_attr = Attribute()
        email_attr.label = 'email'
        email_attr.variable_name = 'email'
        email_attr.value.type = ParameterType.PARAMETER_STRING_ARRAY
        email_attr.value.string_array_value = ['guest@tudelft.nl']
        employee_var_thing3.attributes.append(email_attr)

        full_name_attr = Attribute()
        full_name_attr.label = 'full-name'
        full_name_attr.variable_name = 'full-name'
        full_name_attr.value.type = ParameterType.PARAMETER_STRING_ARRAY
        full_name_attr.value.string_array_value = ['Random guest']
        employee_var_thing3.attributes.append(full_name_attr)
        
        employee_var_thing3_result = QueryResult()
        employee_var_thing3_result.type = QueryResult.THING
        employee_var_thing3_result.result_index = 6
        employee_var_thing3_result.thing = employee_var_thing3
        
        employment_var_thing3 = Thing()
        employment_var_thing3.type = Thing.RELATION
        employment_var_thing3.variable_name = 'employment_var'
        employment_var_thing3.type_name = 'employment'

        salary_attr = Attribute()
        salary_attr.label = 'salary'
        salary_attr.variable_name = 'salary'
        salary_attr.value.type = ParameterType.PARAMETER_INTEGER_ARRAY
        salary_attr.value.integer_array_value = [0]
        employment_var_thing3.attributes.append(salary_attr)

        employment_var_thing3_result = QueryResult()
        employment_var_thing3_result.type = QueryResult.THING
        employment_var_thing3_result.result_index = 7
        employment_var_thing3_result.thing = employment_var_thing3

        list_index = IndexList()
        list_index.index.append(6)
        list_index.index.append(7)
        employee_names_subquery.children_index.append(list_index)

        expected_tree.results.append(company_var_query_result)
    
        expected_tree.results.append(employee_names_subquery)
        
        expected_tree.results.append(employee_var_thing1_result)
        expected_tree.results.append(employment_var_thing1_result)

        expected_tree.results.append(employee_var_thing2_result)
        expected_tree.results.append(employment_var_thing2_result)

        expected_tree.results.append(employee_var_thing3_result)
        expected_tree.results.append(employment_var_thing3_result)

        query_res = node.call_service(node.query_srv, query_req)
        
        assert query_res.success is True
        assert len(query_res.results) == 1

        expected = [company_var_query_result, employee_names_subquery,
                    employee_var_thing1_result, employment_var_thing1_result,
                    employee_var_thing2_result, employment_var_thing2_result,
                    employee_var_thing3_result, employment_var_thing3_result]

        actual = []
        for result in query_res.results[0].results:
            actual.append(result)

        import copy
        def strip_indices(obj):
            # Make a shallow copy to avoid mutating the original
            obj_copy = copy.copy(obj)
            if hasattr(obj_copy, 'result_index'):
                obj_copy.result_index = 0
            if hasattr(obj_copy, 'children_index'):
                obj_copy.children_index = [IndexList()]
            return obj_copy

        # Compare expected and actual ignoring result_index and children_index
        for item in expected:
            assert any(strip_indices(item) == strip_indices(a) for a in actual)

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
        query_req.query_type = query_req.GET
        query_req.query = """
            match
                $p isa person, has full-name $name, has email $email;
            get $name, $email;
            sort $name asc; limit 3;
        """
        query_res = node.call_service(node.query_srv, query_req)

        assert query_res.success is True
        assert len(query_res.results) == 3 
        assert len(query_res.results[0].results) == 2
        assert len(query_res.results[1].results) == 2
        assert len(query_res.results[2].results) == 2

        correct_name = False
        correct_email = False
        for r in query_res.results[0].results:
            if r.attribute.variable_name == 'name' and r.attribute.label == 'full-name' \
               and r.attribute.value.string_value == 'Ahmed Frazier':
                correct_name = True
            if r.attribute.variable_name == 'email' and r.attribute.label == 'email' and \
               r.attribute.value.string_value == 'ahmed.frazier@gmail.com':
                correct_email = True
        assert correct_name and correct_email

        correct_name = False
        correct_email = False
        for r in query_res.results[1].results:
            if r.attribute.variable_name == 'name' and r.attribute.label == 'full-name' \
               and r.attribute.value.string_value == 'Big Boss':
                correct_name = True
            if r.attribute.variable_name == 'email' and r.attribute.label == 'email' and \
               r.attribute.value.string_value == 'boss@tudelft.nl':
                correct_email = True
        assert correct_name and correct_email

        correct_name = False
        correct_email = False
        for r in query_res.results[2].results:
            if r.attribute.variable_name == 'name' and r.attribute.label == 'full-name' \
               and r.attribute.value.string_value == 'Dominic Lyons':
                correct_name = True
            if r.attribute.variable_name == 'email' and r.attribute.label == 'email' and \
               r.attribute.value.string_value == 'dominic.lyons@gmail.com':
                correct_email = True
        assert correct_name and correct_email
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
        query_req.query_type = query_req.GET_AGGREGATE
        query_req.query = """
            match
                $person isa person,
                    has email "test@test.com";
                (employee:$person) isa employment, has salary $s;
            get $s; sum $s;
        """
        query_res = node.call_service(node.query_srv, query_req)

        assert query_res.success is True
        assert query_res.results[0].results[0].type == QueryResult.ATTRIBUTE
        assert query_res.results[0].results[0].attribute.value.integer_value == 1500
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
