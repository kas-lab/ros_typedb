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

import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

from rcl_interfaces.msg import ParameterType
from rclpy.lifecycle import TransitionCallbackReturn

from ros_typedb.ros_typedb_interface import convert_attribute_dict_to_ros_msg
from ros_typedb.ros_typedb_interface import fetch_result_to_ros_result_tree
from ros_typedb.ros_typedb_interface import ROSTypeDBInterface

from ros_typedb_msgs.msg import Attribute
from ros_typedb_msgs.msg import IndexList
from ros_typedb_msgs.msg import QueryResult
from ros_typedb_msgs.msg import ResultTree
from ros_typedb_msgs.msg import Thing
from ros_typedb_msgs.srv import Query


def test_query_srv_has_timeout_s_field():
    """Query.Request must have a timeout_s field defaulting to 0.0."""
    req = Query.Request()
    assert hasattr(req, 'timeout_s')
    assert req.timeout_s == 0.0


def test_init_typedb_interface_accepts_query_timeout_s():
    """init_typedb_interface passes query_timeout_s to TypeDBInterface constructor."""
    captured = {}

    class FakeTypeDBInterface:

        def __init__(self, *args, **kwargs):
            captured.update(kwargs)
            self.insert_data_event = None
            self.delete_data_event = None

    node = ROSTypeDBInterface.__new__(ROSTypeDBInterface)
    node.typedb_interface_class = FakeTypeDBInterface
    node.init_typedb_interface(
        address='localhost:1729',
        database_name='test',
        query_timeout_s=15.0)

    assert captured.get('query_timeout_s') == 15.0


def test_init_typedb_interface_accepts_reload_schema():
    """init_typedb_interface passes reload_schema to TypeDBInterface."""
    captured = {}

    class FakeTypeDBInterface:

        def __init__(self, *args, **kwargs):
            captured.update(kwargs)
            self.insert_data_event = None
            self.delete_data_event = None

    node = ROSTypeDBInterface.__new__(ROSTypeDBInterface)
    node.typedb_interface_class = FakeTypeDBInterface
    node.init_typedb_interface(
        address='localhost:1729',
        database_name='test',
        reload_schema=False)

    assert captured.get('reload_schema') is False


def test_query_service_cb_passes_timeout_to_wrapper():
    """query_service_cb extracts req.timeout_s and passes it as timeout= to the wrapper."""
    captured = {}

    class FakeTypeDBInterface:
        last_error = ''

        def fetch_database(self, query, timeout=None):
            captured['timeout'] = timeout
            return []

    node = ROSTypeDBInterface.__new__(ROSTypeDBInterface)
    node.typedb_interface = FakeTypeDBInterface()

    req = Query.Request()
    req.query_type = Query.Request.FETCH
    req.query = 'match $x isa thing; fetch $x;'
    req.timeout_s = 5.0

    response = Query.Response()
    node.query_service_cb(req, response)

    assert captured.get('timeout') == 5.0


def test_query_service_cb_passes_none_when_timeout_s_is_zero():
    """query_service_cb converts timeout_s=0.0 to timeout=None (use node default)."""
    captured = {}

    class FakeTypeDBInterface:
        last_error = ''

        def fetch_database(self, query, timeout=None):
            captured['timeout'] = timeout
            return []

    node = ROSTypeDBInterface.__new__(ROSTypeDBInterface)
    node.typedb_interface = FakeTypeDBInterface()

    req = Query.Request()
    req.query_type = Query.Request.FETCH
    req.query = 'match $x isa thing; fetch $x;'
    req.timeout_s = 0.0

    response = Query.Response()
    node.query_service_cb(req, response)

    assert captured.get('timeout') is None


class MockTypeDBDriver:

    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


class MockTypeDBInterface:

    def __init__(self):
        self.driver = MockTypeDBDriver()


def test_close_typedb_interface_closes_driver_and_clears_reference():
    ros_typedb_interface = ROSTypeDBInterface.__new__(ROSTypeDBInterface)
    typedb_interface = MockTypeDBInterface()
    ros_typedb_interface.typedb_interface = typedb_interface

    ros_typedb_interface.close_typedb_interface()

    assert typedb_interface.driver.closed is True
    assert ros_typedb_interface.typedb_interface is None


def test_fetch_result_to_ros_result_tree_accepts_more_than_uint8_indices():
    json_test = {
        f'attr_{index}': {
            'value': index,
            'type': {
                'label': 'age',
                'root': 'attribute',
                'value_type': 'long'}}
        for index in range(260)
    }

    result_tree, tree_index = fetch_result_to_ros_result_tree(json_test)

    assert len(result_tree.results) == 260
    assert tree_index == 260
    assert result_tree.results[255].result_index == 255
    assert result_tree.results[256].result_index == 256

    index_list = IndexList()
    index_list.index = list(range(260))
    assert index_list.index[256] == 256


def test_close_typedb_interface_closes_driver_and_clears_interface():
    driver = MagicMock()
    node = SimpleNamespace(typedb_interface=SimpleNamespace(driver=driver))

    ROSTypeDBInterface.close_typedb_interface(node)

    driver.close.assert_called_once_with()
    assert node.typedb_interface is None


def test_on_cleanup_destroys_ros_entities_and_closes_typedb_driver():
    driver = MagicMock()
    typedb_interface = SimpleNamespace(driver=driver)

    event_pub = MagicMock()
    query_service = MagicMock()
    delete_db_service = MagicMock()

    node = SimpleNamespace(
        event_pub=event_pub,
        query_service=query_service,
        delete_db_service=delete_db_service,
        typedb_interface=typedb_interface,
        destroy_publisher=MagicMock(return_value=True),
        destroy_service=MagicMock(return_value=True),
        get_logger=MagicMock(return_value=MagicMock()),
        get_name=MagicMock(return_value='ros_typedb'),
        close_typedb_interface=MagicMock(
            side_effect=lambda: ROSTypeDBInterface.close_typedb_interface(node)
        ),
        _get_service_callback_lock=(
            lambda: ROSTypeDBInterface._get_service_callback_lock(node))
    )

    result = ROSTypeDBInterface.on_cleanup(node, None)

    assert result == TransitionCallbackReturn.SUCCESS
    node.destroy_publisher.assert_called_once_with(event_pub)
    node.destroy_service.assert_any_call(query_service)
    node.destroy_service.assert_any_call(delete_db_service)
    assert node.destroy_service.call_count == 2
    assert node.close_typedb_interface.call_count == 1
    driver.close.assert_called_once_with()
    assert not hasattr(node, 'event_pub')
    assert not hasattr(node, 'query_service')
    assert not hasattr(node, 'delete_db_service')
    assert node.typedb_interface is None


def test_on_cleanup_waits_for_active_service_callback_before_closing_driver():
    driver = MagicMock()
    typedb_interface = SimpleNamespace(driver=driver)
    service_callback_lock = threading.Lock()
    service_callback_lock.acquire()

    node = SimpleNamespace(
        event_pub=MagicMock(),
        query_service=MagicMock(),
        delete_db_service=MagicMock(),
        typedb_interface=typedb_interface,
        _service_callback_lock=service_callback_lock,
        destroy_publisher=MagicMock(return_value=True),
        destroy_service=MagicMock(return_value=True),
        get_logger=MagicMock(return_value=MagicMock()),
        get_name=MagicMock(return_value='ros_typedb'),
        close_typedb_interface=MagicMock(
            side_effect=lambda: ROSTypeDBInterface.close_typedb_interface(node)
        ),
        _get_service_callback_lock=(
            lambda: ROSTypeDBInterface._get_service_callback_lock(node))
    )

    cleanup_thread = threading.Thread(
        target=ROSTypeDBInterface.on_cleanup,
        args=(node, None))
    cleanup_thread.start()

    time.sleep(0.05)
    driver.close.assert_not_called()
    assert node.typedb_interface is typedb_interface

    service_callback_lock.release()
    cleanup_thread.join(timeout=1.0)

    assert not cleanup_thread.is_alive()
    driver.close.assert_called_once_with()
    assert node.typedb_interface is None


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
        'company_var': {
            'address': [
                {'value': 'street 1',
                 'type': {'label': 'address',
                          'root': 'attribute',
                          'value_type': 'string'}},
                {'value': 'street 2',
                 'type': {'label': 'address',
                          'root': 'attribute',
                          'value_type': 'string'}}
            ],
            'name': [{'value': 'TU Delft',
                     'type': {'label': 'name',
                              'root': 'attribute',
                              'value_type': 'string'}}],
            'type': {'label': 'company', 'root': 'entity'}
        }
    }
    address_attr = convert_attribute_dict_to_ros_msg(
        'company_var', test_dict['company_var']['address'])
    assert expected_address_attr == address_attr

    name_attr = convert_attribute_dict_to_ros_msg(
        'company_var', test_dict['company_var']['name'])
    assert expected_name_attr == name_attr

    test_dict = {
        'age': {
            'value': 33,
            'type': {
                'label': 'age',
                'root': 'attribute',
                'value_type': 'long'}},
        'alive': {
            'value': True,
            'type': {
                'label': 'alive',
                'root': 'attribute',
                'value_type': 'boolean'}},
        'date': {
            'value': '1990-06-01T00:00:00.000',
            'type': {
                'label': 'birth-date',
                'root': 'attribute',
                'value_type': 'datetime'}},
        'height': {
            'value': 1,
            'type': {
                'label': 'height',
                'root': 'attribute',
                'value_type': 'double'}},
        'nick': {
            'value': 'test',
            'type': {
                'label': 'nickname',
                'root': 'attribute',
                'value_type': 'string'}}}
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

    height_attr = convert_attribute_dict_to_ros_msg(
        'height', test_dict['height'])
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
        'age': {
            'value': 33,
            'type': {
                'label': 'age',
                'root': 'attribute',
                'value_type': 'long'}},
        'alive': {
            'value': True,
            'type': {
                'label': 'alive',
                'root': 'attribute',
                'value_type': 'boolean'}},
        'date': {
            'value': '1990-06-01T00:00:00.000',
            'type': {
                'label': 'birth-date',
                'root': 'attribute',
                'value_type': 'datetime'}},
        'height': {
            'value': 1,
            'type': {
                'label': 'height',
                'root': 'attribute',
                'value_type': 'double'}},
        'nick': {
            'value': 'test',
            'type': {
                'label': 'nickname',
                'root': 'attribute',
                'value_type': 'string'}}}

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

    json_test = {
        'company_var': {
            'address': [
                {'value': 'street 1', 'type': {
                    'label': 'address',
                    'root': 'attribute',
                    'value_type': 'string'}},
                {'value': 'street 2', 'type': {
                    'label': 'address',
                    'root': 'attribute',
                    'value_type': 'string'}}
            ],
            'name': [{'value': 'TU Delft', 'type': {
                'label': 'name',
                'root': 'attribute',
                'value_type': 'string'}}],
            'type': {'label': 'company', 'root': 'entity'}
        },
        'employee_names': [
            {
                'employee_var': {
                    'email': [{'value': 'phd@tudelft.nl', 'type': {
                        'label': 'email',
                        'root': 'attribute',
                        'value_type': 'string'}}],
                    'full-name': [{'value': 'PhD candidate 1', 'type': {
                        'label': 'full-name',
                        'root': 'attribute',
                        'value_type': 'string'}}],
                    'type': {'label': 'person', 'root': 'entity'}
                },
                'employment_var': {
                    'salary': [{'value': 30000, 'type': {
                        'label': 'salary',
                        'root': 'attribute',
                        'value_type': 'long'}}],
                    'type': {'label': 'employment', 'root': 'relation'}
                }
            },
            {
                'employee_var': {
                    'email': [{'value': 'boss@tudelft.nl', 'type': {
                        'label': 'email',
                        'root': 'attribute',
                        'value_type': 'string'}}],
                    'full-name': [{'value': 'Big Boss', 'type': {
                        'label': 'full-name',
                        'root': 'attribute',
                        'value_type': 'string'}}],
                    'type': {'label': 'person', 'root': 'entity'}
                },
                'employment_var': {
                    'salary': [{'value': 999999, 'type': {
                        'label': 'salary',
                        'root': 'attribute',
                        'value_type': 'long'}}],
                    'type': {'label': 'employment', 'root': 'relation'}
                }
            },
            {
                'employee_var': {
                    'email': [{'value': 'guest@tudelft.nl', 'type': {
                        'label': 'email',
                        'root': 'attribute',
                        'value_type': 'string'}}],
                    'full-name': [{'value': 'Random guest', 'type': {
                        'label': 'full-name',
                        'root': 'attribute',
                        'value_type': 'string'}}],
                    'type': {'label': 'person', 'root': 'entity'}
                },
                'employment_var': {
                    'salary': [{'value': 0, 'type': {
                        'label': 'salary',
                        'root': 'attribute',
                        'value_type': 'long'}}],
                    'type': {'label': 'employment', 'root': 'relation'}
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

    # Subtree 2
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

    # Subtree 3
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
