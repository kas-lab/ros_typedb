# Copyright 2026 KAS Lab
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
"""typedb_helpers - standalone helper functions for TypeDB operations."""

from typing import Literal
from typing import Optional
from typing import Tuple

from ros_typedb.typedb_interface import convert_py_type_to_query_type
from ros_typedb.typedb_interface import convert_query_type_to_py_type
from ros_typedb.typedb_interface import TypeDBInterface


def attribute_dict_to_query(db: TypeDBInterface, attribute_dict: dict) -> str:
    """
    Convert an attribute dict to a TypeQL 'has' clause fragment.

    :param db: TypeDBInterface instance (unused, kept for API consistency).
    :param attribute_dict: dict mapping attribute names to values.
    :return: TypeQL string fragment like " has email 'x', has age 5".
    """
    _query = ''
    first = True
    for attr, attr_value in attribute_dict.items():
        if not isinstance(attr_value, list):
            attr_value = [attr_value]
        for v in attr_value:
            if first is False:
                _query += ','
            _query += ' has {0} {1}'.format(attr, convert_py_type_to_query_type(v))
            first = False
    return _query


def create_match_query(
        db: TypeDBInterface,
        things_list: list,
        prefix: str = 't') -> Tuple[str, list]:
    """
    Build a match clause from a list of (type, attr, value) tuples.

    :param db: TypeDBInterface instance (unused, kept for API consistency).
    :param things_list: list of (thing_type, attr_name, attr_value) tuples.
    :param prefix: variable name prefix.
    :return: tuple of (match_clause_string, list_of_variable_names).
    """
    match_query = ''
    prefix_list = []
    t_counter = 0
    for thing in things_list:
        match_query += ' ${0}_{1} isa {2},'.format(prefix, t_counter, thing[0])
        match_query += ' has {0} {1};'.format(
            thing[1], convert_py_type_to_query_type(thing[2]))
        prefix_list.append('{0}_{1}'.format(prefix, t_counter))
        t_counter += 1
    return match_query, prefix_list


def create_relationship_query(
        db: TypeDBInterface,
        relationship: str,
        related_dict: dict,
        attribute_list: Optional[list] = None,
        prefix: str = 'r') -> str:
    """
    Build a TypeQL insert clause for a relationship.

    :param db: TypeDBInterface instance (unused, kept for API consistency).
    :param relationship: relationship type name.
    :param related_dict: dict mapping role names to lists of variable name strings.
    :param attribute_list: list of (attr_name, attr_value) tuples.
    :param prefix: variable name prefix for the relationship variable.
    :return: TypeQL insert clause string.
    """
    if attribute_list is None:
        attribute_list = []
    related_things = ''
    for role, variables in related_dict.items():
        for v in variables:
            aux = '{0}:${1}'.format(role, v)
            if related_things != '':
                aux = ',' + aux
            related_things += aux
    query = ' ({0}) isa {1}'.format(related_things, relationship)
    for attribute in attribute_list:
        if attribute[0] is not None:
            query += ', has {} {} '.format(
                attribute[0], convert_py_type_to_query_type(attribute[1]))
    query += ';'
    return query


def delete_thing(
        db: TypeDBInterface,
        thing: str,
        key: str,
        key_value) -> Literal[True] | None:
    """
    Delete a thing from the database by its key attribute.

    :param db: TypeDBInterface instance.
    :param thing: thing type (e.g. 'person').
    :param key: key attribute name (e.g. 'email').
    :param key_value: key attribute value.
    :return: True on success, None on failure.
    """
    query = 'match $thing isa {}, has {} {}; delete $thing;'.format(
        thing, key, convert_py_type_to_query_type(key_value))
    return db.delete_from_database(query)


def insert_entity(
        db: TypeDBInterface,
        entity: str,
        attribute_list: Optional[list] = None) -> bool | None:
    """
    Insert an entity into the database with optional attributes.

    :param db: TypeDBInterface instance.
    :param entity: entity type name.
    :param attribute_list: list of (attr_name, attr_value) tuples.
    :return: True on success, None on failure.
    """
    if attribute_list is None:
        attribute_list = []
    query = 'insert $entity isa {}'.format(entity)
    for attribute in attribute_list:
        if attribute[0] is not None:
            value = convert_py_type_to_query_type(attribute[1])
            query += ', has {} {}'.format(attribute[0], value)
    query += ';'
    return db.insert_database(query)


def insert_relationship(
        db: TypeDBInterface,
        relationship: str,
        related_dict: dict,
        attribute_list: Optional[list] = None) -> bool | None:
    """
    Insert a relationship between existing things.

    :param db: TypeDBInterface instance.
    :param relationship: relationship type name.
    :param related_dict: dict mapping role names to lists of (type, key, value) tuples.
    :param attribute_list: list of (attr_name, attr_value) tuples for the relationship.
    :return: True on success, None on failure.
    """
    if attribute_list is None:
        attribute_list = []
    match_query = 'match '
    _related_dict = {}
    for key, things in related_dict.items():
        _match_query, _prefix_list = create_match_query(db, things, key)
        match_query += _match_query
        _related_dict[key] = _prefix_list
    insert_query = 'insert ' + create_relationship_query(
        db, relationship, _related_dict, attribute_list=attribute_list, prefix=relationship)
    return db.insert_database(match_query + insert_query)


def fetch_attribute_from_thing(
        db: TypeDBInterface,
        thing: str,
        key_attr_list: list,
        attr: str) -> list:
    """
    Fetch attribute values from a thing matched by one or more key attributes.

    :param db: TypeDBInterface instance.
    :param thing: thing type name (e.g. 'person').
    :param key_attr_list: list of (attr_name, attr_value) pairs to match on.
    :param attr: attribute name to fetch.
    :return: list of attribute values (Python-typed).
    """
    query = 'match $thing isa {}'.format(thing)
    for (key, value) in key_attr_list:
        query += ', has {} {}'.format(key, convert_py_type_to_query_type(value))
    query += ', has {} $attribute; select $attribute;'.format(attr)
    result = db.get_database(query)
    return [
        convert_query_type_to_py_type(value_dict=r.get('attribute'))
        for r in result
    ]


def fetch_attribute_from_thing_raw(
        db: TypeDBInterface,
        thing: str,
        key_attr_list: list,
        attr: str) -> list:
    """
    Fetch raw normalised attribute dicts from a thing matched by key attributes.

    :param db: TypeDBInterface instance.
    :param thing: thing type name.
    :param key_attr_list: list of (attr_name, attr_value) pairs to match on.
    :param attr: attribute name to fetch.
    :return: list of dicts with key 'attribute' containing normalised attribute dict.
    """
    query = 'match $thing isa {}'.format(thing)
    for (key, value) in key_attr_list:
        query += ', has {} {}'.format(key, convert_py_type_to_query_type(value))
    query += ', has {} $attribute; select $attribute;'.format(attr)
    return db.get_database(query)


def delete_attribute_from_thing(
        db: TypeDBInterface,
        thing: str,
        key: str,
        key_value,
        attr: str) -> Literal[True] | None:
    """
    Delete an attribute from a thing matched by a key attribute.

    :param db: TypeDBInterface instance.
    :param thing: thing type name.
    :param key: key attribute name.
    :param key_value: key attribute value.
    :param attr: attribute name to delete.
    :return: True on success, None on failure.
    """
    key_value = convert_py_type_to_query_type(key_value)
    query = (
        'match $thing isa {}, has {} {}, has {} $attribute;'
        ' delete $attribute of $thing;'
    ).format(thing, key, key_value, attr)
    return db.delete_from_database(query)


def insert_attribute_in_thing(
        db: TypeDBInterface,
        thing: str,
        key: str,
        key_value,
        attr: str,
        attr_value) -> bool | None:
    """
    Insert an attribute into a thing matched by a key attribute.

    :param db: TypeDBInterface instance.
    :param thing: thing type name.
    :param key: key attribute name.
    :param key_value: key attribute value.
    :param attr: attribute name to insert.
    :param attr_value: attribute value to insert.
    :return: True on success, None on failure.
    """
    key_value = convert_py_type_to_query_type(key_value)
    attr_value = convert_py_type_to_query_type(attr_value)
    query = 'match $thing isa {}, has {} {}; insert $thing has {} {};'.format(
        thing, key, key_value, attr, attr_value)
    return db.insert_database(query)


def delete_attributes_from_thing(
        db: TypeDBInterface,
        thing: str,
        key: str,
        key_value,
        attr_list: list) -> None:
    """
    Delete multiple attributes from a thing matched by a key attribute.

    :param db: TypeDBInterface instance.
    :param thing: thing type name.
    :param key: key attribute name.
    :param key_value: key attribute value.
    :param attr_list: list of attribute names to delete.
    """
    for attr in attr_list:
        delete_attribute_from_thing(db, thing, key, key_value, attr)


def insert_attributes_in_thing(
        db: TypeDBInterface,
        thing: str,
        key: str,
        key_value,
        attribute_list: list) -> None:
    """
    Insert multiple attributes into a thing matched by a key attribute.

    :param db: TypeDBInterface instance.
    :param thing: thing type name.
    :param key: key attribute name.
    :param key_value: key attribute value.
    :param attribute_list: list of (attr_name, attr_value) tuples to insert.
    """
    for attr, attr_value in attribute_list:
        insert_attribute_in_thing(db, thing, key, key_value, attr, attr_value)


def update_attribute_in_thing(
        db: TypeDBInterface,
        thing: str,
        key: str,
        key_value,
        attr: str,
        attr_value) -> bool | None:
    """
    Update an attribute in a thing by deleting the old value and inserting the new one.

    :param db: TypeDBInterface instance.
    :param thing: thing type name.
    :param key: key attribute name.
    :param key_value: key attribute value.
    :param attr: attribute name to update.
    :param attr_value: new attribute value.
    :return: True on success, None on failure.
    """
    delete_attribute_from_thing(db, thing, key, key_value, attr)
    return insert_attribute_in_thing(db, thing, key, key_value, attr, attr_value)


def update_attributes_in_thing(
        db: TypeDBInterface,
        thing: str,
        key: str,
        key_value,
        attribute_list: list) -> None:
    """
    Update multiple attributes in a thing.

    :param db: TypeDBInterface instance.
    :param thing: thing type name.
    :param key: key attribute name.
    :param key_value: key attribute value.
    :param attribute_list: list of (attr_name, new_attr_value) tuples.
    """
    for attr, attr_value in attribute_list:
        update_attribute_in_thing(db, thing, key, key_value, attr, attr_value)


def dict_to_query(db: TypeDBInterface, data_dict: dict) -> str:
    """
    Convert a dict of attributes to a TypeQL 'has' clause fragment.

    :param db: TypeDBInterface instance (unused, kept for API consistency).
    :param data_dict: dict mapping attribute names to values.
    :return: TypeQL string fragment.
    """
    return attribute_dict_to_query(db, data_dict)
