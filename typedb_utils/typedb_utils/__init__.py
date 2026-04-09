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
"""Reusable TypeDB 3 Python utilities."""

from typedb_utils.typedb_helpers import (
    attribute_dict_to_query,
    AttributePair,
    convert_py_type_to_query_type,
    convert_query_type_to_py_type,
    create_match_query,
    create_relationship_query,
    dict_to_query,
    RelatedThingsDict,
    RelatedVariablesDict,
    ThingMatchTuple,
)
from typedb_utils.typedb_interface import MatchResultDict
from typedb_utils.typedb_interface import TypeDBInterface
from typedb_utils.typedb_interface import TypeDBQueryError

__all__ = [
    'AttributePair',
    'MatchResultDict',
    'RelatedThingsDict',
    'RelatedVariablesDict',
    'ThingMatchTuple',
    'TypeDBInterface',
    'TypeDBQueryError',
    'attribute_dict_to_query',
    'convert_py_type_to_query_type',
    'convert_query_type_to_py_type',
    'create_match_query',
    'create_relationship_query',
    'dict_to_query',
]
