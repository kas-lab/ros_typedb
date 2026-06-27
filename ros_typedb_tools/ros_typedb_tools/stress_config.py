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
"""Shared configuration helpers for the ros_typedb stress tools."""

from __future__ import annotations

import argparse
from dataclasses import dataclass

from ros_typedb_msgs.srv import Query


QUERY_TYPE_BY_NAME = {
    'insert': Query.Request.INSERT,
    'delete': Query.Request.DELETE,
    'fetch': Query.Request.FETCH,
    'get': Query.Request.GET,
    'get_aggregate': Query.Request.GET_AGGREGATE,
    'define': Query.Request.DEFINE,
    'update': Query.Request.UPDATE,
}

DEFAULT_READ_QUERY_SPECS = (
    (
        'match $x isa entity; get $x;',
        'get',
    ),
    (
        'match $x isa relation; get $x;',
        'get',
    ),
    (
        'match $x isa attribute; get $x;',
        'get',
    ),
)


@dataclass(frozen=True)
class QuerySpec:
    """Query and service query type used by one request."""

    query: str
    query_type: str


@dataclass(frozen=True)
class RequestRecord:
    """Result of one service request."""

    index: int
    client_id: int
    query_index: int
    query_type: str
    query: str
    started_at_s: float
    ended_at_s: float
    latency_s: float
    success: bool
    timed_out: bool
    cancel_requested: bool
    error_message: str
    result_count: int


def validate_experiment_args(args: argparse.Namespace) -> None:
    """Validate stress arguments before ROS resources are created."""
    if args.requests < 1:
        raise ValueError('--requests must be greater than zero')
    if args.clients < 1:
        raise ValueError('--clients must be greater than zero')
    if args.duration_s is not None and args.duration_s <= 0:
        raise ValueError('--duration-s must be greater than zero')
    if args.timeout_s <= 0:
        raise ValueError('--timeout-s must be greater than zero')
    if args.wait_service_timeout_s <= 0:
        raise ValueError('--wait-service-timeout-s must be greater than zero')
    if args.request_gap_s < 0:
        raise ValueError(
            '--request-gap-s must be greater than or equal to zero'
        )
    if args.max_in_flight is not None and args.max_in_flight < 1:
        raise ValueError('--max-in-flight must be greater than zero')
    if args.executor_threads < 1:
        raise ValueError('--executor-threads must be greater than zero')


def build_query_specs(args: argparse.Namespace) -> list[QuerySpec]:
    """Build the query sequence used by the stress experiment."""
    if args.query and not args.query_type:
        raise ValueError('--query-type is required when --query is used')
    if args.query_type and not args.query:
        raise ValueError('--query-type requires --query')

    if args.query:
        return [QuerySpec(query=args.query, query_type=args.query_type)]

    if args.mode == 'read':
        return [
            QuerySpec(query=query, query_type=query_type)
            for query, query_type in DEFAULT_READ_QUERY_SPECS
        ]

    raise ValueError(f'unsupported mode: {args.mode}')


def build_query_request(
    query: str,
    query_type: str,
    timeout_s: float,
) -> Query.Request:
    """Build a Query service request from a TypeDB query spec."""
    request = Query.Request()
    request.query = query
    request.query_type = QUERY_TYPE_BY_NAME[query_type]
    request.timeout_s = timeout_s
    return request
