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
import json
from pathlib import Path
from typing import Any
from typing import TYPE_CHECKING

from ros_typedb_msgs.srv import Query

if TYPE_CHECKING:
    from ros_typedb_tools.stress_mixed import MixedQueryProfile


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

INVARIANT_PROFILE_FILES = {
    'plan-schema-mixed': 'plan_schema_mixed_invariants.json',
    'plan-schema': 'plan_schema_invariants.json',
    'test-data': 'test_data_invariants.json',
}
INVARIANT_PROFILE_NAMES = ('none', *INVARIANT_PROFILE_FILES)
INVARIANT_PROFILE_DIR = Path(__file__).with_name('profiles')

MIXED_PROFILE_FILES = {
    'plan-schema': 'plan_schema_mixed.json',
    'test-data': 'test_data_mixed.json',
}
MIXED_PROFILE_NAMES = ('auto', *MIXED_PROFILE_FILES)


@dataclass(frozen=True)
class QuerySpec:
    """Query and service query type used by one request."""

    query: str
    query_type: str
    cleanup_key: str | None = None


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


@dataclass(frozen=True)
class InvariantSpec:
    """Query and expected scalar value for one invariant check."""

    name: str
    query: str
    query_type: str
    expected_value: Any


@dataclass(frozen=True)
class InvariantRecord:
    """Result of one invariant query."""

    index: int
    phase: str
    name: str
    query_type: str
    query: str
    started_at_s: float
    ended_at_s: float
    latency_s: float
    success: bool
    timed_out: bool
    error_message: str
    expected_value: Any
    actual_value: Any


@dataclass(frozen=True)
class ResolvedStressConfig:
    """Concrete runtime configuration derived from CLI arguments.

    The CLI exposes convenient selectors such as ``--mixed-profile auto``.
    The runner should not need to interpret those selectors while it is also
    managing ROS futures. This object stores the selected query mix, invariant
    checks, and mixed profile after all CLI-level choices have been resolved.
    """

    query_specs: list[QuerySpec]
    invariant_specs: list[InvariantSpec]
    mixed_profile: MixedQueryProfile | None
    mixed_profile_name: str | None
    invariant_profile_name: str
    max_in_flight: int
    debug_events_output: Path | None


@dataclass(frozen=True)
class StressExperimentResult:
    """Complete stress experiment result."""

    records: list[RequestRecord]
    invariant_records: list[InvariantRecord]
    config: ResolvedStressConfig


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
    if args.invariant_profile not in INVARIANT_PROFILE_NAMES:
        raise ValueError(
            f'unsupported invariant profile: {args.invariant_profile}'
        )
    if args.invariant_period_s < 0:
        raise ValueError(
            '--invariant-period-s must be greater than or equal to zero'
        )
    mixed_profile = getattr(args, 'mixed_profile', 'auto')
    if mixed_profile not in MIXED_PROFILE_NAMES:
        raise ValueError(f'unsupported mixed profile: {mixed_profile}')


def build_query_specs(args: argparse.Namespace) -> list[QuerySpec]:
    """Build fixed query specs for explicit-query or read-only modes.

    Mixed mode uses schema-specific templates and is resolved in
    ``stress_resolution``. Keeping this helper fixed-query-only avoids hidden
    profile loading from a generic config function.
    """
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

    if args.mode == 'mixed':
        raise ValueError('mixed mode requires a resolved mixed profile')

    raise ValueError(f'unsupported mode: {args.mode}')


def build_invariant_specs(args: argparse.Namespace) -> list[InvariantSpec]:
    """Build invariant checks for the selected invariant profile."""
    if args.invariant_profile == 'none':
        return []

    profile_filename = INVARIANT_PROFILE_FILES.get(args.invariant_profile)
    if profile_filename is None:
        raise ValueError(
            f'unsupported invariant profile: {args.invariant_profile}'
        )

    return _load_invariant_profile(INVARIANT_PROFILE_DIR / profile_filename)


def _load_invariant_profile(profile_path: Path) -> list[InvariantSpec]:
    """Load invariant checks from a packaged JSON profile."""
    payload = json.loads(profile_path.read_text(encoding='utf-8'))
    invariants = payload.get('invariants', [])
    specs = []
    for invariant in invariants:
        query_type = invariant['query_type']
        if query_type not in QUERY_TYPE_BY_NAME:
            raise ValueError(f'unsupported invariant query type: {query_type}')
        specs.append(
            InvariantSpec(
                name=invariant['name'],
                query=invariant['query'],
                query_type=query_type,
                expected_value=invariant['expected_value'],
            )
        )
    return specs


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
