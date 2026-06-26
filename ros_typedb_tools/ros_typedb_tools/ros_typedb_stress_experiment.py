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
"""Stress experiment CLI for the ros_typedb query service."""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from collections.abc import Sequence
from dataclasses import asdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy

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
    started_at_s: float
    ended_at_s: float
    latency_s: float
    success: bool
    timed_out: bool
    error_message: str
    result_count: int


def build_argument_parser() -> argparse.ArgumentParser:
    """Build command-line arguments for the stress experiment."""
    parser = argparse.ArgumentParser(
        prog='ros_typedb_stress_experiment',
        description='Run a stress experiment against ros_typedb.',
    )
    parser.add_argument(
        '--service-name',
        default='/ros_typedb_interface/query',
        help='Query service name. Defaults to /ros_typedb_interface/query.',
    )
    parser.add_argument(
        '--query',
        help='TypeDB query to send on each request.',
    )
    parser.add_argument(
        '--query-type',
        choices=sorted(QUERY_TYPE_BY_NAME),
        help='TypeDB query type. Required when --query is used.',
    )
    parser.add_argument(
        '--requests',
        type=int,
        default=1,
        help=(
            'Number of requests to send when --duration-s is omitted. '
            'Defaults to 1.'
        ),
    )
    parser.add_argument(
        '--clients',
        type=int,
        default=1,
        help='Number of concurrent service clients. Defaults to 1.',
    )
    parser.add_argument(
        '--duration-s',
        type=float,
        help=(
            'Run load for this many seconds instead of a fixed request count.'
        ),
    )
    parser.add_argument(
        '--mode',
        choices=('read',),
        default='read',
        help='Built-in query mix to use when --query is omitted.',
    )
    parser.add_argument(
        '--timeout-s',
        type=float,
        default=10.0,
        help=(
            'Client and server timeout per request in seconds. Defaults to 10.'
        ),
    )
    parser.add_argument(
        '--wait-service-timeout-s',
        type=float,
        default=10.0,
        help='Seconds to wait for the query service. Defaults to 10.',
    )
    parser.add_argument(
        '--output',
        help='Optional JSON output path.',
    )
    return parser


def _latency_percentile(
    sorted_latencies: Sequence[float],
    rank: float,
) -> float:
    if not sorted_latencies:
        raise ValueError('cannot compute percentile for empty sequence')
    if len(sorted_latencies) == 1:
        return sorted_latencies[0]

    position = (len(sorted_latencies) - 1) * rank
    lower_index = int(position)
    upper_index = min(lower_index + 1, len(sorted_latencies) - 1)
    fraction = position - lower_index
    lower_value = sorted_latencies[lower_index]
    upper_value = sorted_latencies[upper_index]
    return lower_value + ((upper_value - lower_value) * fraction)


def summarize_records(records: list[RequestRecord]) -> dict[str, Any]:
    """Build summary metrics from request records."""
    latencies = sorted(record.latency_s for record in records)
    successes = sum(1 for record in records if record.success)
    timeouts = sum(1 for record in records if record.timed_out)
    failures = len(records) - successes

    summary: dict[str, Any] = {
        'total_requests': len(records),
        'successes': successes,
        'failures': failures,
        'timeouts': timeouts,
        'latency_s': {},
    }
    if latencies:
        summary['latency_s'] = {
            'min': min(latencies),
            'mean': statistics.fmean(latencies),
            'p50': _latency_percentile(latencies, 0.50),
            'p90': _latency_percentile(latencies, 0.90),
            'p95': _latency_percentile(latencies, 0.95),
            'p99': _latency_percentile(latencies, 0.99),
            'max': max(latencies),
        }
    return summary


def write_results(
    output_path: Path,
    *,
    service_name: str,
    query_type: str | None,
    query: str | None,
    timeout_s: float,
    records: list[RequestRecord],
    clients: int = 1,
    duration_s: float | None = None,
    mode: str = 'read',
    query_mix: Sequence[QuerySpec] | None = None,
) -> None:
    """Write experiment results to a JSON file."""
    payload = {
        'service_name': service_name,
        'mode': mode,
        'clients': clients,
        'duration_s': duration_s,
        'query_type': query_type,
        'query': query,
        'query_mix': [
            asdict(query_spec) for query_spec in query_mix or ()
        ],
        'timeout_s': timeout_s,
        'summary': summarize_records(records),
        'requests': [asdict(record) for record in records],
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )


def print_summary(summary: dict[str, Any]) -> None:
    """Print compact experiment summary."""
    latency = summary['latency_s']
    print('ros_typedb stress experiment summary')
    print(f"  total_requests: {summary['total_requests']}")
    print(f"  successes: {summary['successes']}")
    print(f"  failures: {summary['failures']}")
    print(f"  timeouts: {summary['timeouts']}")
    if latency:
        print(f"  latency_min_s: {latency['min']:.6f}")
        print(f"  latency_mean_s: {latency['mean']:.6f}")
        print(f"  latency_p50_s: {latency['p50']:.6f}")
        print(f"  latency_p90_s: {latency['p90']:.6f}")
        print(f"  latency_p95_s: {latency['p95']:.6f}")
        print(f"  latency_p99_s: {latency['p99']:.6f}")
        print(f"  latency_max_s: {latency['max']:.6f}")


def _build_query_request(
    query: str,
    query_type: str,
    timeout_s: float,
) -> Query.Request:
    request = Query.Request()
    request.query = query
    request.query_type = QUERY_TYPE_BY_NAME[query_type]
    request.timeout_s = timeout_s
    return request


def _build_query_specs(args: argparse.Namespace) -> list[QuerySpec]:
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


@dataclass
class _PendingRequest:
    client_id: int
    index: int
    query_spec: QuerySpec
    future: Any
    started_at_s: float


def _record_from_pending(
    pending: _PendingRequest,
    *,
    ended_at_s: float,
    success: bool,
    timed_out: bool,
    error_message: str,
    result_count: int,
) -> RequestRecord:
    return RequestRecord(
        index=pending.index,
        client_id=pending.client_id,
        started_at_s=pending.started_at_s,
        ended_at_s=ended_at_s,
        latency_s=ended_at_s - pending.started_at_s,
        success=success,
        timed_out=timed_out,
        error_message=error_message,
        result_count=result_count,
    )


def run_experiment(args: argparse.Namespace) -> list[RequestRecord]:
    """Run the stress experiment."""
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
    query_specs = _build_query_specs(args)

    node = rclpy.create_node('ros_typedb_stress_experiment')
    clients = [
        node.create_client(Query, args.service_name)
        for _ in range(args.clients)
    ]
    records: list[RequestRecord] = []
    pending_by_client: dict[int, _PendingRequest] = {}
    next_index = 0
    deadline_s = None
    if args.duration_s is not None:
        deadline_s = time.monotonic() + args.duration_s

    def should_send_request() -> bool:
        if deadline_s is not None:
            return time.monotonic() < deadline_s
        return next_index < args.requests

    def start_request(client_id: int) -> None:
        nonlocal next_index
        query_spec = query_specs[next_index % len(query_specs)]
        request = _build_query_request(
            query_spec.query,
            query_spec.query_type,
            args.timeout_s,
        )
        started_at_s = time.monotonic()
        future = clients[client_id].call_async(request)
        pending_by_client[client_id] = _PendingRequest(
            client_id=client_id,
            index=next_index,
            query_spec=query_spec,
            future=future,
            started_at_s=started_at_s,
        )
        next_index += 1

    try:
        service_ready = clients[0].wait_for_service(
            timeout_sec=args.wait_service_timeout_s
        )
        if not service_ready:
            raise RuntimeError(
                f'Query service is not available: {args.service_name}'
            )

        while should_send_request() or pending_by_client:
            for client_id in range(args.clients):
                if client_id in pending_by_client:
                    continue
                if should_send_request():
                    start_request(client_id)

            rclpy.spin_once(node, timeout_sec=0.01)
            now_s = time.monotonic()
            completed_client_ids = []

            for client_id, pending in pending_by_client.items():
                future = pending.future
                if not future.done():
                    if now_s - pending.started_at_s < args.timeout_s:
                        continue
                    records.append(
                        _record_from_pending(
                            pending,
                            ended_at_s=now_s,
                            success=False,
                            timed_out=True,
                            error_message='client request timed out',
                            result_count=0,
                        )
                    )
                    completed_client_ids.append(client_id)
                    continue

                exception = future.exception()
                if exception is not None:
                    records.append(
                        _record_from_pending(
                            pending,
                            ended_at_s=now_s,
                            success=False,
                            timed_out=False,
                            error_message=str(exception),
                            result_count=0,
                        )
                    )
                    completed_client_ids.append(client_id)
                    continue

                response = future.result()
                records.append(
                    _record_from_pending(
                        pending,
                        ended_at_s=now_s,
                        success=response.success,
                        timed_out=False,
                        error_message=response.error_message,
                        result_count=len(response.results),
                    )
                )
                completed_client_ids.append(client_id)

            for client_id in completed_client_ids:
                del pending_by_client[client_id]
    finally:
        node.destroy_node()
    return records


def main(argv: list[str] | None = None) -> int:
    """Run the stress experiment CLI."""
    parser = build_argument_parser()
    args = parser.parse_args(argv)

    rclpy.init(args=None)
    try:
        records = run_experiment(args)
        summary = summarize_records(records)
        print_summary(summary)
        if args.output:
            query_specs = _build_query_specs(args)
            write_results(
                Path(args.output),
                service_name=args.service_name,
                query_type=args.query_type,
                query=args.query,
                timeout_s=args.timeout_s,
                records=records,
                clients=args.clients,
                duration_s=args.duration_s,
                mode=args.mode,
                query_mix=query_specs,
            )
        return 0 if summary['failures'] == 0 else 1
    except (RuntimeError, ValueError, OSError) as exc:
        print(f'ros_typedb stress experiment failed: {exc}', file=sys.stderr)
        return 1
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
