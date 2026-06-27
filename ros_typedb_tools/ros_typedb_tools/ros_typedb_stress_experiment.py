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
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import SingleThreadedExecutor

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
        '--request-gap-s',
        type=float,
        default=0.0,
        help=(
            'Minimum delay between requests from the same client. Defaults '
            'to 0.'
        ),
    )
    parser.add_argument(
        '--max-in-flight',
        type=int,
        help=(
            'Maximum number of outstanding requests across all clients. '
            'Defaults to the number of clients.'
        ),
    )
    parser.add_argument(
        '--executor',
        choices=('global', 'single', 'multi'),
        default='global',
        help=(
            'Client-side executor used for spinning. Defaults to global, '
            'matching rclpy.spin_once(node).'
        ),
    )
    parser.add_argument(
        '--executor-threads',
        type=int,
        default=2,
        help='Thread count for --executor multi. Defaults to 2.',
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
    parser.add_argument(
        '--timeout-output',
        help=(
            'Optional JSON path for timeout-only records. Defaults to '
            '<output-stem>_timeouts.json when --output is set.'
        ),
    )
    parser.add_argument(
        '--debug-events-output',
        help='Optional JSONL path for request/future/executor debug events.',
    )
    parser.add_argument(
        '--fail-on-failure',
        action='store_true',
        help=(
            'Exit with status 1 when any request fails. By default, request '
            'failures are reported in the summary and JSON output without '
            'making the tool command fail.'
        ),
    )
    return parser


def build_fake_service_argument_parser() -> argparse.ArgumentParser:
    """Build command-line arguments for the fake Query service."""
    parser = argparse.ArgumentParser(
        prog='ros_typedb_fake_query_service',
        description='Serve the ros_typedb Query service without TypeDB work.',
    )
    parser.add_argument(
        '--service-name',
        default='/ros_typedb_interface/query',
        help='Query service name. Defaults to /ros_typedb_interface/query.',
    )
    parser.add_argument(
        '--response-delay-s',
        type=float,
        default=0.0,
        help='Optional delay before each response. Defaults to 0.',
    )
    parser.add_argument(
        '--executor',
        choices=('single', 'multi'),
        default='single',
        help='Server executor. Defaults to single.',
    )
    parser.add_argument(
        '--executor-threads',
        type=int,
        default=2,
        help='Thread count for --executor multi. Defaults to 2.',
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
    request_gap_s: float = 0.0,
    max_in_flight: int | None = None,
    executor: str = 'global',
) -> None:
    """Write experiment results to a JSON file."""
    payload = {
        'service_name': service_name,
        'mode': mode,
        'clients': clients,
        'duration_s': duration_s,
        'request_gap_s': request_gap_s,
        'max_in_flight': max_in_flight,
        'executor': executor,
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


def default_timeout_output_path(output_path: Path) -> Path:
    """Build the default timeout-only output path for a result path."""
    suffix = output_path.suffix or '.json'
    stem = output_path.stem if output_path.suffix else output_path.name
    return output_path.with_name(f'{stem}_timeouts{suffix}')


def write_timeout_results(
    output_path: Path,
    *,
    source_output_path: Path | None,
    records: list[RequestRecord],
) -> None:
    """Write timeout-only request records to a JSON file."""
    timeout_records = [
        record
        for record in records
        if record.timed_out
    ]
    payload = {
        'source_output_path': (
            str(source_output_path) if source_output_path is not None else None
        ),
        'summary': {
            'timeouts': len(timeout_records),
            'total_requests': len(records),
        },
        'timeouts': [asdict(record) for record in timeout_records],
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )


class DebugEventWriter:
    """Write compact JSONL events for diagnosing client-side scheduling."""

    def __init__(self, output_path: Path | None) -> None:
        """Create an event writer for the optional output path."""
        self._output_path = output_path
        self._stream = None

    def __enter__(self) -> 'DebugEventWriter':
        """Open the event stream when debug output is enabled."""
        if self._output_path is not None:
            self._output_path.parent.mkdir(parents=True, exist_ok=True)
            self._stream = self._output_path.open('w', encoding='utf-8')
        return self

    def __exit__(self, *args: object) -> None:
        """Close the event stream when it was opened."""
        if self._stream is not None:
            self._stream.close()

    def log(self, event: str, **fields: Any) -> None:
        """Append one debug event if debug output is enabled."""
        if self._stream is None:
            return
        payload = {
            'event': event,
            'monotonic_s': time.monotonic(),
            **fields,
        }
        self._stream.write(json.dumps(payload, sort_keys=True) + '\n')


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
    query_index: int
    query_spec: QuerySpec
    future: Any
    started_at_s: float


def _build_executor(args: argparse.Namespace) -> Any | None:
    if args.executor == 'global':
        return None
    if args.executor == 'single':
        return SingleThreadedExecutor()
    if args.executor == 'multi':
        return MultiThreadedExecutor(num_threads=args.executor_threads)
    raise ValueError(f'unsupported executor: {args.executor}')


def _spin_once(node: Any, executor: Any | None) -> None:
    if executor is None:
        rclpy.spin_once(node, timeout_sec=0.01)
    else:
        executor.spin_once(timeout_sec=0.01)


def _record_from_pending(
    pending: _PendingRequest,
    *,
    ended_at_s: float,
    success: bool,
    timed_out: bool,
    cancel_requested: bool,
    error_message: str,
    result_count: int,
) -> RequestRecord:
    return RequestRecord(
        index=pending.index,
        client_id=pending.client_id,
        query_index=pending.query_index,
        query_type=pending.query_spec.query_type,
        query=pending.query_spec.query,
        started_at_s=pending.started_at_s,
        ended_at_s=ended_at_s,
        latency_s=ended_at_s - pending.started_at_s,
        success=success,
        timed_out=timed_out,
        cancel_requested=cancel_requested,
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
    if args.request_gap_s < 0:
        raise ValueError(
            '--request-gap-s must be greater than or equal to zero'
        )
    if args.max_in_flight is not None and args.max_in_flight < 1:
        raise ValueError('--max-in-flight must be greater than zero')
    if args.executor_threads < 1:
        raise ValueError('--executor-threads must be greater than zero')
    query_specs = _build_query_specs(args)
    max_in_flight = args.max_in_flight or args.clients
    debug_events_output = (
        Path(args.debug_events_output).expanduser()
        if args.debug_events_output
        else None
    )

    node = rclpy.create_node('ros_typedb_stress_experiment')
    executor = _build_executor(args)
    if executor is not None:
        executor.add_node(node)
    clients = [
        node.create_client(Query, args.service_name)
        for _ in range(args.clients)
    ]
    records: list[RequestRecord] = []
    pending_by_client: dict[int, _PendingRequest] = {}
    last_started_by_client = [float('-inf') for _ in range(args.clients)]
    next_index = 0
    deadline_s = None
    if args.duration_s is not None:
        deadline_s = time.monotonic() + args.duration_s
    next_snapshot_s = time.monotonic() + 1.0

    def should_send_request() -> bool:
        if deadline_s is not None:
            return time.monotonic() < deadline_s
        return next_index < args.requests

    def start_request(client_id: int, started_at_s: float) -> None:
        nonlocal next_index
        query_index = next_index % len(query_specs)
        query_spec = query_specs[query_index]
        request = _build_query_request(
            query_spec.query,
            query_spec.query_type,
            args.timeout_s,
        )
        future = clients[client_id].call_async(request)
        pending_by_client[client_id] = _PendingRequest(
            client_id=client_id,
            index=next_index,
            query_index=query_index,
            query_spec=query_spec,
            future=future,
            started_at_s=started_at_s,
        )
        last_started_by_client[client_id] = started_at_s
        next_index += 1

    try:
        with DebugEventWriter(debug_events_output) as debug_events:
            debug_events.log(
                'experiment_started',
                clients=args.clients,
                executor=args.executor,
                max_in_flight=max_in_flight,
                request_gap_s=args.request_gap_s,
            )
            wait_deadline_s = time.monotonic() + args.wait_service_timeout_s
            for client_id, client in enumerate(clients):
                wait_timeout_s = max(0.0, wait_deadline_s - time.monotonic())
                service_ready = client.wait_for_service(
                    timeout_sec=wait_timeout_s
                )
                debug_events.log(
                    'service_wait_finished',
                    client_id=client_id,
                    ready=service_ready,
                )
                if not service_ready:
                    raise RuntimeError(
                        f'Query service is not available: {args.service_name}'
                    )

            while should_send_request() or pending_by_client:
                spin_started_s = time.monotonic()
                _spin_once(node, executor)
                now_s = time.monotonic()
                debug_events.log(
                    'spin_once',
                    duration_s=now_s - spin_started_s,
                    pending_count=len(pending_by_client),
                )
                if now_s >= next_snapshot_s:
                    debug_events.log(
                        'pending_snapshot',
                        next_index=next_index,
                        pending_count=len(pending_by_client),
                        pending_indices=[
                            pending.index
                            for pending in pending_by_client.values()
                        ],
                    )
                    next_snapshot_s = now_s + 1.0

                completed_client_ids = []
                timed_out_this_cycle = False

                for client_id, pending in pending_by_client.items():
                    future = pending.future
                    if not future.done():
                        if now_s - pending.started_at_s < args.timeout_s:
                            continue
                        timed_out_this_cycle = True
                        future.cancel()
                        records.append(
                            _record_from_pending(
                                pending,
                                ended_at_s=now_s,
                                success=False,
                                timed_out=True,
                                cancel_requested=True,
                                error_message='client request timed out',
                                result_count=0,
                            )
                        )
                        debug_events.log(
                            'request_timeout',
                            client_id=client_id,
                            index=pending.index,
                            latency_s=now_s - pending.started_at_s,
                            pending_count=len(pending_by_client),
                            query_index=pending.query_index,
                        )
                        completed_client_ids.append(client_id)
                        continue

                    debug_events.log(
                        'future_done',
                        client_id=client_id,
                        index=pending.index,
                        latency_s=now_s - pending.started_at_s,
                        pending_count=len(pending_by_client),
                        query_index=pending.query_index,
                    )
                    exception = future.exception()
                    if exception is not None:
                        records.append(
                            _record_from_pending(
                                pending,
                                ended_at_s=now_s,
                                success=False,
                                timed_out=False,
                                cancel_requested=False,
                                error_message=str(exception),
                                result_count=0,
                            )
                        )
                        debug_events.log(
                            'request_exception',
                            client_id=client_id,
                            error_message=str(exception),
                            index=pending.index,
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
                            cancel_requested=False,
                            error_message=response.error_message,
                            result_count=len(response.results),
                        )
                    )
                    debug_events.log(
                        'request_complete',
                        client_id=client_id,
                        index=pending.index,
                        result_count=len(response.results),
                        success=response.success,
                    )
                    completed_client_ids.append(client_id)

                for client_id in completed_client_ids:
                    del pending_by_client[client_id]

                if timed_out_this_cycle:
                    debug_events.log(
                        'timeout_barrier',
                        pending_count=len(pending_by_client),
                    )
                    continue

                now_s = time.monotonic()
                for client_id in range(args.clients):
                    if len(pending_by_client) >= max_in_flight:
                        break
                    if client_id in pending_by_client:
                        continue
                    if (
                        args.request_gap_s > 0
                        and now_s - last_started_by_client[client_id]
                        < args.request_gap_s
                    ):
                        continue
                    if should_send_request():
                        start_request(client_id, now_s)
                        pending = pending_by_client[client_id]
                        debug_events.log(
                            'request_sent',
                            client_id=client_id,
                            index=pending.index,
                            pending_count=len(pending_by_client),
                            query_index=pending.query_index,
                            query_type=pending.query_spec.query_type,
                        )
            debug_events.log(
                'experiment_finished',
                records=len(records),
                timeouts=sum(1 for record in records if record.timed_out),
            )
    finally:
        if executor is not None:
            executor.remove_node(node)
            executor.shutdown()
        node.destroy_node()
    return records


def run_fake_query_service(args: argparse.Namespace) -> None:
    """Run an immediate-response Query service for ROS client diagnostics."""
    if args.response_delay_s < 0:
        raise ValueError(
            '--response-delay-s must be greater than or equal to zero'
        )
    if args.executor_threads < 1:
        raise ValueError('--executor-threads must be greater than zero')

    node = rclpy.create_node('ros_typedb_fake_query_service')
    executor = (
        MultiThreadedExecutor(num_threads=args.executor_threads)
        if args.executor == 'multi'
        else SingleThreadedExecutor()
    )
    executor.add_node(node)

    def handle_query(
        request: Query.Request,
        response: Query.Response,
    ) -> Query.Response:
        if args.response_delay_s > 0:
            time.sleep(args.response_delay_s)
        response.success = True
        response.error_message = ''
        response.results = []
        return response

    service = node.create_service(Query, args.service_name, handle_query)
    node.get_logger().info(
        f'Fake Query service ready on {args.service_name}'
    )
    try:
        executor.spin()
    finally:
        node.destroy_service(service)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()


def main(argv: list[str] | None = None) -> int:
    """Run the stress experiment CLI."""
    parser = build_argument_parser()
    args = parser.parse_args(argv)

    rclpy.init(args=None)
    try:
        records = run_experiment(args)
        summary = summarize_records(records)
        print_summary(summary)
        output_path = None
        if args.output:
            query_specs = _build_query_specs(args)
            output_path = Path(args.output).expanduser()
            write_results(
                output_path,
                service_name=args.service_name,
                query_type=args.query_type,
                query=args.query,
                timeout_s=args.timeout_s,
                records=records,
                clients=args.clients,
                duration_s=args.duration_s,
                mode=args.mode,
                query_mix=query_specs,
                request_gap_s=args.request_gap_s,
                max_in_flight=args.max_in_flight,
                executor=args.executor,
            )
            print(f'  output: {output_path}')
        timeout_output_path = None
        if args.timeout_output:
            timeout_output_path = Path(args.timeout_output).expanduser()
        elif output_path is not None:
            timeout_output_path = default_timeout_output_path(output_path)
        if timeout_output_path is not None:
            write_timeout_results(
                timeout_output_path,
                source_output_path=output_path,
                records=records,
            )
            print(f'  timeout_output: {timeout_output_path}')
        if args.fail_on_failure and summary['failures'] > 0:
            return 1
        return 0
    except (RuntimeError, ValueError, OSError) as exc:
        print(f'ros_typedb stress experiment failed: {exc}', file=sys.stderr)
        return 1
    finally:
        rclpy.shutdown()


def fake_query_service_main(argv: list[str] | None = None) -> int:
    """Run the fake Query service CLI."""
    parser = build_fake_service_argument_parser()
    args = parser.parse_args(argv)

    rclpy.init(args=None)
    try:
        run_fake_query_service(args)
        return 0
    except (RuntimeError, ValueError, OSError, KeyboardInterrupt) as exc:
        if isinstance(exc, KeyboardInterrupt):
            return 0
        print(f'ros_typedb fake query service failed: {exc}', file=sys.stderr)
        return 1
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
