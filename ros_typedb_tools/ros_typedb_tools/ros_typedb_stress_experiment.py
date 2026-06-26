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


@dataclass(frozen=True)
class RequestRecord:
    """Result of one service request."""

    index: int
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
        description='Run a fixed-count stress experiment against ros_typedb.',
    )
    parser.add_argument(
        '--service-name',
        default='/ros_typedb_interface/query',
        help='Query service name. Defaults to /ros_typedb_interface/query.',
    )
    parser.add_argument(
        '--query',
        required=True,
        help='TypeDB query to send on each request.',
    )
    parser.add_argument(
        '--query-type',
        required=True,
        choices=sorted(QUERY_TYPE_BY_NAME),
        help='TypeDB query type.',
    )
    parser.add_argument(
        '--requests',
        type=int,
        default=1,
        help='Number of requests to send. Defaults to 1.',
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


def summarize_records(records: list[RequestRecord]) -> dict[str, Any]:
    """Build summary metrics from request records."""
    latencies = [record.latency_s for record in records]
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
            'max': max(latencies),
        }
    return summary


def write_results(
    output_path: Path,
    *,
    service_name: str,
    query_type: str,
    query: str,
    timeout_s: float,
    records: list[RequestRecord],
) -> None:
    """Write experiment results to a JSON file."""
    payload = {
        'service_name': service_name,
        'query_type': query_type,
        'query': query,
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


def run_experiment(args: argparse.Namespace) -> list[RequestRecord]:
    """Run the fixed-count stress experiment."""
    if args.requests < 1:
        raise ValueError('--requests must be greater than zero')
    if args.timeout_s <= 0:
        raise ValueError('--timeout-s must be greater than zero')
    if args.wait_service_timeout_s <= 0:
        raise ValueError('--wait-service-timeout-s must be greater than zero')

    node = rclpy.create_node('ros_typedb_stress_experiment')
    client = node.create_client(Query, args.service_name)
    records: list[RequestRecord] = []
    try:
        service_ready = client.wait_for_service(
            timeout_sec=args.wait_service_timeout_s
        )
        if not service_ready:
            raise RuntimeError(
                f'Query service is not available: {args.service_name}'
            )

        for index in range(args.requests):
            request = _build_query_request(
                args.query,
                args.query_type,
                args.timeout_s,
            )
            started_at_s = time.monotonic()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(
                node,
                future,
                timeout_sec=args.timeout_s,
            )
            ended_at_s = time.monotonic()
            latency_s = ended_at_s - started_at_s

            if not future.done():
                records.append(
                    RequestRecord(
                        index=index,
                        started_at_s=started_at_s,
                        ended_at_s=ended_at_s,
                        latency_s=latency_s,
                        success=False,
                        timed_out=True,
                        error_message='client request timed out',
                        result_count=0,
                    )
                )
                continue

            exception = future.exception()
            if exception is not None:
                records.append(
                    RequestRecord(
                        index=index,
                        started_at_s=started_at_s,
                        ended_at_s=ended_at_s,
                        latency_s=latency_s,
                        success=False,
                        timed_out=False,
                        error_message=str(exception),
                        result_count=0,
                    )
                )
                continue

            response = future.result()
            records.append(
                RequestRecord(
                    index=index,
                    started_at_s=started_at_s,
                    ended_at_s=ended_at_s,
                    latency_s=latency_s,
                    success=response.success,
                    timed_out=False,
                    error_message=response.error_message,
                    result_count=len(response.results),
                )
            )
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
            write_results(
                Path(args.output),
                service_name=args.service_name,
                query_type=args.query_type,
                query=args.query,
                timeout_s=args.timeout_s,
                records=records,
            )
        return 0 if summary['failures'] == 0 else 1
    except (RuntimeError, ValueError, OSError) as exc:
        print(f'ros_typedb stress experiment failed: {exc}', file=sys.stderr)
        return 1
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
