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
from pathlib import Path
import sys

import rclpy

from ros_typedb_tools.stress_config import INVARIANT_PROFILE_NAMES
from ros_typedb_tools.stress_config import MIXED_PROFILE_NAMES
from ros_typedb_tools.stress_config import QUERY_TYPE_BY_NAME
from ros_typedb_tools.stress_config import StressExperimentResult
from ros_typedb_tools.stress_output import default_timeout_output_path
from ros_typedb_tools.stress_output import print_summary
from ros_typedb_tools.stress_output import summarize_invariant_records
from ros_typedb_tools.stress_output import summarize_records
from ros_typedb_tools.stress_output import write_results
from ros_typedb_tools.stress_output import write_timeout_results
from ros_typedb_tools.stress_runner import run_experiment


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
        choices=('read', 'mixed'),
        default='read',
        help='Built-in query mix to use when --query is omitted.',
    )
    parser.add_argument(
        '--mixed-profile',
        choices=MIXED_PROFILE_NAMES,
        default='auto',
        help=(
            'Schema-specific mixed-mode profile. auto selects plan-schema '
            'when --invariant-profile is plan-schema or plan-schema-mixed; '
            'otherwise it selects test-data.'
        ),
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
        '--invariant-profile',
        choices=INVARIANT_PROFILE_NAMES,
        default='none',
        help=(
            'Optional built-in invariant profile. Use test-data with the '
            'bundled ros_typedb test schema/data. Defaults to none.'
        ),
    )
    parser.add_argument(
        '--invariant-period-s',
        type=float,
        default=10.0,
        help=(
            'Seconds between periodic invariant checks. Use 0 to run only '
            'the final check. Defaults to 10.'
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
    parser.add_argument(
        '--fault',
        choices=('none', 'delete-database'),
        default='none',
        help=(
            'Fault to inject during the experiment. '
            'Defaults to none.'
        ),
    )
    parser.add_argument(
        '--fault-at-s',
        type=float,
        help=(
            'Seconds into the experiment at which to trigger the fault. '
            'Required when --fault is not none.'
        ),
    )
    parser.add_argument(
        '--fault-recovery-timeout-s',
        type=float,
        default=30.0,
        help=(
            'Seconds to wait for invariants to pass after a fault. '
            'Defaults to 30.'
        ),
    )
    parser.add_argument(
        '--delete-database-service-name',
        help=(
            'delete_database service name. Defaults to the --service-name '
            'prefix with /delete_database appended '
            '(e.g. /ros_typedb_interface/delete_database).'
        ),
    )
    return parser


def _write_requested_outputs(
    args: argparse.Namespace,
    result: StressExperimentResult,
) -> None:
    output_path = None
    if args.output:
        output_path = Path(args.output).expanduser()
        write_results(
            output_path,
            service_name=args.service_name,
            query_type=args.query_type,
            query=args.query,
            timeout_s=args.timeout_s,
            records=result.records,
            clients=args.clients,
            duration_s=args.duration_s,
            mode=args.mode,
            mixed_profile=(
                result.config.mixed_profile_name or args.mixed_profile
            ),
            query_mix=result.config.query_specs,
            request_gap_s=args.request_gap_s,
            max_in_flight=result.config.max_in_flight,
            executor=args.executor,
            invariant_profile=result.config.invariant_profile_name,
            invariant_period_s=args.invariant_period_s,
            invariant_records=result.invariant_records,
            fault_result=result.fault_result,
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
            records=result.records,
        )
        print(f'  timeout_output: {timeout_output_path}')


def main(argv: list[str] | None = None) -> int:
    """Run the stress experiment CLI."""
    parser = build_argument_parser()
    args = parser.parse_args(argv)

    rclpy.init(args=None)
    try:
        result = run_experiment(args)
        summary = summarize_records(result.records)
        invariant_summary = summarize_invariant_records(
            result.invariant_records
        )
        print_summary(summary, invariant_summary, result.fault_result)
        _write_requested_outputs(args, result)
        fault_result = result.fault_result
        if (
            fault_result.fault != 'none'
            and fault_result.fault_triggered_at_s is not None
            and fault_result.fault_recovered_at_s is None
        ):
            return 1
        if invariant_summary['failures'] > 0:
            return 1
        if args.fail_on_failure and summary['failures'] > 0:
            return 1
        return 0
    except (RuntimeError, ValueError, OSError) as exc:
        print(f'ros_typedb stress experiment failed: {exc}', file=sys.stderr)
        return 1
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
