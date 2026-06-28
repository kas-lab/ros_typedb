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
"""Output helpers for ros_typedb stress experiments."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import asdict
import json
from pathlib import Path
import statistics
import threading
import time
from typing import Any

from ros_typedb_tools.stress_config import FaultResult
from ros_typedb_tools.stress_config import InvariantRecord
from ros_typedb_tools.stress_config import QuerySpec
from ros_typedb_tools.stress_config import RequestRecord


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


def summarize_invariant_records(
    records: list[InvariantRecord],
) -> dict[str, Any]:
    """Build summary metrics from invariant records."""
    successes = sum(1 for record in records if record.success)
    timeouts = sum(1 for record in records if record.timed_out)
    failures = len(records) - successes
    return {
        'total_invariants': len(records),
        'successes': successes,
        'failures': failures,
        'timeouts': timeouts,
    }


def _fault_payload(fault_result: FaultResult | None) -> dict[str, Any]:
    """Build a stable JSON object for fault result fields."""
    if fault_result is None:
        fault_result = FaultResult(
            fault='none',
            fault_at_s=None,
            fault_triggered_at_s=None,
            fault_delete_success=None,
            fault_delete_error=None,
            fault_delete_latency_s=None,
            fault_recovered_at_s=None,
            fault_recovery_s=None,
        )
    return {
        'fault': fault_result.fault,
        'fault_at_s': fault_result.fault_at_s,
        'fault_triggered_at_s': fault_result.fault_triggered_at_s,
        'fault_delete_success': fault_result.fault_delete_success,
        'fault_delete_error': fault_result.fault_delete_error,
        'fault_delete_latency_s': fault_result.fault_delete_latency_s,
        'fault_recovered_at_s': fault_result.fault_recovered_at_s,
        'fault_recovery_s': fault_result.fault_recovery_s,
        'fault_restart_stop_success': (
            fault_result.fault_restart_stop_success
        ),
        'fault_restart_stop_error': fault_result.fault_restart_stop_error,
        'fault_restart_stop_latency_s': (
            fault_result.fault_restart_stop_latency_s
        ),
        'fault_restart_start_success': (
            fault_result.fault_restart_start_success
        ),
        'fault_restart_start_error': fault_result.fault_restart_start_error,
        'fault_restart_start_latency_s': (
            fault_result.fault_restart_start_latency_s
        ),
        'fault_restart_delay_s': fault_result.fault_restart_delay_s,
        'fault_restart_outage_s': fault_result.fault_restart_outage_s,
        'fault_observed_outage_s': fault_result.fault_observed_outage_s,
    }


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
    mixed_profile: str = 'auto',
    query_mix: Sequence[QuerySpec] | None = None,
    request_gap_s: float = 0.0,
    max_in_flight: int | None = None,
    executor: str = 'global',
    invariant_profile: str = 'none',
    invariant_period_s: float = 10.0,
    invariant_records: list[InvariantRecord] | None = None,
    fault_result: FaultResult | None = None,
) -> None:
    """Write experiment results to a JSON file."""
    invariants = invariant_records or []
    payload = {
        'service_name': service_name,
        'mode': mode,
        'mixed_profile': mixed_profile,
        'clients': clients,
        'duration_s': duration_s,
        'request_gap_s': request_gap_s,
        'max_in_flight': max_in_flight,
        'executor': executor,
        'invariant_profile': invariant_profile,
        'invariant_period_s': invariant_period_s,
        'query_type': query_type,
        'query': query,
        'query_mix': [
            asdict(query_spec) for query_spec in query_mix or ()
        ],
        'timeout_s': timeout_s,
        'summary': summarize_records(records),
        'invariant_summary': summarize_invariant_records(invariants),
        'fault': _fault_payload(fault_result),
        'requests': [asdict(record) for record in records],
        'invariants': [asdict(record) for record in invariants],
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
        self._lock = threading.Lock()

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
        with self._lock:
            self._stream.write(json.dumps(payload, sort_keys=True) + '\n')


def print_summary(
    summary: dict[str, Any],
    invariant_summary: dict[str, Any] | None = None,
    fault_result: FaultResult | None = None,
) -> None:
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
    if invariant_summary is not None:
        print(f"  invariants_total: {invariant_summary['total_invariants']}")
        print(f"  invariants_successes: {invariant_summary['successes']}")
        print(f"  invariants_failures: {invariant_summary['failures']}")
        print(f"  invariants_timeouts: {invariant_summary['timeouts']}")
    if fault_result is not None and fault_result.fault != 'none':
        triggered_at = (
            f'{fault_result.fault_triggered_at_s:.2f}s'
            if fault_result.fault_triggered_at_s is not None
            else 'not triggered'
        )
        delete_latency = (
            f'{fault_result.fault_delete_latency_s:.3f}s'
            if fault_result.fault_delete_latency_s is not None
            else 'n/a'
        )
        delete_status = (
            'ok' if fault_result.fault_delete_success else 'FAILED'
        )
        recovery_str = (
            f'{fault_result.fault_recovery_s:.2f}s'
            if fault_result.fault_recovery_s is not None
            else 'not recovered'
        )
        if fault_result.fault == 'restart-typedb':
            restart_status = (
                'ok'
                if (
                    fault_result.fault_restart_stop_success
                    and fault_result.fault_restart_start_success
                )
                else 'FAILED'
            )
            restart_outage = (
                f'{fault_result.fault_restart_outage_s:.3f}s'
                if fault_result.fault_restart_outage_s is not None
                else 'n/a'
            )
            observed_outage = (
                f'{fault_result.fault_observed_outage_s:.3f}s'
                if fault_result.fault_observed_outage_s is not None
                else 'n/a'
            )
            print(
                f'  fault: {fault_result.fault} triggered at '
                f'{triggered_at} | restart: {restart_status} '
                f'({restart_outage}) | observed outage: '
                f'{observed_outage} | recovery: {recovery_str}'
            )
            return
        print(
            f'  fault: {fault_result.fault} triggered at {triggered_at} | '
            f'delete: {delete_status} ({delete_latency}) | '
            f'recovery: {recovery_str}'
        )
