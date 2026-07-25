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
"""ROS service-client runner for the ros_typedb stress experiment."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import shlex
import subprocess
import threading
import time
from typing import Any
import uuid

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import SingleThreadedExecutor

from lifecycle_msgs.msg import State
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.srv import GetState
from ros_typedb_msgs.srv import Query

from ros_typedb_benchmark.stress_config import build_query_request
from ros_typedb_benchmark.stress_config import DEFAULT_TYPEDB_START_COMMAND
from ros_typedb_benchmark.stress_config import DEFAULT_TYPEDB_STOP_COMMAND
from ros_typedb_benchmark.stress_config import FaultResult
from ros_typedb_benchmark.stress_config import InvariantRecord
from ros_typedb_benchmark.stress_config import InvariantSpec
from ros_typedb_benchmark.stress_config import QuerySpec
from ros_typedb_benchmark.stress_config import RequestRecord
from ros_typedb_benchmark.stress_config import StressExperimentResult
from ros_typedb_benchmark.stress_invariants import build_timeout_invariant_record
from ros_typedb_benchmark.stress_invariants import evaluate_invariant_response
from ros_typedb_benchmark.stress_mixed import build_mixed_cleanup_query_spec
from ros_typedb_benchmark.stress_mixed import build_mixed_query_spec
from ros_typedb_benchmark.stress_mixed import MixedQueryProfile
from ros_typedb_benchmark.stress_output import DebugEventWriter
from ros_typedb_benchmark.stress_resolution import resolve_stress_config
from std_srvs.srv import Empty


@dataclass
class _PendingRequest:
    client_id: int
    index: int
    query_index: int
    query_spec: QuerySpec
    future: Any
    started_at_s: float


@dataclass
class _RunState:
    records: list[RequestRecord]
    invariant_records: list[InvariantRecord]
    pending_by_client: dict[int, _PendingRequest]
    last_started_by_client: list[float]
    sent_count_by_client: list[int]
    mixed_cleanup_keys: set[str]
    mixed_profile: MixedQueryProfile | None
    mixed_run_id: str
    next_index: int
    next_invariant_index: int
    deadline_s: float | None
    next_snapshot_s: float
    next_invariant_s: float
    fault_triggered: bool = False
    fault_triggered_at_s: float | None = None
    fault_delete_success: bool | None = None
    fault_delete_error: str | None = None
    fault_delete_latency_s: float | None = None
    fault_recovered_at_s: float | None = None
    fault_restart_stop_success: bool | None = None
    fault_restart_stop_error: str | None = None
    fault_restart_stop_latency_s: float | None = None
    fault_restart_start_success: bool | None = None
    fault_restart_start_error: str | None = None
    fault_restart_start_latency_s: float | None = None
    fault_restart_started_at_s: float | None = None
    fault_restart_ended_at_s: float | None = None
    fault_restart_delay_s: float | None = None
    fault_lifecycle_deactivate_success: bool | None = None
    fault_lifecycle_deactivate_error: str | None = None
    fault_lifecycle_deactivate_latency_s: float | None = None
    fault_lifecycle_cleanup_success: bool | None = None
    fault_lifecycle_cleanup_error: str | None = None
    fault_lifecycle_cleanup_latency_s: float | None = None
    fault_lifecycle_configure_success: bool | None = None
    fault_lifecycle_configure_error: str | None = None
    fault_lifecycle_configure_latency_s: float | None = None
    fault_lifecycle_activate_success: bool | None = None
    fault_lifecycle_activate_error: str | None = None
    fault_lifecycle_activate_latency_s: float | None = None
    fault_lifecycle_reactivate: bool | None = None


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


def _should_send_request(
    state: _RunState,
    *,
    requests: int,
) -> bool:
    if state.deadline_s is not None:
        return time.monotonic() < state.deadline_s
    return state.next_index < requests


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


def _wait_for_all_clients(
    clients: list[Any],
    *,
    service_name: str,
    wait_service_timeout_s: float,
    debug_events: DebugEventWriter,
) -> None:
    wait_deadline_s = time.monotonic() + wait_service_timeout_s
    for client_id, client in enumerate(clients):
        wait_timeout_s = max(0.0, wait_deadline_s - time.monotonic())
        service_ready = client.wait_for_service(timeout_sec=wait_timeout_s)
        debug_events.log(
            'service_wait_finished',
            client_id=client_id,
            ready=service_ready,
        )
        if not service_ready:
            raise RuntimeError(
                f'Query service is not available: {service_name}'
            )


def _start_request(
    *,
    client_id: int,
    clients: list[Any],
    query_specs: list[QuerySpec],
    state: _RunState,
    timeout_s: float,
    started_at_s: float,
    use_mixed_queries: bool,
    mixed_profile: MixedQueryProfile | None,
    mixed_run_id: str,
) -> _PendingRequest:
    if use_mixed_queries:
        if mixed_profile is None:
            raise RuntimeError('mixed profile is required for mixed mode')
        query_index = state.sent_count_by_client[client_id] % len(
            mixed_profile.templates
        )
        query_spec = build_mixed_query_spec(
            profile=mixed_profile,
            run_id=mixed_run_id,
            client_id=client_id,
            client_request_index=state.sent_count_by_client[client_id],
        )
    else:
        query_index = state.next_index % len(query_specs)
        query_spec = query_specs[query_index]
    request = build_query_request(
        query_spec.query,
        query_spec.query_type,
        timeout_s,
    )
    future = clients[client_id].call_async(request)
    pending = _PendingRequest(
        client_id=client_id,
        index=state.next_index,
        query_index=query_index,
        query_spec=query_spec,
        future=future,
        started_at_s=started_at_s,
    )
    state.pending_by_client[client_id] = pending
    state.last_started_by_client[client_id] = started_at_s
    state.sent_count_by_client[client_id] += 1
    # Mixed workloads create experiment-owned temporary concepts. Track keys
    # until a matching delete succeeds. Final cleanup removes leftovers.
    if (
        query_spec.cleanup_key is not None
        and query_spec.query_type != 'delete'
    ):
        state.mixed_cleanup_keys.add(query_spec.cleanup_key)
    state.next_index += 1
    return pending


def _send_available_requests(
    *,
    args: argparse.Namespace,
    clients: list[Any],
    query_specs: list[QuerySpec],
    state: _RunState,
    max_in_flight: int,
    debug_events: DebugEventWriter,
) -> None:
    now_s = time.monotonic()
    for client_id in range(args.clients):
        if len(state.pending_by_client) >= max_in_flight:
            break
        if client_id in state.pending_by_client:
            continue
        if (
            args.request_gap_s > 0
            and now_s - state.last_started_by_client[client_id]
            < args.request_gap_s
        ):
            continue
        if not _should_send_request(state, requests=args.requests):
            continue
        pending = _start_request(
            client_id=client_id,
            clients=clients,
            query_specs=query_specs,
            state=state,
            timeout_s=args.timeout_s,
            started_at_s=now_s,
            use_mixed_queries=args.mode == 'mixed' and args.query is None,
            mixed_profile=state.mixed_profile,
            mixed_run_id=state.mixed_run_id,
        )
        debug_events.log(
            'request_sent',
            client_id=client_id,
            index=pending.index,
            pending_count=len(state.pending_by_client),
            query_index=pending.query_index,
            query_type=pending.query_spec.query_type,
        )


def _log_pending_snapshot(
    *,
    state: _RunState,
    now_s: float,
    debug_events: DebugEventWriter,
) -> None:
    if now_s < state.next_snapshot_s:
        return
    debug_events.log(
        'pending_snapshot',
        next_index=state.next_index,
        pending_count=len(state.pending_by_client),
        pending_indices=[
            pending.index
            for pending in state.pending_by_client.values()
        ],
    )
    state.next_snapshot_s = now_s + 1.0


def _record_timeout(
    *,
    pending: _PendingRequest,
    now_s: float,
    pending_count: int,
    state: _RunState,
    debug_events: DebugEventWriter,
) -> None:
    pending.future.cancel()
    state.records.append(
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
        client_id=pending.client_id,
        index=pending.index,
        latency_s=now_s - pending.started_at_s,
        pending_count=pending_count,
        query_index=pending.query_index,
    )


def _record_done_future(
    *,
    pending: _PendingRequest,
    now_s: float,
    pending_count: int,
    state: _RunState,
    debug_events: DebugEventWriter,
) -> None:
    debug_events.log(
        'future_done',
        client_id=pending.client_id,
        index=pending.index,
        latency_s=now_s - pending.started_at_s,
        pending_count=pending_count,
        query_index=pending.query_index,
    )
    exception = pending.future.exception()
    if exception is not None:
        state.records.append(
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
            client_id=pending.client_id,
            error_message=str(exception),
            index=pending.index,
        )
        return

    response = pending.future.result()
    if (
        response.success
        and pending.query_spec.cleanup_key is not None
        and pending.query_spec.query_type == 'delete'
    ):
        state.mixed_cleanup_keys.discard(pending.query_spec.cleanup_key)
    state.records.append(
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
        client_id=pending.client_id,
        index=pending.index,
        result_count=len(response.results),
        success=response.success,
    )


def _process_pending_requests(
    *,
    args: argparse.Namespace,
    state: _RunState,
    now_s: float,
    debug_events: DebugEventWriter,
) -> bool:
    completed_client_ids = []
    timed_out_this_cycle = False
    pending_count = len(state.pending_by_client)

    for client_id, pending in state.pending_by_client.items():
        future = pending.future
        if not future.done():
            if now_s - pending.started_at_s < args.timeout_s:
                continue
            timed_out_this_cycle = True
            _record_timeout(
                pending=pending,
                now_s=now_s,
                pending_count=pending_count,
                state=state,
                debug_events=debug_events,
            )
            completed_client_ids.append(client_id)
            continue

        _record_done_future(
            pending=pending,
            now_s=now_s,
            pending_count=pending_count,
            state=state,
            debug_events=debug_events,
        )
        completed_client_ids.append(client_id)

    for client_id in completed_client_ids:
        del state.pending_by_client[client_id]

    if timed_out_this_cycle:
        debug_events.log(
            'timeout_barrier',
            pending_count=len(state.pending_by_client),
        )
    return timed_out_this_cycle


def _spin_until_future_done(
    future: Any,
    node: Any,
    executor: Any | None,
    timeout_s: float,
    started_at_s: float,
) -> tuple[bool, float]:
    """
    Spin until future completes or timeout expires.

    Returns (timed_out, ended_at_s). Cancels future on timeout.
    """
    while True:
        _spin_once(node, executor)
        now_s = time.monotonic()
        if future.done():
            return False, now_s
        if now_s - started_at_s >= timeout_s:
            future.cancel()
            return True, now_s


def _run_invariant_check(
    *,
    node: Any,
    executor: Any | None,
    client: Any,
    spec: InvariantSpec,
    index: int,
    phase: str,
    timeout_s: float,
    debug_events: DebugEventWriter,
) -> InvariantRecord:
    started_at_s = time.monotonic()
    request = build_query_request(
        spec.query,
        spec.query_type,
        timeout_s,
    )
    future = client.call_async(request)
    debug_events.log(
        'invariant_sent',
        index=index,
        name=spec.name,
        phase=phase,
    )

    timed_out, now_s = _spin_until_future_done(
        future, node, executor, timeout_s, started_at_s
    )
    if timed_out:
        record = build_timeout_invariant_record(
            index=index,
            phase=phase,
            spec=spec,
            started_at_s=started_at_s,
        )
        debug_events.log(
            'invariant_timeout',
            index=index,
            name=spec.name,
            phase=phase,
            latency_s=record.latency_s,
        )
        return record
    exception = future.exception()
    if exception is not None:
        record = evaluate_invariant_response(
            index=index,
            phase=phase,
            spec=spec,
            started_at_s=started_at_s,
            ended_at_s=now_s,
            response=None,
            exception_message=str(exception),
        )
    else:
        record = evaluate_invariant_response(
            index=index,
            phase=phase,
            spec=spec,
            started_at_s=started_at_s,
            ended_at_s=now_s,
            response=future.result(),
        )
    debug_events.log(
        'invariant_complete',
        index=index,
        name=spec.name,
        phase=phase,
        success=record.success,
        latency_s=record.latency_s,
        error_message=record.error_message,
    )
    return record


def _run_invariant_profile(
    *,
    node: Any,
    executor: Any | None,
    client: Any,
    specs: list[InvariantSpec],
    state: _RunState,
    phase: str,
    timeout_s: float,
    debug_events: DebugEventWriter,
) -> list[InvariantRecord]:
    records: list[InvariantRecord] = []
    for spec in specs:
        record = _run_invariant_check(
            node=node,
            executor=executor,
            client=client,
            spec=spec,
            index=state.next_invariant_index,
            phase=phase,
            timeout_s=timeout_s,
            debug_events=debug_events,
        )
        state.invariant_records.append(record)
        state.next_invariant_index += 1
        records.append(record)
    return records


def _mark_fault_recovered_from_invariant_pass(
    *,
    state: _RunState,
    records: list[InvariantRecord],
    debug_events: DebugEventWriter,
) -> bool:
    """Record fault recovery when one full invariant pass succeeds."""
    if state.fault_triggered_at_s is None:
        return False
    if state.fault_recovered_at_s is not None:
        return False
    if not records or not all(record.success for record in records):
        return False

    state.fault_recovered_at_s = time.monotonic()
    debug_events.log(
        'recovery_complete',
        recovery_s=state.fault_recovered_at_s - state.fault_triggered_at_s,
    )
    return True


def _run_cleanup_request(
    *,
    node: Any,
    executor: Any | None,
    client: Any,
    state: _RunState,
    query_spec: QuerySpec,
    timeout_s: float,
    debug_events: DebugEventWriter,
) -> None:
    started_at_s = time.monotonic()
    request = build_query_request(
        query_spec.query,
        query_spec.query_type,
        timeout_s,
    )
    future = client.call_async(request)
    pending = _PendingRequest(
        client_id=-1,
        index=state.next_index,
        query_index=-1,
        query_spec=query_spec,
        future=future,
        started_at_s=started_at_s,
    )
    state.next_index += 1
    debug_events.log(
        'cleanup_sent',
        index=pending.index,
        cleanup_key=query_spec.cleanup_key,
    )

    timed_out, now_s = _spin_until_future_done(
        future, node, executor, timeout_s, started_at_s
    )
    if timed_out:
        _record_timeout(
            pending=pending,
            now_s=now_s,
            pending_count=1,
            state=state,
            debug_events=debug_events,
        )
        debug_events.log(
            'cleanup_timeout',
            index=pending.index,
            cleanup_key=query_spec.cleanup_key,
        )
        return
    _record_done_future(
        pending=pending,
        now_s=now_s,
        pending_count=1,
        state=state,
        debug_events=debug_events,
    )
    debug_events.log(
        'cleanup_complete',
        index=pending.index,
        cleanup_key=query_spec.cleanup_key,
    )
    return


def _run_oneshot_query(
    *,
    node: Any,
    executor: Any | None,
    client: Any,
    state: _RunState,
    query_spec: QuerySpec,
    timeout_s: float,
    phase: str,
    debug_events: DebugEventWriter,
) -> None:
    """Send one synchronous query and record the result.

    Used for setup and teardown queries that run outside the main load loop.
    """
    started_at_s = time.monotonic()
    request = build_query_request(
        query_spec.query,
        query_spec.query_type,
        timeout_s,
    )
    future = client.call_async(request)
    pending = _PendingRequest(
        client_id=-1,
        index=state.next_index,
        query_index=-1,
        query_spec=query_spec,
        future=future,
        started_at_s=started_at_s,
    )
    state.next_index += 1
    debug_events.log(
        'oneshot_query_sent',
        phase=phase,
        index=pending.index,
        query_type=query_spec.query_type,
    )

    timed_out, now_s = _spin_until_future_done(
        future, node, executor, timeout_s, started_at_s
    )
    if timed_out:
        _record_timeout(
            pending=pending,
            now_s=now_s,
            pending_count=1,
            state=state,
            debug_events=debug_events,
        )
        debug_events.log(
            'oneshot_query_timeout',
            phase=phase,
            index=pending.index,
        )
        return
    _record_done_future(
        pending=pending,
        now_s=now_s,
        pending_count=1,
        state=state,
        debug_events=debug_events,
    )
    debug_events.log(
        'oneshot_query_complete',
        phase=phase,
        index=pending.index,
    )


def _run_phase_queries(
    *,
    node: Any,
    executor: Any | None,
    client: Any,
    specs: list[QuerySpec],
    state: _RunState,
    phase: str,
    timeout_s: float,
    debug_events: DebugEventWriter,
) -> None:
    """Run a list of oneshot queries sequentially for setup or teardown."""
    if not specs:
        return
    debug_events.log('phase_queries_started', phase=phase, count=len(specs))
    for spec in specs:
        _run_oneshot_query(
            node=node,
            executor=executor,
            client=client,
            state=state,
            query_spec=spec,
            timeout_s=timeout_s,
            phase=phase,
            debug_events=debug_events,
        )
    debug_events.log('phase_queries_finished', phase=phase)


def _cleanup_mixed_data(
    *,
    args: argparse.Namespace,
    node: Any,
    executor: Any | None,
    cleanup_client: Any | None,
    mixed_profile: MixedQueryProfile | None,
    state: _RunState,
    debug_events: DebugEventWriter,
) -> None:
    if args.mode != 'mixed' or args.query is not None:
        return
    if (
        cleanup_client is None
        or mixed_profile is None
        or not state.mixed_cleanup_keys
    ):
        return

    debug_events.log(
        'cleanup_started',
        cleanup_keys=len(state.mixed_cleanup_keys),
    )
    for key in sorted(state.mixed_cleanup_keys):
        _run_cleanup_request(
            node=node,
            executor=executor,
            client=cleanup_client,
            state=state,
            query_spec=build_mixed_cleanup_query_spec(mixed_profile, key),
            timeout_s=args.timeout_s,
            debug_events=debug_events,
        )
    debug_events.log(
        'cleanup_finished',
        cleanup_keys=len(state.mixed_cleanup_keys),
    )


def _derive_delete_db_service_name(service_name: str) -> str:
    """Derive delete_database service name from the query service name."""
    parts = service_name.rstrip('/').rsplit('/', 1)
    prefix = parts[0] if len(parts) > 1 else ''
    return prefix + '/delete_database'


def _derive_lifecycle_change_state_service_name(service_name: str) -> str:
    """Derive lifecycle change_state service name from the query service."""
    parts = service_name.rstrip('/').rsplit('/', 1)
    prefix = parts[0] if len(parts) > 1 else ''
    return prefix + '/change_state'


def _derive_lifecycle_get_state_service_name(service_name: str) -> str:
    """Derive lifecycle get_state service name from the query service."""
    parts = service_name.rstrip('/').rsplit('/', 1)
    prefix = parts[0] if len(parts) > 1 else ''
    return prefix + '/get_state'


def _maybe_run_periodic_invariants(
    *,
    args: argparse.Namespace,
    node: Any,
    executor: Any | None,
    invariant_client: Any | None,
    invariant_specs: list[InvariantSpec],
    state: _RunState,
    now_s: float,
    debug_events: DebugEventWriter,
) -> None:
    if not invariant_specs or invariant_client is None:
        return
    if args.invariant_period_s == 0:
        return
    if now_s < state.next_invariant_s:
        return

    records = _run_invariant_profile(
        node=node,
        executor=executor,
        client=invariant_client,
        specs=invariant_specs,
        state=state,
        phase='periodic',
        timeout_s=args.timeout_s,
        debug_events=debug_events,
    )
    _mark_fault_recovered_from_invariant_pass(
        state=state,
        records=records,
        debug_events=debug_events,
    )
    state.next_invariant_s = time.monotonic() + args.invariant_period_s


def _trigger_delete_database_fault(
    *,
    node: Any,
    executor: Any | None,
    delete_db_client: Any,
    state: _RunState,
    timeout_s: float,
    debug_events: DebugEventWriter,
) -> None:
    """Call the delete_database service and record fault timing."""
    started_at_s = time.monotonic()
    pending_count = len(state.pending_by_client)
    state.fault_triggered = True
    state.fault_triggered_at_s = started_at_s
    debug_events.log(
        'fault_triggered',
        fault='delete-database',
        triggered_at_s=started_at_s,
        pending_count_at_fault=pending_count,
    )
    request = Empty.Request()
    future = delete_db_client.call_async(request)
    timed_out, ended_at_s = _spin_until_future_done(
        future, node, executor, timeout_s, started_at_s
    )
    state.fault_delete_latency_s = ended_at_s - started_at_s
    if timed_out:
        state.fault_delete_success = False
        state.fault_delete_error = (
            f'delete_database service timed out after {timeout_s:g}s'
        )
        debug_events.log(
            'fault_delete_timeout',
            latency_s=state.fault_delete_latency_s,
        )
    else:
        exc = future.exception()
        if exc is not None:
            state.fault_delete_success = False
            state.fault_delete_error = str(exc)
            debug_events.log(
                'fault_delete_error',
                error=state.fault_delete_error,
                latency_s=state.fault_delete_latency_s,
            )
        else:
            state.fault_delete_success = True
            debug_events.log(
                'fault_delete_complete',
                latency_s=state.fault_delete_latency_s,
            )


def _build_typedb_restart_commands(
    *,
    typedb_container: str | None,
    typedb_stop_command: str | None,
    typedb_start_command: str | None,
) -> tuple[list[str], list[str]]:
    """Build stop/start command argv for a TypeDB restart fault."""
    if typedb_container:
        return (
            ['docker', 'stop', typedb_container],
            ['docker', 'start', typedb_container],
        )
    return (
        shlex.split(typedb_stop_command or DEFAULT_TYPEDB_STOP_COMMAND),
        shlex.split(typedb_start_command or DEFAULT_TYPEDB_START_COMMAND),
    )


def _run_fault_command(
    command: list[str],
    *,
    timeout_s: float,
) -> tuple[bool, str | None, float]:
    """Run one TypeDB restart command and return success, error, latency."""
    started_at_s = time.monotonic()
    try:
        completed = subprocess.run(
            command,
            capture_output=True,
            check=False,
            text=True,
            timeout=timeout_s,
        )
    except subprocess.TimeoutExpired:
        return (
            False,
            f'command timed out after {timeout_s:g}s: {command!r}',
            time.monotonic() - started_at_s,
        )
    except Exception as exc:  # noqa: B902
        return False, str(exc), time.monotonic() - started_at_s

    if completed.returncode == 0:
        return True, None, time.monotonic() - started_at_s
    error = completed.stderr.strip() or completed.stdout.strip()
    if not error:
        error = f'command exited with status {completed.returncode}'
    return False, error, time.monotonic() - started_at_s


def _start_fault_command(
    command: list[str],
    *,
    observation_s: float = 1.0,
) -> tuple[bool, str | None, float]:
    """Start a TypeDB process command that may keep running in foreground."""
    started_at_s = time.monotonic()
    try:
        process = subprocess.Popen(  # noqa: S603
            command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception as exc:  # noqa: B902
        return False, str(exc), time.monotonic() - started_at_s

    try:
        returncode = process.wait(timeout=observation_s)
    except subprocess.TimeoutExpired:
        return True, None, time.monotonic() - started_at_s

    if returncode == 0:
        return True, None, time.monotonic() - started_at_s
    return (
        False,
        f'command exited with status {returncode}',
        time.monotonic() - started_at_s,
    )


def _restart_typedb_fault_worker(
    *,
    stop_command: list[str],
    start_command: list[str],
    start_command_waits_for_exit: bool,
    restart_delay_s: float,
    command_timeout_s: float,
    state: _RunState,
    debug_events: DebugEventWriter,
) -> None:
    """Stop and restart TypeDB while the main experiment loop continues."""
    state.fault_restart_started_at_s = time.monotonic()
    state.fault_restart_delay_s = restart_delay_s
    debug_events.log(
        'fault_restart_stop_started',
        command=stop_command,
        command_timeout_s=command_timeout_s,
    )
    stop_success, stop_error, stop_latency_s = _run_fault_command(
        stop_command,
        timeout_s=command_timeout_s,
    )
    state.fault_restart_stop_success = stop_success
    state.fault_restart_stop_error = stop_error
    state.fault_restart_stop_latency_s = stop_latency_s
    debug_events.log(
        'fault_restart_stop_complete',
        success=stop_success,
        error=stop_error,
        latency_s=stop_latency_s,
    )

    if restart_delay_s > 0:
        time.sleep(restart_delay_s)

    debug_events.log(
        'fault_restart_start_started',
        command=start_command,
        command_timeout_s=command_timeout_s,
    )
    if start_command_waits_for_exit:
        start_success, start_error, start_latency_s = _run_fault_command(
            start_command,
            timeout_s=command_timeout_s,
        )
    else:
        start_success, start_error, start_latency_s = _start_fault_command(
            start_command,
        )
    state.fault_restart_start_success = start_success
    state.fault_restart_start_error = start_error
    state.fault_restart_start_latency_s = start_latency_s
    state.fault_restart_ended_at_s = time.monotonic()
    debug_events.log(
        'fault_restart_start_complete',
        success=start_success,
        error=start_error,
        latency_s=start_latency_s,
        controller_outage_s=(
            state.fault_restart_ended_at_s
            - state.fault_restart_started_at_s
        ),
    )


def _trigger_restart_typedb_fault(
    *,
    config: Any,
    state: _RunState,
    debug_events: DebugEventWriter,
) -> threading.Thread:
    """Start a background TypeDB restart fault controller."""
    started_at_s = time.monotonic()
    pending_count = len(state.pending_by_client)
    state.fault_triggered = True
    state.fault_triggered_at_s = started_at_s
    stop_command, start_command = _build_typedb_restart_commands(
        typedb_container=config.typedb_container,
        typedb_stop_command=config.typedb_stop_command,
        typedb_start_command=config.typedb_start_command,
    )
    start_command_waits_for_exit = config.typedb_container is not None
    debug_events.log(
        'fault_triggered',
        fault='restart-typedb',
        triggered_at_s=started_at_s,
        pending_count_at_fault=pending_count,
        stop_command=stop_command,
        start_command=start_command,
    )
    thread = threading.Thread(
        target=_restart_typedb_fault_worker,
        kwargs={
            'stop_command': stop_command,
            'start_command': start_command,
            'start_command_waits_for_exit': start_command_waits_for_exit,
            'restart_delay_s': config.typedb_restart_delay_s,
            'command_timeout_s': config.fault_command_timeout_s,
            'state': state,
            'debug_events': debug_events,
        },
        daemon=True,
    )
    thread.start()
    return thread


def _call_lifecycle_transition(
    *,
    client: Any,
    transition_id: int,
    transition_name: str,
    timeout_s: float,
) -> tuple[bool, str | None, float]:
    """Request one lifecycle transition and wait for the main loop to spin."""
    started_at_s = time.monotonic()
    request = ChangeState.Request()
    request.transition.id = transition_id
    future = client.call_async(request)
    deadline_s = started_at_s + timeout_s

    while not future.done():
        if time.monotonic() >= deadline_s:
            return (
                False,
                (
                    f'lifecycle {transition_name} transition timed out '
                    f'after {timeout_s:g}s'
                ),
                time.monotonic() - started_at_s,
            )
        time.sleep(0.01)

    latency_s = time.monotonic() - started_at_s
    exc = future.exception()
    if exc is not None:
        return False, str(exc), latency_s

    result = future.result()
    if not getattr(result, 'success', False):
        return (
            False,
            f'lifecycle {transition_name} transition rejected',
            latency_s,
        )
    return True, None, latency_s


def _call_lifecycle_get_state(
    *,
    client: Any,
    timeout_s: float,
) -> tuple[bool, str | None, float, int | None, str | None]:
    """Request the current lifecycle state."""
    started_at_s = time.monotonic()
    future = client.call_async(GetState.Request())
    deadline_s = started_at_s + timeout_s

    while not future.done():
        if time.monotonic() >= deadline_s:
            return (
                False,
                f'lifecycle get_state timed out after {timeout_s:g}s',
                time.monotonic() - started_at_s,
                None,
                None,
            )
        time.sleep(0.01)

    latency_s = time.monotonic() - started_at_s
    exc = future.exception()
    if exc is not None:
        return False, str(exc), latency_s, None, None

    current_state = future.result().current_state
    return (
        True,
        None,
        latency_s,
        current_state.id,
        current_state.label,
    )


def _wait_for_lifecycle_state(
    *,
    client: Any,
    expected_state_id: int,
    expected_state_label: str,
    timeout_s: float,
    debug_events: DebugEventWriter,
) -> tuple[bool, str | None, float]:
    """Poll lifecycle get_state until the expected primary state is reached."""
    started_at_s = time.monotonic()
    deadline_s = started_at_s + timeout_s
    last_state_id = None
    last_state_label = None
    last_error = None

    while time.monotonic() < deadline_s:
        remaining_s = max(0.0, deadline_s - time.monotonic())
        success, error, _, state_id, state_label = _call_lifecycle_get_state(
            client=client,
            timeout_s=min(1.0, remaining_s),
        )
        debug_events.log(
            'fault_lifecycle_state_observed',
            expected_state=expected_state_label,
            state_id=state_id,
            state_label=state_label,
            success=success,
            error=error,
        )
        if success and state_id == expected_state_id:
            return True, None, time.monotonic() - started_at_s
        last_state_id = state_id
        last_state_label = state_label
        last_error = error
        time.sleep(0.05)

    if last_error:
        detail = last_error
    else:
        detail = f'last state was {last_state_label} ({last_state_id})'
    return (
        False,
        (
            f'lifecycle did not reach {expected_state_label} after '
            f'{timeout_s:g}s; {detail}'
        ),
        time.monotonic() - started_at_s,
    )


def _record_lifecycle_transition_result(
    *,
    state: _RunState,
    transition_name: str,
    success: bool,
    error: str | None,
    latency_s: float,
) -> None:
    """Store one lifecycle transition result on the run state."""
    setattr(state, f'fault_lifecycle_{transition_name}_success', success)
    setattr(state, f'fault_lifecycle_{transition_name}_error', error)
    setattr(
        state,
        f'fault_lifecycle_{transition_name}_latency_s',
        latency_s,
    )


def _lifecycle_cleanup_fault_worker(
    *,
    lifecycle_change_state_client: Any,
    lifecycle_get_state_client: Any,
    reactivate: bool,
    transition_timeout_s: float,
    state: _RunState,
    debug_events: DebugEventWriter,
) -> None:
    """Cleanup and optionally reactivate the lifecycle node during load."""
    state.fault_lifecycle_reactivate = reactivate
    transitions = [
        (
            'deactivate',
            Transition.TRANSITION_DEACTIVATE,
            State.PRIMARY_STATE_INACTIVE,
            'inactive',
        ),
        (
            'cleanup',
            Transition.TRANSITION_CLEANUP,
            State.PRIMARY_STATE_UNCONFIGURED,
            'unconfigured',
        ),
    ]
    if reactivate:
        transitions.extend([
            (
                'configure',
                Transition.TRANSITION_CONFIGURE,
                State.PRIMARY_STATE_INACTIVE,
                'inactive',
            ),
            (
                'activate',
                Transition.TRANSITION_ACTIVATE,
                State.PRIMARY_STATE_ACTIVE,
                'active',
            ),
        ])

    for (
        transition_name,
        transition_id,
        expected_state_id,
        expected_state_label,
    ) in transitions:
        debug_events.log(
            'fault_lifecycle_transition_started',
            transition=transition_name,
            transition_id=transition_id,
            expected_state=expected_state_label,
            transition_timeout_s=transition_timeout_s,
        )
        success, error, latency_s = _call_lifecycle_transition(
            client=lifecycle_change_state_client,
            transition_id=transition_id,
            transition_name=transition_name,
            timeout_s=transition_timeout_s,
        )
        if success:
            state_success, state_error, state_latency_s = (
                _wait_for_lifecycle_state(
                    client=lifecycle_get_state_client,
                    expected_state_id=expected_state_id,
                    expected_state_label=expected_state_label,
                    timeout_s=transition_timeout_s,
                    debug_events=debug_events,
                )
            )
            success = state_success
            error = state_error
            latency_s += state_latency_s
        _record_lifecycle_transition_result(
            state=state,
            transition_name=transition_name,
            success=success,
            error=error,
            latency_s=latency_s,
        )
        debug_events.log(
            'fault_lifecycle_transition_complete',
            transition=transition_name,
            success=success,
            error=error,
            latency_s=latency_s,
        )
        if not success:
            return


def _trigger_lifecycle_cleanup_fault(
    *,
    lifecycle_change_state_client: Any,
    lifecycle_get_state_client: Any,
    config: Any,
    state: _RunState,
    debug_events: DebugEventWriter,
) -> threading.Thread:
    """Start a background lifecycle cleanup fault controller."""
    started_at_s = time.monotonic()
    pending_count = len(state.pending_by_client)
    state.fault_triggered = True
    state.fault_triggered_at_s = started_at_s
    state.fault_lifecycle_reactivate = config.lifecycle_reactivate
    debug_events.log(
        'fault_triggered',
        fault='lifecycle-cleanup',
        triggered_at_s=started_at_s,
        pending_count_at_fault=pending_count,
        lifecycle_reactivate=config.lifecycle_reactivate,
    )
    thread = threading.Thread(
        target=_lifecycle_cleanup_fault_worker,
        kwargs={
            'lifecycle_change_state_client': lifecycle_change_state_client,
            'lifecycle_get_state_client': lifecycle_get_state_client,
            'reactivate': config.lifecycle_reactivate,
            'transition_timeout_s': config.lifecycle_transition_timeout_s,
            'state': state,
            'debug_events': debug_events,
        },
        daemon=True,
    )
    thread.start()
    return thread


def _compute_observed_fault_outage_s(
    records: list[RequestRecord],
    *,
    fault_triggered_at_s: float | None,
) -> float | None:
    """Measure observed outage from request failures and later success."""
    if fault_triggered_at_s is None:
        return None
    first_failure_ended_at_s = None
    for record in sorted(records, key=lambda item: item.ended_at_s):
        if record.ended_at_s < fault_triggered_at_s:
            continue
        if first_failure_ended_at_s is None:
            if not record.success:
                first_failure_ended_at_s = record.ended_at_s
            continue
        if record.success:
            return record.ended_at_s - first_failure_ended_at_s
    return None


def _wait_for_recovery(
    *,
    node: Any,
    executor: Any | None,
    client: Any,
    specs: list[InvariantSpec],
    state: _RunState,
    recovery_timeout_s: float,
    invariant_timeout_s: float,
    debug_events: DebugEventWriter,
) -> None:
    """Poll invariants until all pass or recovery_timeout_s expires."""
    if not specs or state.fault_triggered_at_s is None:
        return
    if state.fault_recovered_at_s is not None:
        return
    recovery_deadline_s = state.fault_triggered_at_s + recovery_timeout_s
    debug_events.log('recovery_started', recovery_timeout_s=recovery_timeout_s)
    while time.monotonic() < recovery_deadline_s:
        records = _run_invariant_profile(
            node=node,
            executor=executor,
            client=client,
            specs=specs,
            state=state,
            phase='recovery',
            timeout_s=invariant_timeout_s,
            debug_events=debug_events,
        )
        if _mark_fault_recovered_from_invariant_pass(
            state=state,
            records=records,
            debug_events=debug_events,
        ):
            return
    debug_events.log('recovery_timeout', recovery_timeout_s=recovery_timeout_s)


def run_experiment(args: argparse.Namespace) -> StressExperimentResult:
    """Run the stress experiment."""
    config = resolve_stress_config(args)

    node = rclpy.create_node('ros_typedb_stress_experiment')
    executor = _build_executor(args)
    if executor is not None:
        executor.add_node(node)
    clients = [
        node.create_client(Query, args.service_name)
        for _ in range(args.clients)
    ]
    invariant_client = (
        node.create_client(Query, args.service_name)
        if config.invariant_specs
        else None
    )
    cleanup_client = (
        node.create_client(Query, args.service_name)
        if config.mixed_profile is not None
        else None
    )
    delete_db_service_name = (
        args.delete_database_service_name
        if getattr(args, 'delete_database_service_name', None)
        else _derive_delete_db_service_name(args.service_name)
    )
    delete_db_client = (
        node.create_client(Empty, delete_db_service_name)
        if getattr(args, 'fault', 'none') == 'delete-database'
        else None
    )
    lifecycle_change_state_service_name = (
        config.lifecycle_change_state_service_name
        if config.lifecycle_change_state_service_name
        else _derive_lifecycle_change_state_service_name(args.service_name)
    )
    lifecycle_get_state_service_name = (
        config.lifecycle_get_state_service_name
        if config.lifecycle_get_state_service_name
        else _derive_lifecycle_get_state_service_name(args.service_name)
    )
    lifecycle_change_state_client = (
        node.create_client(ChangeState, lifecycle_change_state_service_name)
        if config.fault == 'lifecycle-cleanup'
        else None
    )
    lifecycle_get_state_client = (
        node.create_client(GetState, lifecycle_get_state_service_name)
        if config.fault == 'lifecycle-cleanup'
        else None
    )
    setup_teardown_client = (
        node.create_client(Query, args.service_name)
        if config.setup_specs or config.teardown_specs
        else None
    )
    deadline_s = (
        time.monotonic() + args.duration_s
        if args.duration_s is not None
        else None
    )
    state = _RunState(
        records=[],
        invariant_records=[],
        pending_by_client={},
        last_started_by_client=[float('-inf') for _ in range(args.clients)],
        sent_count_by_client=[0 for _ in range(args.clients)],
        mixed_cleanup_keys=set(),
        mixed_profile=config.mixed_profile,
        mixed_run_id=uuid.uuid4().hex[:12],
        next_index=0,
        next_invariant_index=0,
        deadline_s=deadline_s,
        next_snapshot_s=time.monotonic() + 1.0,
        next_invariant_s=time.monotonic() + args.invariant_period_s,
    )
    restart_fault_thread = None
    lifecycle_fault_thread = None

    try:
        with DebugEventWriter(config.debug_events_output) as debug_events:
            debug_events.log(
                'experiment_started',
                clients=args.clients,
                executor=args.executor,
                max_in_flight=config.max_in_flight,
                request_gap_s=args.request_gap_s,
                invariant_profile=config.invariant_profile_name,
                invariant_period_s=args.invariant_period_s,
                mixed_profile=config.mixed_profile_name,
            )
            _wait_for_all_clients(
                clients
                + ([invariant_client] if invariant_client else [])
                + ([cleanup_client] if cleanup_client else [])
                + ([delete_db_client] if delete_db_client is not None else [])
                + (
                    [lifecycle_change_state_client]
                    if lifecycle_change_state_client is not None
                    else []
                )
                + (
                    [lifecycle_get_state_client]
                    if lifecycle_get_state_client is not None
                    else []
                )
                + (
                    [setup_teardown_client]
                    if setup_teardown_client is not None
                    else []
                ),
                service_name=args.service_name,
                wait_service_timeout_s=args.wait_service_timeout_s,
                debug_events=debug_events,
            )
            if setup_teardown_client is not None and config.setup_specs:
                _run_phase_queries(
                    node=node,
                    executor=executor,
                    client=setup_teardown_client,
                    specs=config.setup_specs,
                    state=state,
                    phase='setup',
                    timeout_s=args.timeout_s,
                    debug_events=debug_events,
                )
            experiment_started_at_s = time.monotonic()

            while (
                _should_send_request(state, requests=args.requests)
                or state.pending_by_client
            ):
                spin_started_s = time.monotonic()
                _spin_once(node, executor)
                now_s = time.monotonic()
                debug_events.log(
                    'spin_once',
                    duration_s=now_s - spin_started_s,
                    pending_count=len(state.pending_by_client),
                )
                _log_pending_snapshot(
                    state=state,
                    now_s=now_s,
                    debug_events=debug_events,
                )

                timed_out_this_cycle = _process_pending_requests(
                    args=args,
                    state=state,
                    now_s=now_s,
                    debug_events=debug_events,
                )
                if timed_out_this_cycle:
                    continue

                _send_available_requests(
                    args=args,
                    clients=clients,
                    query_specs=config.query_specs,
                    state=state,
                    max_in_flight=config.max_in_flight,
                    debug_events=debug_events,
                )
                _maybe_run_periodic_invariants(
                    args=args,
                    node=node,
                    executor=executor,
                    invariant_client=invariant_client,
                    invariant_specs=config.invariant_specs,
                    state=state,
                    now_s=now_s,
                    debug_events=debug_events,
                )
                if (
                    config.fault == 'delete-database'
                    and delete_db_client is not None
                    and not state.fault_triggered
                    and config.fault_at_s is not None
                    and time.monotonic() - experiment_started_at_s
                    >= config.fault_at_s
                ):
                    _trigger_delete_database_fault(
                        node=node,
                        executor=executor,
                        delete_db_client=delete_db_client,
                        state=state,
                        timeout_s=args.timeout_s,
                        debug_events=debug_events,
                    )
                if (
                    config.fault == 'restart-typedb'
                    and not state.fault_triggered
                    and config.fault_at_s is not None
                    and time.monotonic() - experiment_started_at_s
                    >= config.fault_at_s
                ):
                    restart_fault_thread = _trigger_restart_typedb_fault(
                        config=config,
                        state=state,
                        debug_events=debug_events,
                    )
                if (
                    config.fault == 'lifecycle-cleanup'
                    and lifecycle_change_state_client is not None
                    and lifecycle_get_state_client is not None
                    and not state.fault_triggered
                    and config.fault_at_s is not None
                    and time.monotonic() - experiment_started_at_s
                    >= config.fault_at_s
                ):
                    lifecycle_fault_thread = _trigger_lifecycle_cleanup_fault(
                        lifecycle_change_state_client=(
                            lifecycle_change_state_client
                        ),
                        lifecycle_get_state_client=lifecycle_get_state_client,
                        config=config,
                        state=state,
                        debug_events=debug_events,
                    )

            if restart_fault_thread is not None:
                restart_fault_thread.join()
            if lifecycle_fault_thread is not None:
                lifecycle_fault_thread.join()

            # 1. Recovery must come before cleanup. Cleanup queries trigger DB
            #    recreation, silently advancing recovery unobserved.
            if (
                state.fault_triggered
                and config.invariant_specs
                and invariant_client is not None
            ):
                _wait_for_recovery(
                    node=node,
                    executor=executor,
                    client=invariant_client,
                    specs=config.invariant_specs,
                    state=state,
                    recovery_timeout_s=config.fault_recovery_timeout_s,
                    invariant_timeout_s=args.timeout_s,
                    debug_events=debug_events,
                )

            # 2. Mixed cleanup (existing)
            _cleanup_mixed_data(
                args=args,
                node=node,
                executor=executor,
                cleanup_client=cleanup_client,
                mixed_profile=config.mixed_profile,
                state=state,
                debug_events=debug_events,
            )

            # 3. Final invariants (existing)
            if config.invariant_specs and invariant_client is not None:
                _run_invariant_profile(
                    node=node,
                    executor=executor,
                    client=invariant_client,
                    specs=config.invariant_specs,
                    state=state,
                    phase='final',
                    timeout_s=args.timeout_s,
                    debug_events=debug_events,
                )

            # 4. Teardown (complement of setup — runs last)
            if setup_teardown_client is not None and config.teardown_specs:
                _run_phase_queries(
                    node=node,
                    executor=executor,
                    client=setup_teardown_client,
                    specs=config.teardown_specs,
                    state=state,
                    phase='teardown',
                    timeout_s=args.timeout_s,
                    debug_events=debug_events,
                )

            debug_events.log(
                'experiment_finished',
                records=len(state.records),
                invariant_records=len(state.invariant_records),
                timeouts=sum(
                    1 for record in state.records if record.timed_out
                ),
                invariant_failures=sum(
                    1
                    for record in state.invariant_records
                    if not record.success
                ),
            )
    finally:
        if executor is not None:
            executor.remove_node(node)
            executor.shutdown()
        node.destroy_node()
    fault_recovery_s = (
        state.fault_recovered_at_s - state.fault_triggered_at_s
        if state.fault_triggered_at_s is not None
        and state.fault_recovered_at_s is not None
        else None
    )
    fault_restart_outage_s = (
        state.fault_restart_ended_at_s - state.fault_restart_started_at_s
        if state.fault_restart_started_at_s is not None
        and state.fault_restart_ended_at_s is not None
        else None
    )
    fault_result = FaultResult(
        fault=config.fault,
        fault_at_s=config.fault_at_s,
        fault_triggered_at_s=state.fault_triggered_at_s,
        fault_delete_success=state.fault_delete_success,
        fault_delete_error=state.fault_delete_error,
        fault_delete_latency_s=state.fault_delete_latency_s,
        fault_recovered_at_s=state.fault_recovered_at_s,
        fault_recovery_s=fault_recovery_s,
        fault_restart_stop_success=state.fault_restart_stop_success,
        fault_restart_stop_error=state.fault_restart_stop_error,
        fault_restart_stop_latency_s=state.fault_restart_stop_latency_s,
        fault_restart_start_success=state.fault_restart_start_success,
        fault_restart_start_error=state.fault_restart_start_error,
        fault_restart_start_latency_s=state.fault_restart_start_latency_s,
        fault_restart_delay_s=state.fault_restart_delay_s,
        fault_restart_outage_s=fault_restart_outage_s,
        fault_observed_outage_s=_compute_observed_fault_outage_s(
            state.records,
            fault_triggered_at_s=state.fault_triggered_at_s,
        ),
        fault_lifecycle_deactivate_success=(
            state.fault_lifecycle_deactivate_success
        ),
        fault_lifecycle_deactivate_error=(
            state.fault_lifecycle_deactivate_error
        ),
        fault_lifecycle_deactivate_latency_s=(
            state.fault_lifecycle_deactivate_latency_s
        ),
        fault_lifecycle_cleanup_success=state.fault_lifecycle_cleanup_success,
        fault_lifecycle_cleanup_error=state.fault_lifecycle_cleanup_error,
        fault_lifecycle_cleanup_latency_s=(
            state.fault_lifecycle_cleanup_latency_s
        ),
        fault_lifecycle_configure_success=(
            state.fault_lifecycle_configure_success
        ),
        fault_lifecycle_configure_error=(
            state.fault_lifecycle_configure_error
        ),
        fault_lifecycle_configure_latency_s=(
            state.fault_lifecycle_configure_latency_s
        ),
        fault_lifecycle_activate_success=(
            state.fault_lifecycle_activate_success
        ),
        fault_lifecycle_activate_error=state.fault_lifecycle_activate_error,
        fault_lifecycle_activate_latency_s=(
            state.fault_lifecycle_activate_latency_s
        ),
        fault_lifecycle_reactivate=state.fault_lifecycle_reactivate,
    )
    return StressExperimentResult(
        records=state.records,
        invariant_records=state.invariant_records,
        config=config,
        fault_result=fault_result,
    )
