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
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import SingleThreadedExecutor

from ros_typedb_msgs.srv import Query

from ros_typedb_tools.stress_config import QuerySpec
from ros_typedb_tools.stress_config import RequestRecord
from ros_typedb_tools.stress_config import build_query_request
from ros_typedb_tools.stress_config import build_query_specs
from ros_typedb_tools.stress_config import validate_experiment_args
from ros_typedb_tools.stress_output import DebugEventWriter


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
    pending_by_client: dict[int, _PendingRequest]
    last_started_by_client: list[float]
    next_index: int
    deadline_s: float | None
    next_snapshot_s: float


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
) -> _PendingRequest:
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


def run_experiment(args: argparse.Namespace) -> list[RequestRecord]:
    """Run the stress experiment."""
    validate_experiment_args(args)
    query_specs = build_query_specs(args)
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
    deadline_s = (
        time.monotonic() + args.duration_s
        if args.duration_s is not None
        else None
    )
    state = _RunState(
        records=[],
        pending_by_client={},
        last_started_by_client=[float('-inf') for _ in range(args.clients)],
        next_index=0,
        deadline_s=deadline_s,
        next_snapshot_s=time.monotonic() + 1.0,
    )

    try:
        with DebugEventWriter(debug_events_output) as debug_events:
            debug_events.log(
                'experiment_started',
                clients=args.clients,
                executor=args.executor,
                max_in_flight=max_in_flight,
                request_gap_s=args.request_gap_s,
            )
            _wait_for_all_clients(
                clients,
                service_name=args.service_name,
                wait_service_timeout_s=args.wait_service_timeout_s,
                debug_events=debug_events,
            )

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
                    query_specs=query_specs,
                    state=state,
                    max_in_flight=max_in_flight,
                    debug_events=debug_events,
                )

            debug_events.log(
                'experiment_finished',
                records=len(state.records),
                timeouts=sum(
                    1 for record in state.records if record.timed_out
                ),
            )
    finally:
        if executor is not None:
            executor.remove_node(node)
            executor.shutdown()
        node.destroy_node()
    return state.records
