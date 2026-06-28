"""Tests for the ros_typedb stress experiment CLI helpers."""

import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue

from ros_typedb_msgs.msg import Attribute
from ros_typedb_msgs.msg import QueryResult
from ros_typedb_msgs.msg import ResultTree
from ros_typedb_benchmark.fake_query_service import (
    build_fake_service_argument_parser,
)
import ros_typedb_benchmark.ros_typedb_stress_experiment as stress_cli
from ros_typedb_benchmark.ros_typedb_stress_experiment import (
    build_argument_parser,
)
from ros_typedb_benchmark.stress_config import (
    build_invariant_specs,
    build_query_specs,
    DEFAULT_TYPEDB_START_COMMAND,
    DEFAULT_TYPEDB_STOP_COMMAND,
    FaultResult,
    InvariantRecord,
    InvariantSpec,
    QUERY_TYPE_BY_NAME,
    RequestRecord,
    ResolvedStressConfig,
    StressExperimentResult,
    validate_experiment_args,
)
from ros_typedb_benchmark.stress_invariants import evaluate_invariant_response
from ros_typedb_benchmark.stress_mixed import build_mixed_cleanup_query_spec
from ros_typedb_benchmark.stress_mixed import build_mixed_query_spec
from ros_typedb_benchmark.stress_mixed import load_mixed_query_profile
from ros_typedb_benchmark.stress_mixed import make_mixed_key
from ros_typedb_benchmark.stress_output import (
    DebugEventWriter,
    default_timeout_output_path,
    summarize_invariant_records,
    summarize_records,
    write_results,
    write_timeout_results,
)
from ros_typedb_benchmark.stress_resolution import resolve_stress_config
from ros_typedb_benchmark.stress_runner import (
    _build_typedb_restart_commands,
    _call_lifecycle_transition,
    _compute_observed_fault_outage_s,
    _derive_lifecycle_change_state_service_name,
    _derive_lifecycle_get_state_service_name,
    _mark_fault_recovered_from_invariant_pass,
    _run_fault_command,
    _start_fault_command,
)
from ros_typedb_benchmark.stress_runner import _RunState
from ros_typedb_benchmark.stress_runner import run_experiment


def _record(
    index: int,
    *,
    latency_s: float,
    success: bool,
    timed_out: bool = False,
) -> RequestRecord:
    return RequestRecord(
        index=index,
        client_id=0,
        query_index=0,
        query_type='fetch',
        query='match $x isa entity; fetch $x: attribute;',
        started_at_s=10.0 + index,
        ended_at_s=10.0 + index + latency_s,
        latency_s=latency_s,
        success=success,
        timed_out=timed_out,
        cancel_requested=timed_out,
        error_message='' if success else 'failed',
        result_count=1 if success else 0,
    )


def _invariant_record(
    index: int,
    *,
    success: bool,
    timed_out: bool = False,
    phase: str = 'final',
    started_at_s: float | None = None,
) -> InvariantRecord:
    started_at_s = 20.0 + index if started_at_s is None else started_at_s
    return InvariantRecord(
        index=index,
        phase=phase,
        name='person-count',
        query_type='get_aggregate',
        query='match $p isa person; get $p; count;',
        started_at_s=started_at_s,
        ended_at_s=started_at_s + 0.1,
        latency_s=0.1,
        success=success,
        timed_out=timed_out,
        error_message='' if success else 'failed',
        expected_value=20,
        actual_value=20 if success else 0,
    )


def _run_state(**overrides) -> _RunState:
    """Return minimal run state for helper tests."""
    state = _RunState(
        records=[],
        invariant_records=[],
        pending_by_client={},
        last_started_by_client=[],
        sent_count_by_client=[],
        mixed_cleanup_keys=set(),
        mixed_profile=None,
        mixed_run_id='test',
        next_index=0,
        next_invariant_index=0,
        deadline_s=None,
        next_snapshot_s=0.0,
        next_invariant_s=0.0,
    )
    for name, value in overrides.items():
        setattr(state, name, value)
    return state


class _DoneFuture:
    """Minimal completed future for lifecycle helper tests."""

    def __init__(self, *, result=None, exception=None):
        self._result = result
        self._exception = exception

    def done(self):
        return True

    def exception(self):
        return self._exception

    def result(self):
        return self._result


class _CapturingClient:
    """Minimal service client that captures the last async request."""

    def __init__(self, future):
        self.future = future
        self.request = None

    def call_async(self, request):
        self.request = request
        return self.future


def _aggregate_count_response(value: int, *, success: bool = True):
    parameter_value = ParameterValue()
    parameter_value.type = ParameterType.PARAMETER_INTEGER
    parameter_value.integer_value = value
    attribute = Attribute()
    attribute.value = parameter_value
    query_result = QueryResult()
    query_result.type = QueryResult.ATTRIBUTE
    query_result.attribute = attribute
    result_tree = ResultTree()
    result_tree.results.append(query_result)
    return SimpleNamespace(
        success=success,
        error_message='',
        results=[result_tree],
    )


def test_parser_accepts_stage_one_arguments():
    """Check the Stage 1 command-line arguments."""
    parser = build_argument_parser()

    args = parser.parse_args(
        [
            '--query',
            'match $x isa entity; fetch $x: attribute;',
            '--query-type',
            'fetch',
            '--requests',
            '3',
            '--timeout-s',
            '2.5',
            '--output',
            'results.json',
            '--timeout-output',
            'timeouts.json',
            '--debug-events-output',
            'events.jsonl',
            '--request-gap-s',
            '0.01',
            '--max-in-flight',
            '2',
            '--executor',
            'multi',
            '--executor-threads',
            '4',
            '--fail-on-failure',
        ]
    )

    assert args.service_name == '/ros_typedb_interface/query'
    assert args.query_type == 'fetch'
    assert args.requests == 3
    assert args.timeout_s == 2.5
    assert args.output == 'results.json'
    assert args.timeout_output == 'timeouts.json'
    assert args.debug_events_output == 'events.jsonl'
    assert args.request_gap_s == 0.01
    assert args.max_in_flight == 2
    assert args.executor == 'multi'
    assert args.executor_threads == 4
    assert args.fail_on_failure is True


def test_parser_accepts_stage_two_arguments():
    """Check Stage 2 duration and concurrency arguments."""
    parser = build_argument_parser()

    args = parser.parse_args(
        [
            '--clients',
            '20',
            '--duration-s',
            '60',
            '--timeout-s',
            '10',
            '--mode',
            'read',
        ]
    )

    assert args.clients == 20
    assert args.duration_s == 60
    assert args.mode == 'read'
    assert args.query is None
    assert args.query_type is None
    assert args.request_gap_s == 0.0
    assert args.max_in_flight is None
    assert args.executor == 'global'


def test_parser_accepts_stage_three_arguments():
    """Check Stage 3 invariant arguments."""
    parser = build_argument_parser()

    args = parser.parse_args(
        [
            '--clients',
            '10',
            '--duration-s',
            '60',
            '--mode',
            'read',
            '--invariant-profile',
            'test-data',
            '--invariant-period-s',
            '5',
        ]
    )

    assert args.invariant_profile == 'test-data'
    assert args.invariant_period_s == 5


def test_parser_accepts_stage_four_mixed_mode():
    """Check Stage 4 mixed read/write mode arguments."""
    parser = build_argument_parser()

    args = parser.parse_args(
        [
            '--clients',
            '10',
            '--duration-s',
            '120',
            '--mode',
            'mixed',
            '--mixed-profile',
            'plan-schema',
            '--invariant-profile',
            'plan-schema-mixed',
        ]
    )

    assert args.clients == 10
    assert args.duration_s == 120
    assert args.mode == 'mixed'
    assert args.mixed_profile == 'plan-schema'
    assert args.invariant_profile == 'plan-schema-mixed'


def test_parser_accepts_stage_six_restart_fault_arguments():
    """Check Stage 6 TypeDB restart fault arguments."""
    parser = build_argument_parser()

    args = parser.parse_args(
        [
            '--clients',
            '10',
            '--duration-s',
            '240',
            '--mode',
            'read',
            '--fault',
            'restart-typedb',
            '--typedb-container',
            'typedb_server',
            '--fault-at-s',
            '60',
            '--typedb-restart-delay-s',
            '3',
            '--fault-command-timeout-s',
            '15',
            '--invariant-profile',
            'test-data',
        ]
    )

    assert args.fault == 'restart-typedb'
    assert args.typedb_container == 'typedb_server'
    assert args.typedb_stop_command == DEFAULT_TYPEDB_STOP_COMMAND
    assert args.typedb_start_command == DEFAULT_TYPEDB_START_COMMAND
    assert args.fault_at_s == 60
    assert args.typedb_restart_delay_s == 3
    assert args.fault_command_timeout_s == 15


def test_parser_accepts_stage_seven_lifecycle_fault_arguments():
    """Check Stage 7 lifecycle cleanup fault arguments."""
    parser = build_argument_parser()

    args = parser.parse_args(
        [
            '--clients',
            '10',
            '--duration-s',
            '180',
            '--mode',
            'read',
            '--fault',
            'lifecycle-cleanup',
            '--fault-at-s',
            '60',
            '--lifecycle-change-state-service-name',
            '/ros_typedb/change_state',
            '--lifecycle-get-state-service-name',
            '/ros_typedb/get_state',
            '--no-lifecycle-reactivate',
            '--lifecycle-transition-timeout-s',
            '12',
            '--invariant-profile',
            'test-data',
        ]
    )

    assert args.fault == 'lifecycle-cleanup'
    assert args.fault_at_s == 60
    assert args.lifecycle_change_state_service_name == (
        '/ros_typedb/change_state'
    )
    assert args.lifecycle_get_state_service_name == '/ros_typedb/get_state'
    assert args.lifecycle_reactivate is False
    assert args.lifecycle_transition_timeout_s == 12


def test_fake_service_parser_accepts_diagnostic_arguments():
    """Check the fake service CLI options."""
    parser = build_fake_service_argument_parser()

    args = parser.parse_args(
        [
            '--service-name',
            '/debug_query',
            '--response-delay-s',
            '0.001',
            '--executor',
            'multi',
            '--executor-threads',
            '3',
        ]
    )

    assert args.service_name == '/debug_query'
    assert args.response_delay_s == 0.001
    assert args.executor == 'multi'
    assert args.executor_threads == 3


def test_query_type_names_cover_query_service_constants():
    """Check all query service types are accepted by name."""
    assert set(QUERY_TYPE_BY_NAME) == {
        'define',
        'delete',
        'fetch',
        'get',
        'get_aggregate',
        'insert',
        'update',
    }


def test_summarize_records_counts_success_failure_and_timeouts():
    """Check summary counters and latency metrics."""
    records = [
        _record(0, latency_s=0.1, success=True),
        _record(1, latency_s=0.4, success=False),
        _record(2, latency_s=0.2, success=False, timed_out=True),
    ]

    summary = summarize_records(records)

    assert summary['total_requests'] == 3
    assert summary['successes'] == 1
    assert summary['failures'] == 2
    assert summary['timeouts'] == 1
    assert summary['latency_s']['min'] == pytest.approx(0.1)
    assert summary['latency_s']['mean'] == pytest.approx(0.2333333333)
    assert summary['latency_s']['p50'] == pytest.approx(0.2)
    assert summary['latency_s']['p90'] == pytest.approx(0.36)
    assert summary['latency_s']['p95'] == pytest.approx(0.38)
    assert summary['latency_s']['p99'] == pytest.approx(0.396)
    assert summary['latency_s']['max'] == pytest.approx(0.4)


def test_summarize_invariant_records_counts_failures_and_timeouts():
    """Check invariant summary counters."""
    records = [
        _invariant_record(0, success=True),
        _invariant_record(1, success=False),
        _invariant_record(2, success=False, timed_out=True),
    ]

    summary = summarize_invariant_records(records)

    assert summary['total_invariants'] == 3
    assert summary['successes'] == 1
    assert summary['failures'] == 2
    assert summary['timeouts'] == 1


def test_fault_recovery_is_marked_from_successful_invariant_pass():
    """Check a successful post-fault invariant pass records recovery."""
    state = _run_state(
        fault_triggered=True,
        fault_triggered_at_s=1.0,
    )

    marked = _mark_fault_recovered_from_invariant_pass(
        state=state,
        records=[
            _invariant_record(0, success=True),
            _invariant_record(1, success=True),
        ],
        debug_events=DebugEventWriter(None),
    )

    assert marked is True
    assert state.fault_recovered_at_s is not None
    assert state.fault_recovered_at_s > state.fault_triggered_at_s


def test_fault_recovery_ignores_failed_invariant_pass():
    """Check failed invariant passes do not record recovery."""
    state = _run_state(
        fault_triggered=True,
        fault_triggered_at_s=1.0,
    )

    marked = _mark_fault_recovered_from_invariant_pass(
        state=state,
        records=[
            _invariant_record(0, success=True),
            _invariant_record(1, success=False),
        ],
        debug_events=DebugEventWriter(None),
    )

    assert marked is False
    assert state.fault_recovered_at_s is None


def test_write_results_writes_summary_and_request_records(tmp_path: Path):
    """Check JSON output includes metadata, summary, and records."""
    output_path = tmp_path / 'stress' / 'results.json'
    records = [_record(0, latency_s=0.1, success=True)]

    write_results(
        output_path,
        service_name='/ros_typedb_interface/query',
        query_type='fetch',
        query='match $x isa entity; fetch $x: attribute;',
        timeout_s=5.0,
        records=records,
        clients=2,
        duration_s=10.0,
        mode='read',
        mixed_profile='auto',
        query_mix=build_query_specs(
            SimpleNamespace(query=None, query_type=None, mode='read')
        ),
        request_gap_s=0.01,
        max_in_flight=1,
        executor='single',
        invariant_profile='test-data',
        invariant_period_s=5.0,
        invariant_records=[_invariant_record(0, success=True)],
        fault_result=FaultResult(
            fault='delete-database',
            fault_at_s=10.0,
            fault_triggered_at_s=10.25,
            fault_delete_success=True,
            fault_delete_error=None,
            fault_delete_latency_s=0.125,
            fault_recovered_at_s=12.75,
            fault_recovery_s=2.5,
        ),
    )

    payload = json.loads(output_path.read_text(encoding='utf-8'))
    assert payload['service_name'] == '/ros_typedb_interface/query'
    assert payload['clients'] == 2
    assert payload['duration_s'] == 10.0
    assert payload['request_gap_s'] == 0.01
    assert payload['max_in_flight'] == 1
    assert payload['executor'] == 'single'
    assert payload['invariant_profile'] == 'test-data'
    assert payload['invariant_period_s'] == 5.0
    assert payload['mode'] == 'read'
    assert payload['mixed_profile'] == 'auto'
    assert payload['query_type'] == 'fetch'
    assert payload['query_mix'][0]['query_type'] in QUERY_TYPE_BY_NAME
    assert payload['summary']['successes'] == 1
    assert payload['requests'][0]['index'] == 0
    assert payload['requests'][0]['client_id'] == 0
    assert payload['requests'][0]['query_index'] == 0
    assert payload['requests'][0]['query_type'] == 'fetch'
    assert payload['requests'][0]['query'] == (
        'match $x isa entity; fetch $x: attribute;'
    )
    assert payload['requests'][0]['success'] is True
    assert payload['invariant_summary']['successes'] == 1
    assert payload['invariants'][0]['name'] == 'person-count'
    assert payload['fault'] == {
        'fault': 'delete-database',
        'fault_at_s': 10.0,
        'fault_triggered_at_s': 10.25,
        'fault_delete_success': True,
        'fault_delete_error': None,
        'fault_delete_latency_s': 0.125,
        'fault_recovered_at_s': 12.75,
        'fault_recovery_s': 2.5,
        'fault_restart_stop_success': None,
        'fault_restart_stop_error': None,
        'fault_restart_stop_latency_s': None,
        'fault_restart_start_success': None,
        'fault_restart_start_error': None,
        'fault_restart_start_latency_s': None,
        'fault_restart_delay_s': None,
        'fault_restart_outage_s': None,
        'fault_observed_outage_s': None,
        'fault_lifecycle_deactivate_success': None,
        'fault_lifecycle_deactivate_error': None,
        'fault_lifecycle_deactivate_latency_s': None,
        'fault_lifecycle_cleanup_success': None,
        'fault_lifecycle_cleanup_error': None,
        'fault_lifecycle_cleanup_latency_s': None,
        'fault_lifecycle_configure_success': None,
        'fault_lifecycle_configure_error': None,
        'fault_lifecycle_configure_latency_s': None,
        'fault_lifecycle_activate_success': None,
        'fault_lifecycle_activate_error': None,
        'fault_lifecycle_activate_latency_s': None,
        'fault_lifecycle_reactivate': None,
    }


def test_write_results_writes_default_fault_payload(tmp_path: Path):
    """Check JSON output always includes fault metadata."""
    output_path = tmp_path / 'results.json'

    write_results(
        output_path,
        service_name='/ros_typedb_interface/query',
        query_type='fetch',
        query='match $x isa entity; fetch $x: attribute;',
        timeout_s=5.0,
        records=[],
    )

    payload = json.loads(output_path.read_text(encoding='utf-8'))
    assert payload['fault'] == {
        'fault': 'none',
        'fault_at_s': None,
        'fault_triggered_at_s': None,
        'fault_delete_success': None,
        'fault_delete_error': None,
        'fault_delete_latency_s': None,
        'fault_recovered_at_s': None,
        'fault_recovery_s': None,
        'fault_restart_stop_success': None,
        'fault_restart_stop_error': None,
        'fault_restart_stop_latency_s': None,
        'fault_restart_start_success': None,
        'fault_restart_start_error': None,
        'fault_restart_start_latency_s': None,
        'fault_restart_delay_s': None,
        'fault_restart_outage_s': None,
        'fault_observed_outage_s': None,
        'fault_lifecycle_deactivate_success': None,
        'fault_lifecycle_deactivate_error': None,
        'fault_lifecycle_deactivate_latency_s': None,
        'fault_lifecycle_cleanup_success': None,
        'fault_lifecycle_cleanup_error': None,
        'fault_lifecycle_cleanup_latency_s': None,
        'fault_lifecycle_configure_success': None,
        'fault_lifecycle_configure_error': None,
        'fault_lifecycle_configure_latency_s': None,
        'fault_lifecycle_activate_success': None,
        'fault_lifecycle_activate_error': None,
        'fault_lifecycle_activate_latency_s': None,
        'fault_lifecycle_reactivate': None,
    }


def test_build_typedb_restart_commands_uses_container():
    """Check container restart commands override process hooks."""
    stop_command, start_command = _build_typedb_restart_commands(
        typedb_container='typedb_server',
        typedb_stop_command='custom stop',
        typedb_start_command='custom start',
    )

    assert stop_command == ['docker', 'stop', 'typedb_server']
    assert start_command == ['docker', 'start', 'typedb_server']


def test_build_typedb_restart_commands_uses_default_process_hooks():
    """Check default process restart commands."""
    stop_command, start_command = _build_typedb_restart_commands(
        typedb_container=None,
        typedb_stop_command=None,
        typedb_start_command=None,
    )

    assert stop_command == ['pkill', '-f', 'typedb/core/server']
    assert start_command == ['typedb', 'server']


def test_build_typedb_restart_commands_uses_command_hooks():
    """Check restart command hooks are parsed into argv."""
    stop_command, start_command = _build_typedb_restart_commands(
        typedb_container=None,
        typedb_stop_command='bash -lc "typedb server stop"',
        typedb_start_command='bash -lc "typedb server start"',
    )

    assert stop_command == ['bash', '-lc', 'typedb server stop']
    assert start_command == ['bash', '-lc', 'typedb server start']


def test_derive_lifecycle_change_state_service_name_from_query_service():
    """Check lifecycle service name follows the query service prefix."""
    assert _derive_lifecycle_change_state_service_name(
        '/ros_typedb_interface/query'
    ) == '/ros_typedb_interface/change_state'
    assert _derive_lifecycle_change_state_service_name(
        '/query'
    ) == '/change_state'


def test_derive_lifecycle_get_state_service_name_from_query_service():
    """Check lifecycle get_state service name follows the query prefix."""
    assert _derive_lifecycle_get_state_service_name(
        '/ros_typedb_interface/query'
    ) == '/ros_typedb_interface/get_state'
    assert _derive_lifecycle_get_state_service_name(
        '/query'
    ) == '/get_state'


def test_call_lifecycle_transition_reports_success():
    """Check lifecycle transition helper builds a ChangeState request."""
    response = SimpleNamespace(success=True)
    client = _CapturingClient(_DoneFuture(result=response))

    success, error, latency_s = _call_lifecycle_transition(
        client=client,
        transition_id=3,
        transition_name='activate',
        timeout_s=1.0,
    )

    assert success is True
    assert error is None
    assert latency_s >= 0.0
    assert client.request.transition.id == 3


def test_call_lifecycle_transition_reports_rejected_transition():
    """Check lifecycle transition helper treats success=False as failure."""
    response = SimpleNamespace(success=False)
    client = _CapturingClient(_DoneFuture(result=response))

    success, error, latency_s = _call_lifecycle_transition(
        client=client,
        transition_id=2,
        transition_name='cleanup',
        timeout_s=1.0,
    )

    assert success is False
    assert error == 'lifecycle cleanup transition rejected'
    assert latency_s >= 0.0


def test_run_fault_command_reports_nonzero_exit(monkeypatch):
    """Check fault command execution reports stderr on failure."""
    class Completed:
        returncode = 2
        stdout = ''
        stderr = 'failed to stop'

    def fake_run(command, **kwargs):
        assert command == ['typedb', 'server', 'stop']
        assert kwargs['timeout'] == 5.0
        return Completed()

    monkeypatch.setattr(
        'ros_typedb_benchmark.stress_runner.subprocess.run',
        fake_run,
    )

    success, error, latency_s = _run_fault_command(
        ['typedb', 'server', 'stop'],
        timeout_s=5.0,
    )

    assert success is False
    assert error == 'failed to stop'
    assert latency_s >= 0.0


def test_start_fault_command_treats_running_process_as_success(monkeypatch):
    """Check foreground server commands succeed once they stay running."""

    class RunningProcess:
        def wait(self, timeout):
            assert timeout == 0.25
            raise subprocess.TimeoutExpired(['typedb', 'server'], timeout)

    def fake_popen(command, **kwargs):
        assert command == ['typedb', 'server']
        assert kwargs['stdout'] == subprocess.DEVNULL
        assert kwargs['stderr'] == subprocess.DEVNULL
        return RunningProcess()

    monkeypatch.setattr(
        'ros_typedb_benchmark.stress_runner.subprocess.Popen',
        fake_popen,
    )

    success, error, latency_s = _start_fault_command(
        ['typedb', 'server'],
        observation_s=0.25,
    )

    assert success is True
    assert error is None
    assert latency_s >= 0.0


def test_compute_observed_fault_outage_s_uses_failure_to_success_window():
    """Check observed outage is measured from failure to later success."""
    records = [
        _record(0, latency_s=0.1, success=True),
        _record(1, latency_s=0.2, success=False),
        _record(2, latency_s=0.4, success=False, timed_out=True),
        _record(3, latency_s=0.3, success=True),
    ]

    outage_s = _compute_observed_fault_outage_s(
        records,
        fault_triggered_at_s=10.5,
    )

    assert outage_s == pytest.approx(
        records[3].ended_at_s - records[1].ended_at_s
    )


def test_default_timeout_output_path_uses_result_stem():
    """Check timeout output path is derived from the main output path."""
    assert default_timeout_output_path(
        Path('/tmp/results.json')
    ) == Path('/tmp/results_timeouts.json')
    assert default_timeout_output_path(
        Path('/tmp/results')
    ) == Path('/tmp/results_timeouts.json')


def test_write_timeout_results_writes_only_timeout_records(tmp_path: Path):
    """Check timeout output contains only timed-out request records."""
    output_path = tmp_path / 'stress' / 'timeouts.json'
    records = [
        _record(0, latency_s=0.1, success=True),
        _record(1, latency_s=10.0, success=False, timed_out=True),
    ]

    write_timeout_results(
        output_path,
        source_output_path=tmp_path / 'results.json',
        records=records,
    )

    payload = json.loads(output_path.read_text(encoding='utf-8'))
    assert payload['source_output_path'] == str(tmp_path / 'results.json')
    assert payload['summary']['total_requests'] == 2
    assert payload['summary']['timeouts'] == 1
    assert len(payload['timeouts']) == 1
    assert payload['timeouts'][0]['index'] == 1
    assert payload['timeouts'][0]['query_index'] == 0
    assert payload['timeouts'][0]['cancel_requested'] is True


def test_debug_event_writer_writes_json_lines(tmp_path: Path):
    """Check debug events are written as JSONL records."""
    output_path = tmp_path / 'debug' / 'events.jsonl'

    with DebugEventWriter(output_path) as writer:
        writer.log('request_sent', index=1, pending_count=2)
        writer.log('request_timeout', index=1)

    events = [
        json.loads(line)
        for line in output_path.read_text(encoding='utf-8').splitlines()
    ]
    assert [event['event'] for event in events] == [
        'request_sent',
        'request_timeout',
    ]
    assert events[0]['index'] == 1
    assert 'monotonic_s' in events[0]


def test_build_query_specs_uses_single_explicit_query():
    """Check explicit query arguments override the built-in mix."""
    specs = build_query_specs(
        SimpleNamespace(
            query='match $x isa entity; fetch $x: attribute;',
            query_type='fetch',
            mode='read',
        )
    )

    assert len(specs) == 1
    assert specs[0].query == 'match $x isa entity; fetch $x: attribute;'
    assert specs[0].query_type == 'fetch'


def test_build_query_specs_uses_read_query_mix_by_default():
    """Check read mode provides runnable queries without explicit input."""
    specs = build_query_specs(
        SimpleNamespace(query=None, query_type=None, mode='read')
    )

    assert len(specs) > 1
    assert all(spec.query_type in QUERY_TYPE_BY_NAME for spec in specs)


def test_resolve_stress_config_uses_mixed_query_mix():
    """Check resolved mixed mode includes read and write query types."""
    config = resolve_stress_config(
        SimpleNamespace(
            requests=1,
            clients=1,
            duration_s=None,
            timeout_s=5.0,
            wait_service_timeout_s=1.0,
            request_gap_s=0.0,
            max_in_flight=None,
            executor='global',
            executor_threads=2,
            debug_events_output=None,
            invariant_profile='none',
            invariant_period_s=10.0,
            query=None,
            query_type=None,
            mode='mixed',
            mixed_profile='test-data',
        )
    )

    query_types = {spec.query_type for spec in config.query_specs}
    assert {'delete', 'get', 'insert', 'update'} <= query_types
    assert config.mixed_profile_name == 'test-data'
    assert config.max_in_flight == 1


def test_build_mixed_query_spec_cycles_temporary_robot_operations():
    """Check mixed mode generates insert, update, and delete robot queries."""
    profile = load_mixed_query_profile(
        SimpleNamespace(mixed_profile='test-data', invariant_profile='none')
    )
    read_spec = build_mixed_query_spec(
        profile=profile,
        run_id='abc123',
        client_id=2,
        client_request_index=0,
    )
    insert_spec = build_mixed_query_spec(
        profile=profile,
        run_id='abc123',
        client_id=2,
        client_request_index=1,
    )
    update_spec = build_mixed_query_spec(
        profile=profile,
        run_id='abc123',
        client_id=2,
        client_request_index=3,
    )
    delete_spec = build_mixed_query_spec(
        profile=profile,
        run_id='abc123',
        client_id=2,
        client_request_index=5,
    )

    key = make_mixed_key(run_id='abc123', client_id=2, cycle=0)
    assert read_spec.query_type == 'get'
    assert read_spec.cleanup_key is None
    assert insert_spec.query_type == 'insert'
    assert f'has full-name "{key}"' in insert_spec.query
    assert 'isa robot' in insert_spec.query
    assert insert_spec.cleanup_key == key
    assert update_spec.query_type == 'update'
    assert f'has full-name "{key}"' in update_spec.query
    assert 'delete $robot has $age;' in update_spec.query
    assert update_spec.cleanup_key == key
    assert delete_spec.query_type == 'delete'
    assert f'has full-name "{key}"' in delete_spec.query
    assert delete_spec.cleanup_key == key


def test_build_mixed_cleanup_query_spec_deletes_exact_temp_key():
    """Check mixed cleanup targets only one generated robot key."""
    profile = load_mixed_query_profile(
        SimpleNamespace(mixed_profile='test-data', invariant_profile='none')
    )
    spec = build_mixed_cleanup_query_spec(
        profile,
        'ros-typedb-stress-run-c0-n0',
    )

    assert spec.query_type == 'delete'
    assert spec.cleanup_key == 'ros-typedb-stress-run-c0-n0'
    assert 'isa robot' in spec.query
    assert 'has full-name "ros-typedb-stress-run-c0-n0"' in spec.query


def test_build_mixed_query_spec_supports_plan_schema_profile():
    """Check plan-schema mixed mode writes temporary actions."""
    profile = load_mixed_query_profile(
        SimpleNamespace(mixed_profile='plan-schema', invariant_profile='none')
    )

    insert_spec = build_mixed_query_spec(
        profile=profile,
        run_id='abc123',
        client_id=1,
        client_request_index=1,
    )
    cleanup_spec = build_mixed_cleanup_query_spec(
        profile,
        insert_spec.cleanup_key,
    )

    assert insert_spec.query_type == 'insert'
    assert 'isa Action' in insert_spec.query
    assert 'has action_name "ros-typedb-stress-abc123-c1-n0"' in (
        insert_spec.query
    )
    assert 'has action_duration 0.0' in insert_spec.query
    assert cleanup_spec.query_type == 'delete'
    assert 'isa Action' in cleanup_spec.query


def test_build_query_specs_rejects_query_without_query_type():
    """Check explicit queries still require an explicit query type."""
    args = SimpleNamespace(
        query='match $x isa entity; fetch $x: attribute;',
        query_type=None,
        mode='read',
    )

    with pytest.raises(ValueError, match='--query-type'):
        build_query_specs(args)


def test_build_invariant_specs_uses_test_data_profile():
    """Check the built-in test-data profile."""
    specs = build_invariant_specs(
        SimpleNamespace(invariant_profile='test-data')
    )

    assert [spec.name for spec in specs] == [
        'person-count',
        'company-count',
        'employment-count',
        'boss-sentinel-count',
    ]
    assert all(spec.query_type == 'get_aggregate' for spec in specs)


def test_build_invariant_specs_uses_plan_schema_profile():
    """Check the built-in plan-schema profile."""
    specs = build_invariant_specs(
        SimpleNamespace(invariant_profile='plan-schema')
    )

    assert [spec.name for spec in specs] == [
        'plan-count',
        'action-count',
        'proposition-count',
        'has-action-count',
        'action-preconditions-count',
        'action-effects-count',
        'collect-water-sample-sentinel-count',
    ]
    assert all(spec.query_type == 'get_aggregate' for spec in specs)
    assert {
        spec.name: spec.expected_value
        for spec in specs
    }['action-preconditions-count'] == 4


def test_build_invariant_specs_uses_plan_schema_mixed_profile():
    """Check the mixed-safe plan-schema invariant profile."""
    specs = build_invariant_specs(
        SimpleNamespace(invariant_profile='plan-schema-mixed')
    )

    assert [spec.name for spec in specs] == [
        'plan-count',
        'proposition-count',
        'has-action-count',
        'action-preconditions-count',
        'action-effects-count',
        'collect-water-sample-sentinel-count',
    ]
    assert all(spec.query_type == 'get_aggregate' for spec in specs)
    assert {
        spec.name: spec.expected_value
        for spec in specs
    }['action-preconditions-count'] == 4


def test_build_invariant_specs_returns_empty_list_for_none_profile():
    """Check invariant checks are optional."""
    specs = build_invariant_specs(SimpleNamespace(invariant_profile='none'))

    assert specs == []


def test_evaluate_invariant_response_compares_aggregate_value():
    """Check invariant evaluation reads aggregate scalar values."""
    spec = InvariantSpec(
        name='person-count',
        query='match $p isa person; get $p; count;',
        query_type='get_aggregate',
        expected_value=20,
    )

    record = evaluate_invariant_response(
        index=0,
        phase='final',
        spec=spec,
        started_at_s=1.0,
        ended_at_s=1.2,
        response=_aggregate_count_response(20),
    )

    assert record.success is True
    assert record.actual_value == 20


def test_evaluate_invariant_response_reports_unexpected_value():
    """Check invariant evaluation reports mismatched aggregate values."""
    spec = InvariantSpec(
        name='person-count',
        query='match $p isa person; get $p; count;',
        query_type='get_aggregate',
        expected_value=20,
    )

    record = evaluate_invariant_response(
        index=0,
        phase='final',
        spec=spec,
        started_at_s=1.0,
        ended_at_s=1.2,
        response=_aggregate_count_response(0),
    )

    assert record.success is False
    assert record.actual_value == 0
    assert 'expected 20' in record.error_message


def test_run_experiment_rejects_invalid_request_count():
    """Check request count validation before ROS service use."""
    args = SimpleNamespace(
        requests=0,
        clients=1,
        duration_s=None,
        timeout_s=5.0,
        wait_service_timeout_s=1.0,
        request_gap_s=0.0,
        max_in_flight=None,
        executor='global',
        executor_threads=2,
        debug_events_output=None,
        invariant_profile='none',
        invariant_period_s=10.0,
        query='match $x isa entity; fetch $x: attribute;',
        query_type='fetch',
        mode='read',
    )

    with pytest.raises(ValueError, match='--requests'):
        run_experiment(args)


def test_run_experiment_rejects_invalid_client_count():
    """Check client count validation before ROS service use."""
    args = SimpleNamespace(
        requests=1,
        clients=0,
        duration_s=None,
        timeout_s=5.0,
        wait_service_timeout_s=1.0,
        request_gap_s=0.0,
        max_in_flight=None,
        executor='global',
        executor_threads=2,
        debug_events_output=None,
        invariant_profile='none',
        invariant_period_s=10.0,
        query='match $x isa entity; fetch $x: attribute;',
        query_type='fetch',
        mode='read',
    )

    with pytest.raises(ValueError, match='--clients'):
        run_experiment(args)


def _base_args(**overrides):
    """Return a SimpleNamespace with valid base args for validation tests."""
    base = {
        'requests': 1,
        'clients': 1,
        'duration_s': 60.0,
        'timeout_s': 5.0,
        'wait_service_timeout_s': 1.0,
        'request_gap_s': 0.0,
        'max_in_flight': None,
        'executor': 'global',
        'executor_threads': 2,
        'debug_events_output': None,
        'invariant_profile': 'none',
        'invariant_period_s': 10.0,
        'query': None,
        'query_type': None,
        'mode': 'read',
        'mixed_profile': 'auto',
        'fault': 'none',
        'fault_at_s': None,
        'fault_recovery_timeout_s': 30.0,
        'typedb_container': None,
        'typedb_stop_command': DEFAULT_TYPEDB_STOP_COMMAND,
        'typedb_start_command': DEFAULT_TYPEDB_START_COMMAND,
        'typedb_restart_delay_s': 2.0,
        'fault_command_timeout_s': 30.0,
        'lifecycle_change_state_service_name': None,
        'lifecycle_get_state_service_name': None,
        'lifecycle_reactivate': True,
        'lifecycle_transition_timeout_s': 10.0,
    }
    base.update(overrides)
    return SimpleNamespace(**base)


def test_validate_args_fault_delete_database_requires_invariant_profile():
    """Check --fault delete-database requires --invariant-profile to be set."""
    args = _base_args(
        fault='delete-database',
        invariant_profile='none',
        fault_at_s=10.0,
    )

    with pytest.raises(ValueError, match='--invariant-profile'):
        validate_experiment_args(args)


def test_validate_args_fault_delete_database_requires_duration_s():
    """Check --fault delete-database requires --duration-s."""
    args = _base_args(
        fault='delete-database',
        invariant_profile='test-data',
        duration_s=None,
        fault_at_s=10.0,
    )

    with pytest.raises(ValueError, match='--duration-s'):
        validate_experiment_args(args)


def test_validate_args_fault_delete_database_requires_fault_at_s():
    """Check --fault delete-database requires --fault-at-s."""
    args = _base_args(
        fault='delete-database',
        invariant_profile='test-data',
        duration_s=60.0,
        fault_at_s=None,
    )

    with pytest.raises(ValueError, match='--fault-at-s'):
        validate_experiment_args(args)


def test_validate_args_fault_at_s_must_be_less_than_duration_s():
    """Check --fault-at-s must be less than --duration-s."""
    args = _base_args(
        fault='delete-database',
        invariant_profile='test-data',
        duration_s=30.0,
        fault_at_s=30.0,
    )

    with pytest.raises(ValueError, match='--fault-at-s must be less than'):
        validate_experiment_args(args)


def test_validate_args_fault_restart_typedb_accepts_default_controller():
    """Check restart-typedb accepts default process commands."""
    args = _base_args(
        fault='restart-typedb',
        invariant_profile='test-data',
        duration_s=60.0,
        fault_at_s=10.0,
    )

    validate_experiment_args(args)


def test_validate_args_fault_restart_typedb_accepts_container_override():
    """Check restart-typedb accepts container overriding process commands."""
    args = _base_args(
        fault='restart-typedb',
        invariant_profile='test-data',
        duration_s=60.0,
        fault_at_s=10.0,
        typedb_container='typedb_server',
        typedb_stop_command='typedb server stop',
        typedb_start_command='typedb server start',
    )

    validate_experiment_args(args)


def test_validate_args_fault_restart_typedb_accepts_container():
    """Check restart-typedb accepts a TypeDB container controller."""
    args = _base_args(
        fault='restart-typedb',
        invariant_profile='test-data',
        duration_s=60.0,
        fault_at_s=10.0,
        typedb_container='typedb_server',
    )

    validate_experiment_args(args)


def test_validate_args_fault_lifecycle_cleanup_accepts_defaults():
    """Check lifecycle-cleanup accepts default lifecycle controller args."""
    args = _base_args(
        fault='lifecycle-cleanup',
        invariant_profile='test-data',
        duration_s=60.0,
        fault_at_s=10.0,
    )

    validate_experiment_args(args)


def test_validate_args_lifecycle_transition_timeout_s_must_be_positive():
    """Check lifecycle transition timeout must be greater than zero."""
    args = _base_args(
        fault='lifecycle-cleanup',
        invariant_profile='test-data',
        duration_s=60.0,
        fault_at_s=10.0,
        lifecycle_transition_timeout_s=0.0,
    )

    with pytest.raises(ValueError, match='--lifecycle-transition-timeout-s'):
        validate_experiment_args(args)


def test_validate_args_fault_recovery_timeout_s_must_be_positive():
    """Check --fault-recovery-timeout-s must be greater than zero."""
    args = _base_args(fault_recovery_timeout_s=0.0)

    with pytest.raises(ValueError, match='--fault-recovery-timeout-s'):
        validate_experiment_args(args)


def test_validate_args_fault_none_passes_without_fault_args():
    """Check --fault none (default) passes validation with no fault args."""
    args = _base_args()

    validate_experiment_args(args)  # should not raise


def test_main_fails_when_triggered_fault_does_not_recover(monkeypatch):
    """Check CLI exits nonzero for unrecovered triggered faults."""
    fault_result = FaultResult(
        fault='delete-database',
        fault_at_s=10.0,
        fault_triggered_at_s=10.0,
        fault_delete_success=True,
        fault_delete_error=None,
        fault_delete_latency_s=0.1,
        fault_recovered_at_s=None,
        fault_recovery_s=None,
    )
    result = StressExperimentResult(
        records=[],
        invariant_records=[],
        config=ResolvedStressConfig(
            query_specs=[],
            invariant_specs=[],
            mixed_profile=None,
            mixed_profile_name=None,
            invariant_profile_name='test-data',
            max_in_flight=1,
            debug_events_output=None,
            fault='delete-database',
            fault_at_s=10.0,
            fault_recovery_timeout_s=30.0,
        ),
        fault_result=fault_result,
    )

    monkeypatch.setattr(stress_cli.rclpy, 'init', lambda args=None: None)
    monkeypatch.setattr(stress_cli.rclpy, 'shutdown', lambda: None)
    monkeypatch.setattr(stress_cli, 'run_experiment', lambda args: result)

    assert stress_cli.main([]) == 1


def test_main_fails_when_restart_fault_controller_fails(monkeypatch):
    """Check CLI exits nonzero when restart stop/start fails."""
    fault_result = FaultResult(
        fault='restart-typedb',
        fault_at_s=10.0,
        fault_triggered_at_s=10.0,
        fault_delete_success=None,
        fault_delete_error=None,
        fault_delete_latency_s=None,
        fault_recovered_at_s=12.0,
        fault_recovery_s=2.0,
        fault_restart_stop_success=False,
        fault_restart_stop_error='failed to stop',
        fault_restart_start_success=True,
    )
    result = StressExperimentResult(
        records=[],
        invariant_records=[],
        config=ResolvedStressConfig(
            query_specs=[],
            invariant_specs=[],
            mixed_profile=None,
            mixed_profile_name=None,
            invariant_profile_name='test-data',
            max_in_flight=1,
            debug_events_output=None,
            fault='restart-typedb',
            fault_at_s=10.0,
            fault_recovery_timeout_s=30.0,
        ),
        fault_result=fault_result,
    )

    monkeypatch.setattr(stress_cli.rclpy, 'init', lambda args=None: None)
    monkeypatch.setattr(stress_cli.rclpy, 'shutdown', lambda: None)
    monkeypatch.setattr(stress_cli, 'run_experiment', lambda args: result)

    assert stress_cli.main([]) == 1


def test_main_fails_when_lifecycle_fault_controller_fails(monkeypatch):
    """Check CLI exits nonzero when lifecycle cleanup fails."""
    fault_result = FaultResult(
        fault='lifecycle-cleanup',
        fault_at_s=10.0,
        fault_triggered_at_s=10.0,
        fault_delete_success=None,
        fault_delete_error=None,
        fault_delete_latency_s=None,
        fault_recovered_at_s=12.0,
        fault_recovery_s=2.0,
        fault_lifecycle_deactivate_success=True,
        fault_lifecycle_cleanup_success=False,
        fault_lifecycle_cleanup_error='cleanup rejected',
        fault_lifecycle_reactivate=True,
    )
    result = StressExperimentResult(
        records=[],
        invariant_records=[],
        config=ResolvedStressConfig(
            query_specs=[],
            invariant_specs=[],
            mixed_profile=None,
            mixed_profile_name=None,
            invariant_profile_name='test-data',
            max_in_flight=1,
            debug_events_output=None,
            fault='lifecycle-cleanup',
            fault_at_s=10.0,
            fault_recovery_timeout_s=30.0,
        ),
        fault_result=fault_result,
    )

    monkeypatch.setattr(stress_cli.rclpy, 'init', lambda args=None: None)
    monkeypatch.setattr(stress_cli.rclpy, 'shutdown', lambda: None)
    monkeypatch.setattr(stress_cli, 'run_experiment', lambda args: result)

    assert stress_cli.main([]) == 1


def test_main_allows_periodic_invariant_failures_during_recovered_fault(
    monkeypatch,
):
    """Check recovered fault runs tolerate outage-window invariant failures."""
    fault_result = FaultResult(
        fault='restart-typedb',
        fault_at_s=10.0,
        fault_triggered_at_s=10.0,
        fault_delete_success=None,
        fault_delete_error=None,
        fault_delete_latency_s=None,
        fault_recovered_at_s=20.0,
        fault_recovery_s=10.0,
        fault_restart_stop_success=True,
        fault_restart_start_success=True,
    )
    result = StressExperimentResult(
        records=[],
        invariant_records=[
            _invariant_record(
                0,
                success=False,
                phase='periodic',
                started_at_s=12.0,
            ),
            _invariant_record(
                1,
                success=False,
                phase='recovery',
                started_at_s=15.0,
            ),
            _invariant_record(
                2,
                success=True,
                phase='final',
                started_at_s=21.0,
            ),
        ],
        config=ResolvedStressConfig(
            query_specs=[],
            invariant_specs=[],
            mixed_profile=None,
            mixed_profile_name=None,
            invariant_profile_name='test-data',
            max_in_flight=1,
            debug_events_output=None,
            fault='restart-typedb',
            fault_at_s=10.0,
            fault_recovery_timeout_s=30.0,
        ),
        fault_result=fault_result,
    )

    monkeypatch.setattr(stress_cli.rclpy, 'init', lambda args=None: None)
    monkeypatch.setattr(stress_cli.rclpy, 'shutdown', lambda: None)
    monkeypatch.setattr(stress_cli, 'run_experiment', lambda args: result)

    assert stress_cli.main([]) == 0


def test_main_fails_final_invariant_failure_after_recovered_fault(monkeypatch):
    """Check final invariant failures still fail recovered fault runs."""
    fault_result = FaultResult(
        fault='restart-typedb',
        fault_at_s=10.0,
        fault_triggered_at_s=10.0,
        fault_delete_success=None,
        fault_delete_error=None,
        fault_delete_latency_s=None,
        fault_recovered_at_s=20.0,
        fault_recovery_s=10.0,
        fault_restart_stop_success=True,
        fault_restart_start_success=True,
    )
    result = StressExperimentResult(
        records=[],
        invariant_records=[
            _invariant_record(
                0,
                success=False,
                phase='final',
                started_at_s=21.0,
            ),
        ],
        config=ResolvedStressConfig(
            query_specs=[],
            invariant_specs=[],
            mixed_profile=None,
            mixed_profile_name=None,
            invariant_profile_name='test-data',
            max_in_flight=1,
            debug_events_output=None,
            fault='restart-typedb',
            fault_at_s=10.0,
            fault_recovery_timeout_s=30.0,
        ),
        fault_result=fault_result,
    )

    monkeypatch.setattr(stress_cli.rclpy, 'init', lambda args=None: None)
    monkeypatch.setattr(stress_cli.rclpy, 'shutdown', lambda: None)
    monkeypatch.setattr(stress_cli, 'run_experiment', lambda args: result)

    assert stress_cli.main([]) == 1
