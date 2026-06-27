"""Tests for the ros_typedb stress experiment CLI helpers."""

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from ros_typedb_tools.fake_query_service import (
    build_fake_service_argument_parser,
)
from ros_typedb_tools.ros_typedb_stress_experiment import (
    build_argument_parser,
)
from ros_typedb_tools.stress_config import (
    QUERY_TYPE_BY_NAME,
    RequestRecord,
    build_query_specs,
)
from ros_typedb_tools.stress_output import (
    DebugEventWriter,
    default_timeout_output_path,
    summarize_records,
    write_results,
    write_timeout_results,
)
from ros_typedb_tools.stress_runner import run_experiment


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
        query_mix=build_query_specs(
            SimpleNamespace(query=None, query_type=None, mode='read')
        ),
        request_gap_s=0.01,
        max_in_flight=1,
        executor='single',
    )

    payload = json.loads(output_path.read_text(encoding='utf-8'))
    assert payload['service_name'] == '/ros_typedb_interface/query'
    assert payload['clients'] == 2
    assert payload['duration_s'] == 10.0
    assert payload['request_gap_s'] == 0.01
    assert payload['max_in_flight'] == 1
    assert payload['executor'] == 'single'
    assert payload['mode'] == 'read'
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


def test_build_query_specs_rejects_query_without_query_type():
    """Check explicit queries still require an explicit query type."""
    args = SimpleNamespace(
        query='match $x isa entity; fetch $x: attribute;',
        query_type=None,
        mode='read',
    )

    with pytest.raises(ValueError, match='--query-type'):
        build_query_specs(args)


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
        query='match $x isa entity; fetch $x: attribute;',
        query_type='fetch',
        mode='read',
    )

    with pytest.raises(ValueError, match='--clients'):
        run_experiment(args)
