"""Tests for the ros_typedb stress experiment CLI helpers."""

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from ros_typedb_tools.ros_typedb_stress_experiment import QUERY_TYPE_BY_NAME
from ros_typedb_tools.ros_typedb_stress_experiment import RequestRecord
from ros_typedb_tools.ros_typedb_stress_experiment import build_argument_parser
from ros_typedb_tools.ros_typedb_stress_experiment import run_experiment
from ros_typedb_tools.ros_typedb_stress_experiment import summarize_records
from ros_typedb_tools.ros_typedb_stress_experiment import write_results


def _record(
    index: int,
    *,
    latency_s: float,
    success: bool,
    timed_out: bool = False,
) -> RequestRecord:
    return RequestRecord(
        index=index,
        started_at_s=10.0 + index,
        ended_at_s=10.0 + index + latency_s,
        latency_s=latency_s,
        success=success,
        timed_out=timed_out,
        error_message='' if success else 'failed',
        result_count=1 if success else 0,
    )


def test_parser_accepts_stage_one_arguments():
    """Check the Stage 1 command-line arguments."""
    parser = build_argument_parser()

    args = parser.parse_args(
        [
            '--query',
            'match $x isa entity; fetch $x;',
            '--query-type',
            'fetch',
            '--requests',
            '3',
            '--timeout-s',
            '2.5',
            '--output',
            'results.json',
        ]
    )

    assert args.service_name == '/ros_typedb_interface/query'
    assert args.query_type == 'fetch'
    assert args.requests == 3
    assert args.timeout_s == 2.5
    assert args.output == 'results.json'


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
    assert summary['latency_s']['max'] == pytest.approx(0.4)


def test_write_results_writes_summary_and_request_records(tmp_path: Path):
    """Check JSON output includes metadata, summary, and records."""
    output_path = tmp_path / 'stress' / 'results.json'
    records = [_record(0, latency_s=0.1, success=True)]

    write_results(
        output_path,
        service_name='/ros_typedb_interface/query',
        query_type='fetch',
        query='match $x isa entity; fetch $x;',
        timeout_s=5.0,
        records=records,
    )

    payload = json.loads(output_path.read_text(encoding='utf-8'))
    assert payload['service_name'] == '/ros_typedb_interface/query'
    assert payload['query_type'] == 'fetch'
    assert payload['summary']['successes'] == 1
    assert payload['requests'][0]['index'] == 0
    assert payload['requests'][0]['success'] is True


def test_run_experiment_rejects_invalid_request_count():
    """Check request count validation before ROS service use."""
    args = SimpleNamespace(
        requests=0,
        timeout_s=5.0,
        wait_service_timeout_s=1.0,
    )

    with pytest.raises(ValueError, match='--requests'):
        run_experiment(args)
