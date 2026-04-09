"""Tests for TypeDB function diagram parsing and rendering."""

from pathlib import Path

import pytest

from ros_typedb_tools.rule_diagram import build_rule_graph
from ros_typedb_tools.rule_diagram import default_rule_output_path
from ros_typedb_tools.rule_diagram import parse_rule_file
from ros_typedb_tools.rule_diagram import parse_rule_files
from ros_typedb_tools.rule_diagram import parse_rule_text


def _fixture_function_path() -> Path:
    return Path(__file__).parent / "fixtures" / "mock_functions_schema.tql"


def test_parse_rule_file_extracts_function_reads_returns_and_calls():
    model = parse_rule_file(_fixture_function_path())

    assert set(model.rules) == {
        "measurement_score",
        "measurement_alert",
        "measurement_review_target",
    }
    assert model.rules["measurement_score"].reads == {"Reading", "confidence"}
    assert model.rules["measurement_score"].returns == {"double"}
    assert model.rules["measurement_score"].calls == set()

    assert model.rules["measurement_alert"].reads == {"Reading", "confidence"}
    assert model.rules["measurement_alert"].returns == {"boolean"}
    assert model.rules["measurement_alert"].calls == {"measurement_score"}

    assert model.rules["measurement_review_target"].reads == {
        "Reading",
        "Sensor",
        "confidence",
    }
    assert model.rules["measurement_review_target"].returns == {"Sensor"}
    assert model.rules["measurement_review_target"].calls == {"measurement_alert"}


def test_parse_rule_text_handles_nested_braces_and_repeated_calls():
    model = parse_rule_text(
        """
define

entity Reading;
attribute confidence, value double;

fun measurement_score($reading: Reading) -> double:
  match
    $reading has confidence $c;
  return first $c;

fun strict_monitor($reading: Reading) -> boolean:
  match
    {
      let $score = measurement_score($reading);
      $score > 0.9;
    } or {
      let $score = measurement_score($reading);
      $score > 0.8;
    };
    let $result = true;
  return first $result;
"""
    )

    assert model.rules["strict_monitor"].reads == {"Reading", "confidence"}
    assert model.rules["strict_monitor"].returns == {"boolean"}
    assert model.rules["strict_monitor"].calls == {"measurement_score"}


def test_parse_rule_text_ignores_function_like_tokens_inside_string_literals():
    model = parse_rule_text(
        """
define

entity Sensor;
attribute status, value string;

fun literal_guard($sensor: Sensor) -> string:
  match
    $sensor has status $status;
    "measurement_alert(fake)" == "measurement_alert(fake)";
  return first $status;
"""
    )

    assert model.rules["literal_guard"].reads == {"Sensor", "status"}
    assert model.rules["literal_guard"].returns == {"string"}
    assert model.rules["literal_guard"].calls == set()


def test_parse_rule_files_rejects_duplicate_function_names(tmp_path: Path):
    first = tmp_path / "first.tql"
    second = tmp_path / "second.tql"
    function_text = (
        "define\n"
        "fun score($x: Thing) -> boolean:\n"
        "  match\n"
        "    let $r = true;\n"
        "  return first $r;"
    )
    first.write_text(
        function_text,
        encoding="utf-8",
    )
    second.write_text(
        function_text,
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="Duplicate function definition encountered"):
        parse_rule_files([first, second])


def test_build_rule_graph_contains_read_return_and_dependency_edges():
    model = parse_rule_file(_fixture_function_path())
    graph_dot = build_rule_graph(model, include_rule_dependencies=True).to_string()

    assert "concept__Reading -> fun__measurement_score" in graph_dot
    assert "fun__measurement_score -> concept__double" in graph_dot
    assert "fun__measurement_alert -> fun__measurement_score" in graph_dot
    assert 'label=calls' in graph_dot


def test_build_rule_graph_disables_function_dependencies():
    model = parse_rule_file(_fixture_function_path())
    graph_dot = build_rule_graph(model, include_rule_dependencies=False).to_string()

    assert "fun__measurement_score" in graph_dot
    assert "fun__measurement_alert -> fun__measurement_score" not in graph_dot


def test_build_rule_graph_rejects_unknown_orientation():
    model = parse_rule_file(_fixture_function_path())
    with pytest.raises(ValueError, match="Unsupported orientation"):
        build_rule_graph(model, orientation="diagonal")


def test_default_rule_output_path_prefers_sibling_schemas_diagrams():
    input_path = Path("/tmp/workspace/schemas/domain/functions.tql")
    expected = Path("/tmp/workspace/schemas_diagrams/functions_rules.svg")
    assert default_rule_output_path(input_path, "svg") == expected


def test_default_rule_output_path_next_to_input_when_not_in_schemas():
    input_path = Path("/tmp/workspace/functions.tql")
    expected = Path("/tmp/workspace/functions_rules.dot")
    assert default_rule_output_path(input_path, "dot") == expected
