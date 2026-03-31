"""Tests for TypeDB rule diagram parsing and rendering."""

from pathlib import Path

import pytest

from ros_typedb_tools.rule_diagram import build_rule_graph
from ros_typedb_tools.rule_diagram import default_rule_output_path
from ros_typedb_tools.rule_diagram import parse_rule_file
from ros_typedb_tools.rule_diagram import parse_rule_files
from ros_typedb_tools.rule_diagram import parse_rule_text


def _fixture_rule_path() -> Path:
    return Path(__file__).parent / "fixtures" / "mock_rules_schema.tql"


def test_parse_rule_file_extracts_read_write_concepts():
    model = parse_rule_file(_fixture_rule_path())

    assert set(model.rules) == {"detect_alarm", "escalate_alarm", "monitor_alarm"}
    assert model.rules["detect_alarm"].reads == {"measurement", "confidence"}
    assert model.rules["detect_alarm"].writes == {"alarm"}
    assert model.rules["escalate_alarm"].reads == {"alarm"}
    assert model.rules["escalate_alarm"].writes == {"status"}
    assert model.rules["monitor_alarm"].reads == {"alarm", "measurement"}
    assert model.rules["monitor_alarm"].writes == {"status"}


def test_parse_rule_text_handles_isa_bang_and_nested_braces():
    model = parse_rule_text(
        """
define
rule strict_rule:
when {
    { $x isa! Alarm; } or { $x isa Sensor; };
}
then {
    $x has status "active";
};
"""
    )

    assert model.rules["strict_rule"].reads == {"Alarm", "Sensor"}
    assert model.rules["strict_rule"].writes == {"status"}


def test_parse_rule_text_ignores_concept_like_tokens_inside_string_literals():
    model = parse_rule_text(
        """
define
rule quoted_tokens:
when {
    $x isa Alarm;
}
then {
    $x has status "do not parse has fake_attr or isa FakeThing";
};
"""
    )

    assert model.rules["quoted_tokens"].reads == {"Alarm"}
    assert model.rules["quoted_tokens"].writes == {"status"}


def test_parse_rule_files_rejects_duplicate_rule_names(tmp_path: Path):
    first = tmp_path / "first.tql"
    second = tmp_path / "second.tql"
    first.write_text(
        "define\nrule r:\nwhen { $x isa A; }\nthen { $x has attr_a true; };",
        encoding="utf-8",
    )
    second.write_text(
        "define\nrule r:\nwhen { $x isa B; }\nthen { $x has attr_b true; };",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="Duplicate rule definition encountered"):
        parse_rule_files([first, second])


def test_build_rule_graph_contains_read_write_and_dependency_edges():
    model = parse_rule_file(_fixture_rule_path())
    graph_dot = build_rule_graph(model, include_rule_dependencies=True).to_string()

    assert "concept__measurement -> rule__detect_alarm" in graph_dot
    assert "rule__detect_alarm -> concept__alarm" in graph_dot
    assert "rule__detect_alarm -> rule__escalate_alarm" in graph_dot
    assert "via alarm" in graph_dot


def test_build_rule_graph_disables_rule_dependencies():
    model = parse_rule_file(_fixture_rule_path())
    graph_dot = build_rule_graph(model, include_rule_dependencies=False).to_string()

    assert "-> rule__escalate_alarm" in graph_dot
    assert "rule__detect_alarm -> rule__escalate_alarm" not in graph_dot


def test_build_rule_graph_rejects_unknown_orientation():
    model = parse_rule_file(_fixture_rule_path())
    with pytest.raises(ValueError, match="Unsupported orientation"):
        build_rule_graph(model, orientation="diagonal")


def test_default_rule_output_path_prefers_sibling_schemas_diagrams():
    input_path = Path("/tmp/workspace/schemas/domain/rules.tql")
    expected = Path("/tmp/workspace/schemas_diagrams/rules_rules.svg")
    assert default_rule_output_path(input_path, "svg") == expected


def test_default_rule_output_path_next_to_input_when_not_in_schemas():
    input_path = Path("/tmp/workspace/rules.tql")
    expected = Path("/tmp/workspace/rules_rules.dot")
    assert default_rule_output_path(input_path, "dot") == expected
