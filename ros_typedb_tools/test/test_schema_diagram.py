"""Tests for TypeDB 3 schema diagram parsing and graph generation."""

from pathlib import Path

import pytest

from ros_typedb_tools.schema_diagram import build_graph
from ros_typedb_tools.schema_diagram import merge_schema_models
from ros_typedb_tools.schema_diagram import parse_schema_file
from ros_typedb_tools.schema_diagram import parse_schema_files
from ros_typedb_tools.schema_diagram import parse_schema_text


def _fixture_schema_path(filename: str = "mock_schema.tql") -> Path:
    return Path(__file__).parent / "fixtures" / filename


def test_parse_schema_extracts_core_elements():
    schema = parse_schema_file(_fixture_schema_path())

    assert "name" in schema.types
    assert schema.types["name"].kind == "attribute"
    assert schema.types["name"].value_type == "string"

    assert "connection" in schema.types
    assert schema.types["connection"].kind == "relation"
    assert schema.types["connection"].relates == {"source", "target"}

    assert "server" in schema.types
    assert schema.types["server"].kind == "entity"
    assert schema.types["server"].owns == {"name", "enabled"}
    assert ("connection", "source") in schema.types["server"].plays

    assert "client" in schema.types
    assert schema.types["client"].kind == "entity"
    assert schema.types["client"].parent == "server"
    assert ("connection", "target") in schema.types["client"].plays

    assert [function.name for function in schema.functions] == [
        "active_server_name",
        "connected_clients",
    ]


def test_graph_contains_expected_edges_and_function_notes():
    schema = parse_schema_file(_fixture_schema_path())
    graph_dot = build_graph(schema, include_functions=True).to_string()

    assert "server" in graph_dot
    assert "client" in graph_dot
    assert "connection" in graph_dot
    assert "label=sub" in graph_dot
    assert "label=owns" in graph_dot
    assert "plays source" in graph_dot
    assert "roles: source, target" in graph_dot
    assert "function__active_server_name" in graph_dot
    assert "function__connected_clients" in graph_dot


def test_build_graph_defaults_to_vertical_rankdir():
    schema = parse_schema_file(_fixture_schema_path())
    graph_dot = build_graph(schema).to_string()

    assert "rankdir=TB" in graph_dot


def test_build_graph_supports_horizontal_orientation():
    schema = parse_schema_file(_fixture_schema_path())
    graph_dot = build_graph(schema, orientation="horizontal").to_string()

    assert "rankdir=LR" in graph_dot


def test_build_graph_rejects_unknown_orientation():
    schema = parse_schema_file(_fixture_schema_path())
    with pytest.raises(ValueError, match="Unsupported orientation"):
        build_graph(schema, orientation="diagonal")


def test_parse_schema_captures_standalone_plays_statements():
    schema = parse_schema_file(_fixture_schema_path("mock_standalone_plays_schema.tql"))

    assert "Publisher" in schema.types
    assert "Value" in schema.types
    assert ("measurement_interface", "source") in schema.types["Publisher"].plays
    assert ("has_parameter_value", "parameter_value") in schema.types["Value"].plays


def test_graph_renders_edges_from_standalone_plays():
    schema = parse_schema_file(_fixture_schema_path("mock_standalone_plays_schema.tql"))
    graph_dot = build_graph(schema).to_string()

    assert "measurement_interface" in graph_dot
    assert "has_parameter_value" in graph_dot
    assert "plays source" in graph_dot
    assert "plays parameter_value" in graph_dot


def test_type_declaration_with_whitespace_around_sub_is_not_misclassified():
    schema = parse_schema_text(
        """
define

link sub relation,
    relates source;

Widget\tsub
    entity,
    plays link:source;
"""
    )

    assert "Widget" in schema.types
    assert schema.types["Widget"].kind == "entity"
    assert schema.types["Widget"].parent is None
    assert ("link", "source") in schema.types["Widget"].plays


def test_comment_stripping_preserves_hash_inside_quotes():
    schema = parse_schema_text(
        """
define

tag sub attribute,
    value string,
    regex "^[A-Z#_]+$"; # comment outside quotes

item sub entity,
    owns tag;
"""
    )

    assert "tag" in schema.types
    assert schema.types["tag"].kind == "attribute"
    assert schema.types["tag"].value_type == "string"
    assert "item" in schema.types
    assert schema.types["item"].owns == {"tag"}


def test_merge_schema_models_combines_definitions_and_functions():
    schema_a = parse_schema_text(
        """
define
name sub attribute, value string;
service sub relation, relates provider;
Node sub entity, owns name, plays service:provider;
fun validate_node() -> boolean:
  match
    $x isa Node;
    let $ok = true;
  return first $ok;
"""
    )
    schema_b = parse_schema_text(
        """
define
name sub attribute, value string;
Node sub entity, owns enabled;
enabled sub attribute, value boolean;
fun validate_node() -> boolean:
  match
    $x isa Node;
    let $ok = true;
  return first $ok;
"""
    )

    merged = merge_schema_models([schema_a, schema_b])

    assert merged.types["Node"].owns == {"enabled", "name"}
    assert ("service", "provider") in merged.types["Node"].plays
    assert merged.types["name"].value_type == "string"
    assert merged.types["enabled"].kind == "attribute"
    assert [function.name for function in merged.functions] == ["validate_node"]


def test_merge_schema_models_raises_on_conflicting_type_kind():
    schema_a = parse_schema_text("define\nThing sub entity;")
    schema_b = parse_schema_text("define\nThing sub relation, relates role_x;")

    with pytest.raises(ValueError, match="Conflicting kind definitions"):
        merge_schema_models([schema_a, schema_b])


def test_merge_schema_models_raises_on_conflicting_parent_root_vs_child():
    schema_a = parse_schema_text(
        """
define
Foo sub relation;
"""
    )
    schema_b = parse_schema_text(
        """
define
parent_relation sub relation;
Foo sub parent_relation;
"""
    )

    with pytest.raises(ValueError, match="Conflicting parent definitions"):
        merge_schema_models([schema_a, schema_b])


def test_merge_schema_models_keeps_known_parent_for_unknown_cross_file_type():
    schema_a = parse_schema_text(
        """
define
fuzzy_expression sub expression;
fuzzy_operator sub fuzzy_expression;
"""
    )
    schema_b = parse_schema_text(
        """
define
fuzzy_operator plays some_relation:role_x;
"""
    )

    merged = merge_schema_models([schema_a, schema_b])

    assert merged.types["fuzzy_operator"].parent == "fuzzy_expression"
    assert ("some_relation", "role_x") in merged.types["fuzzy_operator"].plays


def test_parse_schema_files_merges_multiple_files(tmp_path: Path):
    first = tmp_path / "first.tql"
    second = tmp_path / "second.tql"

    first.write_text("define\nA sub entity;", encoding="utf-8")
    second.write_text(
        "define\nA sub entity, owns x;\nx sub attribute, value string;",
        encoding="utf-8",
    )

    merged = parse_schema_files([first, second])

    assert merged.types["A"].owns == {"x"}
    assert merged.types["x"].kind == "attribute"


def test_external_relation_node_uses_same_relation_symbol_style():
    schema = parse_schema_text(
        """
define
Node sub entity, plays shared_relation:provider;
"""
    )

    graph_dot = build_graph(schema).to_string()

    assert "shared_relation" in graph_dot
    assert "shape=diamond" in graph_dot
    assert "rounded,filled" in graph_dot
    assert "dotted,filled" not in graph_dot
