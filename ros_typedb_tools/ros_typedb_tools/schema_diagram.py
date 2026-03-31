"""Utilities to parse TypeDB schemas and generate diagram graphs."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import re
from typing import Dict, Iterable, List, Optional, Set, Tuple

import pydot


_ROOT_KINDS: Set[str] = {"entity", "relation", "attribute"}


@dataclass(frozen=True)
class SchemaRule:
    """Represents a rule declared in a TypeDB schema."""

    name: str


@dataclass
class SchemaType:
    """Represents a TypeDB schema type definition."""

    name: str
    kind: str
    parent: Optional[str] = None
    owns: Set[str] = field(default_factory=set)
    plays: Set[Tuple[str, str]] = field(default_factory=set)
    relates: Set[str] = field(default_factory=set)
    value_type: Optional[str] = None


@dataclass
class SchemaModel:
    """Represents a parsed TypeDB schema model."""

    types: Dict[str, SchemaType] = field(default_factory=dict)
    rules: List[SchemaRule] = field(default_factory=list)


def parse_schema_file(schema_path: Path) -> SchemaModel:
    """Parse a schema file and return a schema model."""
    return parse_schema_text(schema_path.read_text(encoding="utf-8"))


def parse_schema_files(schema_paths: Iterable[Path]) -> SchemaModel:
    """Parse and merge multiple schema files into a single schema model."""
    normalized_paths = [Path(path) for path in schema_paths]
    if not normalized_paths:
        raise ValueError("No schema files provided for merge.")

    models = [parse_schema_file(path) for path in normalized_paths]
    return merge_schema_models(models)


def parse_schema_text(text: str) -> SchemaModel:
    """Parse TypeDB schema text and extract diagram-relevant structures."""
    cleaned_text = _strip_line_comments(text)
    statements = _split_top_level_statements(cleaned_text)

    model = SchemaModel()
    for statement in statements:
        statement = statement.strip()
        if not statement:
            continue

        if statement.lower().startswith("define"):
            statement = statement[6:].strip()
            if not statement:
                continue

        rule_match = re.match(r"^rule\s+([A-Za-z_][\w-]*)\s*:", statement, flags=re.IGNORECASE)
        if rule_match:
            model.rules.append(SchemaRule(name=rule_match.group(1)))
            continue

        type_match = re.match(
            r"^([A-Za-z_][\w-]*)\s+sub\s+([A-Za-z_][\w-]*)(.*)$",
            statement,
            flags=re.DOTALL,
        )
        if type_match:
            type_name = type_match.group(1)
            parent_name = type_match.group(2)
            remainder = type_match.group(3)

            schema_type = model.types.setdefault(type_name, SchemaType(name=type_name, kind="unknown"))
            if parent_name in _ROOT_KINDS:
                schema_type.kind = parent_name
                schema_type.parent = None
            else:
                schema_type.parent = parent_name

            value_type = _extract_value_type(remainder)
            if value_type is not None:
                schema_type.value_type = value_type

            schema_type.owns.update(_extract_owns(remainder))
            schema_type.plays.update(_extract_plays(remainder))
            schema_type.relates.update(_extract_relates(remainder))
            continue

        standalone_plays_match = re.match(r"^([A-Za-z_][\w-]*)\s+(.*)$", statement, flags=re.DOTALL)
        if standalone_plays_match:
            type_name = standalone_plays_match.group(1)
            remainder = standalone_plays_match.group(2)
            plays = set(_extract_plays(remainder))
            if plays:
                schema_type = model.types.setdefault(type_name, SchemaType(name=type_name, kind="unknown"))
                schema_type.plays.update(plays)

    _infer_kinds(model.types)
    return model


def merge_schema_models(models: Iterable[SchemaModel]) -> SchemaModel:
    """Merge multiple schema models into one, raising on incompatible conflicts."""
    merged = SchemaModel()
    rule_names: Set[str] = set()

    for model in models:
        for type_name, schema_type in model.types.items():
            existing = merged.types.get(type_name)
            if existing is None:
                merged.types[type_name] = SchemaType(
                    name=schema_type.name,
                    kind=schema_type.kind,
                    parent=schema_type.parent,
                    owns=set(schema_type.owns),
                    plays=set(schema_type.plays),
                    relates=set(schema_type.relates),
                    value_type=schema_type.value_type,
                )
                continue

            current_kind = existing.kind
            incoming_kind = schema_type.kind
            existing.kind = _merge_kind(existing.name, current_kind, incoming_kind)
            existing.parent = _merge_parent(
                existing.name,
                existing.parent,
                schema_type.parent,
                current_kind,
                incoming_kind,
            )
            existing.value_type = _merge_optional_field(
                existing.name,
                "value_type",
                existing.value_type,
                schema_type.value_type,
            )
            existing.owns.update(schema_type.owns)
            existing.plays.update(schema_type.plays)
            existing.relates.update(schema_type.relates)

        for rule in model.rules:
            if rule.name in rule_names:
                continue
            rule_names.add(rule.name)
            merged.rules.append(SchemaRule(name=rule.name))

    _infer_kinds(merged.types)
    return merged


def default_output_path(input_path: Path, output_format: str) -> Path:
    """Compute the default output path for the requested output format."""
    extension = _output_extension(output_format)
    input_path = Path(input_path)

    if "schemas" in input_path.parts:
        schemas_index = input_path.parts.index("schemas")
        root = Path(*input_path.parts[:schemas_index]) if schemas_index > 0 else Path(".")
        return root / "schemas_diagrams" / f"{input_path.stem}.{extension}"

    return input_path.with_suffix(f".{extension}")


def build_graph(schema: SchemaModel, include_rules: bool = False, orientation: str = "vertical") -> pydot.Dot:
    """Build a Graphviz graph from a parsed schema model."""
    rankdir = _orientation_to_rankdir(orientation)
    graph = pydot.Dot(
        "typedb_schema",
        graph_type="digraph",
        rankdir=rankdir,
        fontsize="10",
    )

    for schema_type in _sorted_types(schema.types.values()):
        graph.add_node(_create_type_node(schema_type))

    for schema_type in _sorted_types(schema.types.values()):
        if schema_type.parent and schema_type.parent in schema.types:
            graph.add_edge(
                pydot.Edge(
                    schema_type.name,
                    schema_type.parent,
                    label="sub",
                    color="#5E81AC",
                    arrowhead="empty",
                )
            )

        for owned_attribute in sorted(schema_type.owns):
            if owned_attribute not in schema.types:
                graph.add_node(_create_external_attribute_node(owned_attribute))
            graph.add_edge(
                pydot.Edge(
                    schema_type.name,
                    owned_attribute,
                    label="owns",
                    style="dashed",
                    color="#81A1C1",
                )
            )

        for relation_name, role_name in sorted(schema_type.plays):
            if relation_name not in schema.types:
                graph.add_node(_create_external_relation_node(relation_name))
            graph.add_edge(
                pydot.Edge(
                    schema_type.name,
                    relation_name,
                    label=f"plays {role_name}",
                    color="#A3BE8C",
                )
            )

    if include_rules:
        for rule in sorted(schema.rules, key=lambda item: item.name):
            rule_node_name = f"rule__{rule.name}"
            graph.add_node(
                pydot.Node(
                    rule_node_name,
                    label=f"rule: {rule.name}",
                    shape="note",
                    style="filled",
                    fillcolor="#F7F3D2",
                    color="#C0B66F",
                )
            )

    return graph


def write_graph(
    schema: SchemaModel,
    output_path: Path,
    output_format: str,
    include_rules: bool = False,
    orientation: str = "vertical",
) -> Path:
    """Render a schema graph to DOT or SVG and return the output path."""
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    graph = build_graph(schema, include_rules=include_rules, orientation=orientation)
    if output_format == "dot":
        output_path.write_text(graph.to_string(), encoding="utf-8")
    elif output_format == "svg":
        graph.write_svg(str(output_path))
    else:
        raise ValueError(f"Unsupported output format: {output_format}")

    return output_path


def _output_extension(output_format: str) -> str:
    if output_format not in {"dot", "svg"}:
        raise ValueError(f"Unsupported output format: {output_format}")
    return output_format


def _orientation_to_rankdir(orientation: str) -> str:
    if orientation == "vertical":
        return "TB"
    if orientation == "horizontal":
        return "LR"
    raise ValueError(f"Unsupported orientation: {orientation}")


def _strip_line_comments(text: str) -> str:
    result: List[str] = []
    in_comment = False
    quote_char: Optional[str] = None
    escaped = False

    for character in text:
        if in_comment:
            if character == "\n":
                in_comment = False
                result.append(character)
            continue

        if quote_char is not None:
            result.append(character)
            if escaped:
                escaped = False
            elif character == "\\":
                escaped = True
            elif character == quote_char:
                quote_char = None
            continue

        if character in {'"', "'"}:
            quote_char = character
            result.append(character)
            continue

        if character == "#":
            in_comment = True
            continue

        result.append(character)

    return "".join(result)


def _split_top_level_statements(text: str) -> List[str]:
    statements: List[str] = []
    buffer: List[str] = []
    brace_depth = 0

    for character in text:
        if character == "{":
            brace_depth += 1
        elif character == "}":
            brace_depth = max(0, brace_depth - 1)

        if character == ";" and brace_depth == 0:
            statement = "".join(buffer).strip()
            if statement:
                statements.append(statement)
            buffer = []
            continue

        buffer.append(character)

    trailing = "".join(buffer).strip()
    if trailing:
        statements.append(trailing)

    return statements


def _extract_owns(statement_suffix: str) -> Iterable[str]:
    return re.findall(r"\bowns\s+([A-Za-z_][\w-]*)", statement_suffix)


def _extract_plays(statement_suffix: str) -> Iterable[Tuple[str, str]]:
    return re.findall(r"\bplays\s+([A-Za-z_][\w-]*):([A-Za-z_][\w-]*)", statement_suffix)


def _extract_relates(statement_suffix: str) -> Iterable[str]:
    return re.findall(
        r"\brelates\s+([A-Za-z_][\w-]*)(?:\s+as\s+[A-Za-z_][\w-]*)?",
        statement_suffix,
    )


def _extract_value_type(statement_suffix: str) -> Optional[str]:
    value_match = re.search(r"\bvalue\s+([A-Za-z_][\w-]*)", statement_suffix)
    return value_match.group(1) if value_match else None


def _infer_kinds(types: Dict[str, SchemaType]) -> None:
    changed = True
    while changed:
        changed = False
        for schema_type in types.values():
            if schema_type.kind != "unknown" or not schema_type.parent:
                continue
            parent = types.get(schema_type.parent)
            if parent and parent.kind != "unknown":
                schema_type.kind = parent.kind
                changed = True


def _merge_kind(type_name: str, current_kind: str, incoming_kind: str) -> str:
    if current_kind == incoming_kind:
        return current_kind
    if current_kind == "unknown":
        return incoming_kind
    if incoming_kind == "unknown":
        return current_kind
    raise ValueError(
        f"Conflicting kind definitions for type '{type_name}': "
        f"'{current_kind}' and '{incoming_kind}'."
    )


def _merge_optional_field(type_name: str, field_name: str, current_value: Optional[str], incoming_value: Optional[str]) -> Optional[str]:
    if current_value is None:
        return incoming_value
    if incoming_value is None:
        return current_value
    if current_value != incoming_value:
        raise ValueError(
            f"Conflicting {field_name} definitions for type '{type_name}': "
            f"'{current_value}' and '{incoming_value}'."
        )
    return current_value


def _merge_parent(
    type_name: str,
    current_parent: Optional[str],
    incoming_parent: Optional[str],
    current_kind: str,
    incoming_kind: str,
) -> Optional[str]:
    if current_parent == incoming_parent:
        return current_parent
    if current_kind == "unknown" and incoming_kind == "unknown":
        if current_parent is None:
            return incoming_parent
        if incoming_parent is None:
            return current_parent
    if current_kind == "unknown" and incoming_kind != "unknown":
        return incoming_parent
    if incoming_kind == "unknown" and current_kind != "unknown":
        return current_parent
    raise ValueError(
        f"Conflicting parent definitions for type '{type_name}': "
        f"'{current_parent}' and '{incoming_parent}'."
    )


def _sorted_types(schema_types: Iterable[SchemaType]) -> List[SchemaType]:
    return sorted(schema_types, key=lambda item: item.name)


def _create_type_node(schema_type: SchemaType) -> pydot.Node:
    label = schema_type.name
    shape = "box"
    fillcolor = "#E5EEFC"
    color = "#6C8EBF"

    if schema_type.kind == "relation":
        shape = "diamond"
        fillcolor = "#FFF6E3"
        color = "#D6B656"
        if schema_type.relates:
            sorted_roles = ", ".join(sorted(schema_type.relates))
            label = f"{schema_type.name}\\nroles: {sorted_roles}"
    elif schema_type.kind == "attribute":
        shape = "ellipse"
        fillcolor = "#E2E8E1"
        color = "#82B366"
        if schema_type.value_type:
            label = f"{schema_type.name}\\nvalue: {schema_type.value_type}"
    elif schema_type.kind == "unknown":
        fillcolor = "#F4F4F4"
        color = "#999999"

    return pydot.Node(
        schema_type.name,
        label=label,
        shape=shape,
        style="rounded,filled",
        fillcolor=fillcolor,
        color=color,
    )


def _create_external_attribute_node(name: str) -> pydot.Node:
    return pydot.Node(
        name,
        label=name,
        shape="ellipse",
        style="dotted,filled",
        fillcolor="#F0F7EF",
        color="#82B366",
    )


def _create_external_relation_node(name: str) -> pydot.Node:
    return pydot.Node(
        name,
        label=name,
        shape="diamond",
        style="rounded,filled",
        fillcolor="#FFF6E3",
        color="#D6B656",
    )
