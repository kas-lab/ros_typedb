"""Utilities to parse TypeDB rules and generate rule dependency/read-write graphs."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import re
from typing import Dict, Iterable, List, Optional, Set, Tuple

import pydot


@dataclass
class RuleEntry:
    """Represents one TypeDB rule with extracted read/write concepts."""

    name: str
    reads: Set[str] = field(default_factory=set)
    writes: Set[str] = field(default_factory=set)


@dataclass
class RuleModel:
    """Represents a parsed set of TypeDB rules."""

    rules: Dict[str, RuleEntry] = field(default_factory=dict)


def parse_rule_file(schema_path: Path) -> RuleModel:
    """Parse a schema/rules file and return an extracted rule model."""
    return parse_rule_text(Path(schema_path).read_text(encoding="utf-8"))


def parse_rule_files(schema_paths: Iterable[Path]) -> RuleModel:
    """Parse and merge rules from multiple files into one model."""
    normalized_paths = [Path(path) for path in schema_paths]
    if not normalized_paths:
        raise ValueError("No rule files provided for merge.")

    merged = RuleModel()
    for path in normalized_paths:
        model = parse_rule_file(path)
        for name, entry in model.rules.items():
            if name in merged.rules:
                raise ValueError(f"Duplicate rule definition encountered: {name}")
            merged.rules[name] = RuleEntry(name=name, reads=set(entry.reads), writes=set(entry.writes))
    return merged


def parse_rule_text(text: str) -> RuleModel:
    """Parse TypeDB text and extract rule read/write concepts."""
    cleaned = _strip_line_comments(text)
    model = RuleModel()

    for name, when_block, then_block in _extract_rule_blocks(cleaned):
        entry = model.rules.setdefault(name, RuleEntry(name=name))
        entry.reads.update(_extract_concepts_from_block(when_block))
        entry.writes.update(_extract_concepts_from_block(then_block))

    return model


def default_rule_output_path(input_path: Path, output_format: str) -> Path:
    """Compute default output path for rule diagrams."""
    extension = _output_extension(output_format)
    input_path = Path(input_path)

    if "schemas" in input_path.parts:
        schemas_index = input_path.parts.index("schemas")
        root = Path(*input_path.parts[:schemas_index]) if schemas_index > 0 else Path(".")
        return root / "schemas_diagrams" / f"{input_path.stem}_rules.{extension}"

    return input_path.with_name(f"{input_path.stem}_rules.{extension}")


def build_rule_graph(
    model: RuleModel,
    include_rule_dependencies: bool = True,
    orientation: str = "vertical",
) -> pydot.Dot:
    """Build a graph showing rule read/write and optional rule dependencies."""
    rankdir = _orientation_to_rankdir(orientation)
    graph = pydot.Dot(
        "typedb_rule_graph",
        graph_type="digraph",
        rankdir=rankdir,
        fontsize="10",
        fontname="Helvetica",
    )

    concepts = sorted(_all_concepts(model))
    for concept in concepts:
        graph.add_node(
            pydot.Node(
                _concept_node_id(concept),
                label=concept,
                shape="ellipse",
                style="filled",
                fillcolor="#EEF2F7",
                color="#9BA7B6",
            )
        )

    for rule_name in sorted(model.rules):
        graph.add_node(
            pydot.Node(
                _rule_node_id(rule_name),
                label=f"rule: {rule_name}",
                shape="box",
                style="rounded,filled",
                fillcolor="#F9F0D8",
                color="#C8A96B",
            )
        )

    for rule_name in sorted(model.rules):
        entry = model.rules[rule_name]
        for concept in sorted(entry.reads):
            graph.add_edge(
                pydot.Edge(
                    _concept_node_id(concept),
                    _rule_node_id(rule_name),
                    color="#4C78A8",
                    label="read",
                    fontsize="8",
                )
            )
        for concept in sorted(entry.writes):
            graph.add_edge(
                pydot.Edge(
                    _rule_node_id(rule_name),
                    _concept_node_id(concept),
                    color="#54A24B",
                    label="write",
                    fontsize="8",
                )
            )

    if include_rule_dependencies:
        for source_name in sorted(model.rules):
            source = model.rules[source_name]
            for target_name in sorted(model.rules):
                if source_name == target_name:
                    continue
                target = model.rules[target_name]
                shared = source.writes & target.reads
                if not shared:
                    continue
                graph.add_edge(
                    pydot.Edge(
                        _rule_node_id(source_name),
                        _rule_node_id(target_name),
                        style="dashed",
                        color="#8E8E8E",
                        label=_dependency_label(shared),
                        fontsize="8",
                    )
                )

    return graph


def write_rule_graph(
    model: RuleModel,
    output_path: Path,
    output_format: str,
    include_rule_dependencies: bool = True,
    orientation: str = "vertical",
) -> Path:
    """Render a rule graph to DOT or SVG and return output path."""
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    graph = build_rule_graph(
        model,
        include_rule_dependencies=include_rule_dependencies,
        orientation=orientation,
    )

    if output_format == "dot":
        output_path.write_text(graph.to_string(), encoding="utf-8")
    elif output_format == "svg":
        graph.write_svg(str(output_path))
    else:
        raise ValueError(f"Unsupported output format: {output_format}")

    return output_path


def _dependency_label(shared_concepts: Set[str]) -> str:
    ordered = sorted(shared_concepts)
    if len(ordered) <= 2:
        return "via " + ", ".join(ordered)
    return f"via {ordered[0]}, {ordered[1]} (+{len(ordered) - 2})"


def _all_concepts(model: RuleModel) -> Set[str]:
    concepts: Set[str] = set()
    for entry in model.rules.values():
        concepts.update(entry.reads)
        concepts.update(entry.writes)
    return concepts


def _extract_rule_blocks(text: str) -> Iterable[Tuple[str, str, str]]:
    cursor = 0
    while cursor < len(text):
        rule_match = re.search(r"\brule\s+([A-Za-z_][\w-]*)\s*:", text[cursor:], flags=re.IGNORECASE)
        if rule_match is None:
            return

        rule_name = rule_match.group(1)
        rule_start = cursor + rule_match.end()

        when_match = re.search(r"\bwhen\b", text[rule_start:], flags=re.IGNORECASE)
        if when_match is None:
            cursor = rule_start
            continue
        when_keyword_end = rule_start + when_match.end()
        when_open = text.find("{", when_keyword_end)
        if when_open < 0:
            cursor = when_keyword_end
            continue
        when_block, after_when = _extract_braced_block(text, when_open)

        then_match = re.search(r"\bthen\b", text[after_when:], flags=re.IGNORECASE)
        if then_match is None:
            cursor = after_when
            continue
        then_keyword_end = after_when + then_match.end()
        then_open = text.find("{", then_keyword_end)
        if then_open < 0:
            cursor = then_keyword_end
            continue
        then_block, after_then = _extract_braced_block(text, then_open)

        yield rule_name, when_block, then_block
        cursor = after_then


def _extract_braced_block(text: str, open_index: int) -> Tuple[str, int]:
    if open_index < 0 or open_index >= len(text) or text[open_index] != "{":
        raise ValueError("Invalid block opening index.")

    depth = 0
    quote_char: Optional[str] = None
    escaped = False
    content_start = open_index + 1

    for index in range(open_index, len(text)):
        character = text[index]

        if quote_char is not None:
            if escaped:
                escaped = False
            elif character == "\\":
                escaped = True
            elif character == quote_char:
                quote_char = None
            continue

        if character in {'"', "'"}:
            quote_char = character
            continue

        if character == "{":
            depth += 1
            continue
        if character == "}":
            depth -= 1
            if depth == 0:
                return text[content_start:index], index + 1

    raise ValueError("Unbalanced braces while parsing rule block.")


def _extract_concepts_from_block(block_text: str) -> Set[str]:
    block_text = _strip_quoted_content(block_text)
    concepts: Set[str] = set()
    concepts.update(re.findall(r"\bisa!?\s+([A-Za-z_][\w-]*)", block_text))
    concepts.update(re.findall(r"\bhas\s+([A-Za-z_][\w-]*)", block_text))
    return concepts


def _strip_quoted_content(text: str) -> str:
    result: List[str] = []
    quote_char: Optional[str] = None
    escaped = False

    for character in text:
        if quote_char is not None:
            if escaped:
                escaped = False
            elif character == "\\":
                escaped = True
            elif character == quote_char:
                quote_char = None
            elif character == "\n":
                result.append(character)
            continue

        if character in {'"', "'"}:
            quote_char = character
            continue

        result.append(character)

    return "".join(result)


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


def _orientation_to_rankdir(orientation: str) -> str:
    if orientation == "vertical":
        return "TB"
    if orientation == "horizontal":
        return "LR"
    raise ValueError(f"Unsupported orientation: {orientation}")


def _output_extension(output_format: str) -> str:
    if output_format not in {"dot", "svg"}:
        raise ValueError(f"Unsupported output format: {output_format}")
    return output_format


def _rule_node_id(rule_name: str) -> str:
    return f"rule__{rule_name}"


def _concept_node_id(concept_name: str) -> str:
    return f"concept__{concept_name}"
