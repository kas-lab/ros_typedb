"""Utilities to parse TypeDB 3 functions and generate dependency/read graphs."""

from __future__ import annotations

from dataclasses import dataclass
from dataclasses import field
from pathlib import Path
import re
from typing import Dict
from typing import Iterable
from typing import Iterator
from typing import List
from typing import Optional
from typing import Set
from typing import Tuple

import pydot


_FUNCTION_HEADER_RE = re.compile(
    r"(?m)^\s*fun\s+([A-Za-z_][\w-]*)\s*\((.*?)\)\s*->\s*(.*?)\s*:\s*$",
    flags=re.DOTALL,
)
_TYPE_NAME_RE = re.compile(r"(?<![$])\b([A-Za-z_][\w-]*)\b")
_BUILTIN_CALL_NAMES = {
    "abs",
    "ceil",
    "concat",
    "count",
    "floor",
    "iid",
    "label",
    "len",
    "list",
    "max",
    "mean",
    "median",
    "min",
    "round",
    "std",
    "sum",
}


@dataclass
class FunctionEntry:
    """Represents one TypeDB 3 function with extracted reads, returns, and calls."""

    name: str
    reads: Set[str] = field(default_factory=set)
    returns: Set[str] = field(default_factory=set)
    calls: Set[str] = field(default_factory=set)


RuleEntry = FunctionEntry


@dataclass
class FunctionModel:
    """Represents a parsed set of TypeDB 3 functions."""

    rules: Dict[str, FunctionEntry] = field(default_factory=dict)


RuleModel = FunctionModel


def parse_rule_file(schema_path: Path) -> FunctionModel:
    """Parse a TypeDB file and return an extracted function model."""
    return parse_rule_text(Path(schema_path).read_text(encoding="utf-8"))


def parse_rule_files(schema_paths: Iterable[Path]) -> FunctionModel:
    """Parse and merge functions from multiple files into one model."""
    normalized_paths = [Path(path) for path in schema_paths]
    if not normalized_paths:
        raise ValueError("No function files provided for merge.")

    cleaned_texts: List[str] = []
    seen_function_names: Set[str] = set()
    for path in normalized_paths:
        text = Path(path).read_text(encoding="utf-8")
        cleaned_text = _strip_line_comments(text)
        for name, _, _, _ in _extract_function_blocks(cleaned_text):
            if name in seen_function_names:
                raise ValueError(f"Duplicate function definition encountered: {name}")
            seen_function_names.add(name)
        cleaned_texts.append(cleaned_text)

    return parse_rule_text("\n\n".join(cleaned_texts))


def parse_rule_text(text: str) -> FunctionModel:
    """Parse TypeDB 3 text and extract function reads, returns, and call chains."""
    cleaned = _strip_line_comments(text)
    blocks = list(_extract_function_blocks(cleaned))
    known_function_names = {name for name, _, _, _ in blocks}
    model = FunctionModel()

    for name, arguments, return_type, body in blocks:
        entry = model.rules.setdefault(name, FunctionEntry(name=name))
        entry.reads.update(_extract_types(arguments))
        entry.reads.update(_extract_concepts_from_block(body))
        entry.returns.update(_extract_types(return_type))
        entry.calls.update(_extract_called_functions(body, known_function_names))

    for entry in model.rules.values():
        for called_function in entry.calls:
            if called_function in model.rules:
                entry.reads.update(model.rules[called_function].reads)

    return model


def default_rule_output_path(input_path: Path, output_format: str) -> Path:
    """Compute default output path for function dependency diagrams."""
    extension = _output_extension(output_format)
    input_path = Path(input_path)

    if "schemas" in input_path.parts:
        schemas_index = input_path.parts.index("schemas")
        root = Path(*input_path.parts[:schemas_index]) if schemas_index > 0 else Path(".")
        return root / "schemas_diagrams" / f"{input_path.stem}_rules.{extension}"

    return input_path.with_name(f"{input_path.stem}_rules.{extension}")


def build_rule_graph(
    model: FunctionModel,
    include_rule_dependencies: bool = True,
    orientation: str = "vertical",
) -> pydot.Dot:
    """Build a graph showing function reads, returns, and optional call dependencies."""
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

    for function_name in sorted(model.rules):
        graph.add_node(
            pydot.Node(
                _function_node_id(function_name),
                label=f"fun: {function_name}",
                shape="box",
                style='"rounded,filled"',
                fillcolor="#F9F0D8",
                color="#C8A96B",
            )
        )

    for function_name in sorted(model.rules):
        entry = model.rules[function_name]
        for concept in sorted(entry.reads):
            graph.add_edge(
                pydot.Edge(
                    _concept_node_id(concept),
                    _function_node_id(function_name),
                    color="#4C78A8",
                    label="read",
                    fontsize="8",
                )
            )
        for concept in sorted(entry.returns):
            graph.add_edge(
                pydot.Edge(
                    _function_node_id(function_name),
                    _concept_node_id(concept),
                    color="#54A24B",
                    label="return",
                    fontsize="8",
                )
            )

    if include_rule_dependencies:
        for function_name in sorted(model.rules):
            entry = model.rules[function_name]
            for called_function in sorted(entry.calls):
                graph.add_edge(
                    pydot.Edge(
                        _function_node_id(function_name),
                        _function_node_id(called_function),
                        style="dashed",
                        color="#8E8E8E",
                        label="calls",
                        fontsize="8",
                    )
                )

    return graph


def write_rule_graph(
    model: FunctionModel,
    output_path: Path,
    output_format: str,
    include_rule_dependencies: bool = True,
    orientation: str = "vertical",
) -> Path:
    """Render a function graph to DOT or SVG and return output path."""
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


def _all_concepts(model: FunctionModel) -> Set[str]:
    concepts: Set[str] = set()
    for entry in model.rules.values():
        concepts.update(entry.reads)
        concepts.update(entry.returns)
    return concepts


def _extract_function_blocks(text: str) -> Iterator[Tuple[str, str, str, str]]:
    cursor = 0
    while True:
        match = _FUNCTION_HEADER_RE.search(text, pos=cursor)
        if match is None:
            return

        name = match.group(1)
        arguments = match.group(2)
        return_type = match.group(3).strip()
        body_start = match.end()
        next_match = _FUNCTION_HEADER_RE.search(text, pos=body_start)
        body_end = next_match.start() if next_match is not None else len(text)
        body = text[body_start:body_end].strip()
        yield name, arguments, return_type, body
        cursor = body_end


def _extract_concepts_from_block(block_text: str) -> Set[str]:
    block_text = _strip_quoted_content(block_text)
    concepts: Set[str] = set()
    concepts.update(re.findall(r"\bisa!?\s+([A-Za-z_][\w-]*)", block_text))
    concepts.update(re.findall(r"\bhas\s+([A-Za-z_][\w-]*)", block_text))
    return concepts


def _extract_called_functions(block_text: str, known_function_names: Set[str]) -> Set[str]:
    block_text = _strip_quoted_content(block_text)
    called_identifiers = re.findall(r"\b([A-Za-z_][\w-]*)\s*\(", block_text)
    return {
        identifier
        for identifier in called_identifiers
        if identifier in known_function_names and identifier not in _BUILTIN_CALL_NAMES
    }


def _extract_types(signature_fragment: str) -> Set[str]:
    return {
        match
        for match in _TYPE_NAME_RE.findall(signature_fragment)
        if match not in _BUILTIN_CALL_NAMES
    }


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


def _function_node_id(function_name: str) -> str:
    return f"fun__{function_name}"


def _concept_node_id(concept_name: str) -> str:
    return f"concept__{concept_name}"
