"""CLI for generating rule dependency/read-write diagrams from TypeDB files."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

try:
    from ros_typedb_tools.rule_diagram import default_rule_output_path
    from ros_typedb_tools.rule_diagram import parse_rule_file
    from ros_typedb_tools.rule_diagram import parse_rule_files
    from ros_typedb_tools.rule_diagram import write_rule_graph
except ImportError:  # pragma: no cover - exercised via subprocess test
    from rule_diagram import default_rule_output_path
    from rule_diagram import parse_rule_file
    from rule_diagram import parse_rule_files
    from rule_diagram import write_rule_graph


def build_argument_parser() -> argparse.ArgumentParser:
    """Build command-line args for the TypeDB rule diagram tool."""
    parser = argparse.ArgumentParser(
        prog="typedb_rule_diagram",
        description="Generate rule dependency/read-write diagrams from TypeDB .tql files.",
    )
    parser.add_argument(
        "--input",
        "-i",
        required=True,
        nargs="+",
        help="Path(s) to TypeDB .tql files. Multiple paths are merged.",
    )
    parser.add_argument(
        "--output",
        "-o",
        help="Output path. If omitted, defaults next to input or in sibling schemas_diagrams.",
    )
    parser.add_argument(
        "--format",
        "-f",
        choices=["dot", "svg"],
        default="svg",
        help="Output format. Defaults to svg.",
    )
    parser.add_argument(
        "--orientation",
        choices=["vertical", "horizontal"],
        default="vertical",
        help="Diagram orientation. Defaults to vertical.",
    )
    parser.add_argument(
        "--no-rule-dependencies",
        action="store_true",
        help="Disable rule-to-rule dependency edges.",
    )
    return parser


def _default_output_path_for_inputs(input_paths: list[Path], output_format: str) -> Path:
    if len(input_paths) == 1:
        return default_rule_output_path(input_paths[0], output_format)

    common_parent = Path(os.path.commonpath([str(path.resolve().parent) for path in input_paths]))
    if "schemas" in common_parent.parts:
        schemas_index = common_parent.parts.index("schemas")
        root = Path(*common_parent.parts[:schemas_index]) if schemas_index > 0 else Path(".")
        return root / "schemas_diagrams" / f"merged_rules.{output_format}"
    return common_parent / f"merged_rules.{output_format}"


def main(argv: list[str] | None = None) -> int:
    """Run the TypeDB rule diagram CLI."""
    parser = build_argument_parser()
    args = parser.parse_args(argv)

    input_paths = [Path(path) for path in args.input]
    missing_paths = [str(path) for path in input_paths if not path.is_file()]
    if missing_paths:
        print(f"Input rule file not found: {missing_paths[0]}", file=sys.stderr)
        return 2

    try:
        output_path = Path(args.output) if args.output else _default_output_path_for_inputs(input_paths, args.format)
        if len(input_paths) == 1:
            model = parse_rule_file(input_paths[0])
        else:
            model = parse_rule_files(input_paths)

        write_rule_graph(
            model,
            output_path=output_path,
            output_format=args.format,
            include_rule_dependencies=not args.no_rule_dependencies,
            orientation=args.orientation,
        )
    except Exception as exc:  # pragma: no cover - guarded by tests
        print(f"Failed to generate rule diagram: {exc}", file=sys.stderr)
        return 1

    print(f"Wrote rule diagram: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
