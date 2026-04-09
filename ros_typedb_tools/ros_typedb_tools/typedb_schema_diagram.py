"""CLI for generating diagrams from TypeDB 3 schema files."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

try:
    from ros_typedb_tools.schema_diagram import default_output_path
    from ros_typedb_tools.schema_diagram import parse_schema_file
    from ros_typedb_tools.schema_diagram import parse_schema_files
    from ros_typedb_tools.schema_diagram import write_graph
except ImportError:  # pragma: no cover - exercised via subprocess test
    from schema_diagram import default_output_path
    from schema_diagram import parse_schema_file
    from schema_diagram import parse_schema_files
    from schema_diagram import write_graph


def build_argument_parser() -> argparse.ArgumentParser:
    """Build command-line arguments for the schema diagram tool."""
    parser = argparse.ArgumentParser(
        prog="typedb_schema_diagram",
        description="Generate Graphviz diagrams from a TypeDB schema file.",
    )
    parser.add_argument(
        "--input",
        "-i",
        required=True,
        nargs="+",
        help="Path(s) to TypeDB schema .tql files. Multiple paths are merged into one diagram.",
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
        "--include-functions",
        "--include-rules",
        dest="include_functions",
        action="store_true",
        help="Include TypeDB function names as note nodes.",
    )
    parser.add_argument(
        "--orientation",
        choices=["vertical", "horizontal"],
        default="vertical",
        help="Hierarchy orientation. Defaults to vertical.",
    )
    return parser


def _default_output_path_for_inputs(input_paths: list[Path], output_format: str) -> Path:
    if len(input_paths) == 1:
        return default_output_path(input_paths[0], output_format)

    common_parent = Path(os.path.commonpath([str(path.resolve().parent) for path in input_paths]))
    if "schemas" in common_parent.parts:
        schemas_index = common_parent.parts.index("schemas")
        root = Path(*common_parent.parts[:schemas_index]) if schemas_index > 0 else Path(".")
        return root / "schemas_diagrams" / f"merged_schema.{output_format}"
    return common_parent / f"merged_schema.{output_format}"


def main(argv: list[str] | None = None) -> int:
    """Run the schema diagram generator CLI."""
    parser = build_argument_parser()
    args = parser.parse_args(argv)

    input_paths = [Path(input_path) for input_path in args.input]
    missing_paths = [str(input_path) for input_path in input_paths if not input_path.is_file()]
    if missing_paths:
        print(f"Input schema file not found: {missing_paths[0]}", file=sys.stderr)
        return 2

    try:
        output_path = (
            Path(args.output)
            if args.output
            else _default_output_path_for_inputs(input_paths, args.format)
        )
        if len(input_paths) == 1:
            schema_model = parse_schema_file(input_paths[0])
        else:
            schema_model = parse_schema_files(input_paths)
        write_graph(
            schema_model,
            output_path=output_path,
            output_format=args.format,
            include_functions=args.include_functions,
            orientation=args.orientation,
        )
    except Exception as exc:  # pragma: no cover - guarded by dedicated tests
        print(f"Failed to generate schema diagram: {exc}", file=sys.stderr)
        return 1

    print(f"Wrote diagram: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
