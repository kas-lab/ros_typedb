"""Tests for the typedb_schema_diagram CLI behavior."""

import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

from ros_typedb_tools.schema_diagram import default_output_path
from ros_typedb_tools.typedb_schema_diagram import main


def _fixture_schema_path() -> Path:
    return Path(__file__).parent / "fixtures" / "mock_schema.tql"


def test_default_output_path_prefers_sibling_schemas_diagrams():
    input_path = Path("/tmp/workspace/schemas/domain/mock_schema.tql")
    expected = Path("/tmp/workspace/schemas_diagrams/mock_schema.svg")
    assert default_output_path(input_path, "svg") == expected


def test_default_output_path_next_to_input_when_not_in_schemas():
    input_path = Path("/tmp/workspace/my_model.tql")
    expected = Path("/tmp/workspace/my_model.dot")
    assert default_output_path(input_path, "dot") == expected


def test_cli_writes_dot_output_to_default_path(tmp_path: Path):
    schema_root = tmp_path / "project" / "schemas" / "demo"
    schema_root.mkdir(parents=True, exist_ok=True)
    schema_path = schema_root / "mock_schema.tql"
    schema_path.write_text(_fixture_schema_path().read_text(encoding="utf-8"), encoding="utf-8")

    rc = main(["--input", str(schema_path), "--format", "dot"])
    assert rc == 0

    output_path = tmp_path / "project" / "schemas_diagrams" / "mock_schema.dot"
    assert output_path.exists()
    dot_text = output_path.read_text(encoding="utf-8")
    assert "digraph typedb_schema" in dot_text
    assert "rankdir=TB" in dot_text


def test_cli_honors_explicit_output_path(tmp_path: Path):
    schema_path = tmp_path / "local_schema.tql"
    schema_path.write_text(_fixture_schema_path().read_text(encoding="utf-8"), encoding="utf-8")

    output_path = tmp_path / "custom" / "diagram.dot"
    rc = main(
        [
            "--input",
            str(schema_path),
            "--format",
            "dot",
            "--output",
            str(output_path),
            "--include-functions",
        ]
    )
    assert rc == 0
    assert output_path.exists()
    output_text = output_path.read_text(encoding="utf-8")
    assert "function__active_server_name" in output_text
    assert "function__connected_clients" in output_text


def test_cli_honors_horizontal_orientation(tmp_path: Path):
    schema_path = tmp_path / "local_schema.tql"
    schema_path.write_text(_fixture_schema_path().read_text(encoding="utf-8"), encoding="utf-8")

    output_path = tmp_path / "horizontal.dot"
    rc = main(
        [
            "--input",
            str(schema_path),
            "--format",
            "dot",
            "--output",
            str(output_path),
            "--orientation",
            "horizontal",
        ]
    )

    assert rc == 0
    dot_text = output_path.read_text(encoding="utf-8")
    assert "rankdir=LR" in dot_text


@pytest.mark.skipif(shutil.which("dot") is None, reason="Graphviz dot executable is not available")
def test_cli_writes_svg_output(tmp_path: Path):
    schema_path = tmp_path / "schema.tql"
    schema_path.write_text(_fixture_schema_path().read_text(encoding="utf-8"), encoding="utf-8")

    output_path = tmp_path / "schema.svg"
    rc = main(
        [
            "--input",
            str(schema_path),
            "--format",
            "svg",
            "--output",
            str(output_path),
        ]
    )
    assert rc == 0
    assert output_path.exists()
    assert "<svg" in output_path.read_text(encoding="utf-8")


def test_cli_returns_error_for_missing_input(tmp_path: Path):
    missing_path = tmp_path / "missing_schema.tql"
    rc = main(["--input", str(missing_path), "--format", "dot"])
    assert rc != 0


def test_cli_merges_multiple_input_files(tmp_path: Path):
    first_schema = tmp_path / "schemas" / "a" / "first.tql"
    first_schema.parent.mkdir(parents=True, exist_ok=True)
    first_schema.write_text(
        (
            "define\nservice sub relation, relates provider;\n"
            "Node sub entity, plays service:provider;"
        ),
        encoding="utf-8",
    )

    second_schema = tmp_path / "schemas" / "b" / "second.tql"
    second_schema.parent.mkdir(parents=True, exist_ok=True)
    second_schema.write_text(
        "define\nname sub attribute, value string;\nNode sub entity, owns name;",
        encoding="utf-8",
    )

    output_path = tmp_path / "merged.dot"
    rc = main(
        [
            "--input",
            str(first_schema),
            str(second_schema),
            "--format",
            "dot",
            "--output",
            str(output_path),
        ]
    )

    assert rc == 0
    dot_text = output_path.read_text(encoding="utf-8")
    assert "Node" in dot_text
    assert "service" in dot_text
    assert "name" in dot_text
    assert "plays provider" in dot_text
    assert "label=owns" in dot_text


def test_cli_uses_default_merged_output_path_for_multiple_inputs(tmp_path: Path):
    first_schema = tmp_path / "workspace" / "schemas" / "a" / "first.tql"
    second_schema = tmp_path / "workspace" / "schemas" / "b" / "second.tql"
    first_schema.parent.mkdir(parents=True, exist_ok=True)
    second_schema.parent.mkdir(parents=True, exist_ok=True)
    first_schema.write_text("define\nA sub entity;", encoding="utf-8")
    second_schema.write_text("define\nB sub entity;", encoding="utf-8")

    rc = main(
        [
            "--input",
            str(first_schema),
            str(second_schema),
            "--format",
            "dot",
        ]
    )

    assert rc == 0
    assert (tmp_path / "workspace" / "schemas_diagrams" / "merged_schema.dot").exists()


def test_cli_merges_multiple_inputs_with_mixed_absolute_and_relative_paths(tmp_path: Path):
    workspace = tmp_path / "workspace"
    first_schema = workspace / "schemas" / "a" / "first.tql"
    second_schema = workspace / "schemas" / "b" / "second.tql"
    first_schema.parent.mkdir(parents=True, exist_ok=True)
    second_schema.parent.mkdir(parents=True, exist_ok=True)
    first_schema.write_text("define\nA sub entity;", encoding="utf-8")
    second_schema.write_text("define\nB sub entity;", encoding="utf-8")

    previous_cwd = Path.cwd()
    try:
        os.chdir(workspace)
        rc = main(
            [
                "--input",
                str(first_schema),
                str(Path("schemas") / "b" / "second.tql"),
                "--format",
                "dot",
            ]
        )
    finally:
        os.chdir(previous_cwd)

    assert rc == 0
    assert (workspace / "schemas_diagrams" / "merged_schema.dot").exists()


def test_direct_script_execution_writes_dot_output(tmp_path: Path):
    package_root = Path(__file__).resolve().parents[1]
    schema_path = tmp_path / "schema.tql"
    schema_path.write_text(_fixture_schema_path().read_text(encoding="utf-8"), encoding="utf-8")

    output_path = tmp_path / "schema.dot"
    completed = subprocess.run(
        [
            sys.executable,
            str(package_root / "ros_typedb_tools" / "typedb_schema_diagram.py"),
            "--input",
            str(schema_path),
            "--format",
            "dot",
            "--output",
            str(output_path),
        ],
        cwd=package_root,
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    assert output_path.exists()
    assert "digraph typedb_schema" in output_path.read_text(encoding="utf-8")
