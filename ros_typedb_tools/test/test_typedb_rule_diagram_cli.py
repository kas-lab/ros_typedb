"""Tests for typedb_rule_diagram CLI behavior."""

import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

from ros_typedb_tools.typedb_rule_diagram import main


def _fixture_rule_path() -> Path:
    return Path(__file__).parent / "fixtures" / "mock_rules_schema.tql"


def test_cli_writes_dot_output_to_default_path(tmp_path: Path):
    rules_root = tmp_path / "project" / "schemas" / "demo"
    rules_root.mkdir(parents=True, exist_ok=True)
    rules_path = rules_root / "rules.tql"
    rules_path.write_text(_fixture_rule_path().read_text(encoding="utf-8"), encoding="utf-8")

    rc = main(["--input", str(rules_path), "--format", "dot"])
    assert rc == 0

    output_path = tmp_path / "project" / "schemas_diagrams" / "rules_rules.dot"
    assert output_path.exists()
    assert "digraph typedb_rule_graph" in output_path.read_text(encoding="utf-8")


def test_cli_honors_explicit_output_path(tmp_path: Path):
    rules_path = tmp_path / "rules.tql"
    rules_path.write_text(_fixture_rule_path().read_text(encoding="utf-8"), encoding="utf-8")

    output_path = tmp_path / "custom" / "rule_graph.dot"
    rc = main(
        [
            "--input",
            str(rules_path),
            "--format",
            "dot",
            "--output",
            str(output_path),
        ]
    )
    assert rc == 0
    assert output_path.exists()
    assert "rule__detect_alarm" in output_path.read_text(encoding="utf-8")


def test_cli_disables_rule_dependencies(tmp_path: Path):
    rules_path = tmp_path / "rules.tql"
    rules_path.write_text(_fixture_rule_path().read_text(encoding="utf-8"), encoding="utf-8")

    output_path = tmp_path / "nodeps.dot"
    rc = main(
        [
            "--input",
            str(rules_path),
            "--format",
            "dot",
            "--output",
            str(output_path),
            "--no-rule-dependencies",
        ]
    )

    assert rc == 0
    dot_text = output_path.read_text(encoding="utf-8")
    assert "rule__detect_alarm -> rule__escalate_alarm" not in dot_text


def test_cli_honors_horizontal_orientation(tmp_path: Path):
    rules_path = tmp_path / "rules.tql"
    rules_path.write_text(_fixture_rule_path().read_text(encoding="utf-8"), encoding="utf-8")

    output_path = tmp_path / "horizontal.dot"
    rc = main(
        [
            "--input",
            str(rules_path),
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
    rules_path = tmp_path / "rules.tql"
    rules_path.write_text(_fixture_rule_path().read_text(encoding="utf-8"), encoding="utf-8")

    output_path = tmp_path / "rules.svg"
    rc = main(
        [
            "--input",
            str(rules_path),
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
    missing_path = tmp_path / "missing_rules.tql"
    rc = main(["--input", str(missing_path), "--format", "dot"])
    assert rc != 0


def test_cli_merges_multiple_input_files(tmp_path: Path):
    first = tmp_path / "schemas" / "a" / "first.tql"
    second = tmp_path / "schemas" / "b" / "second.tql"
    first.parent.mkdir(parents=True, exist_ok=True)
    second.parent.mkdir(parents=True, exist_ok=True)
    first.write_text("define\nrule r1:\nwhen { $x isa A; }\nthen { $x has mark true; };", encoding="utf-8")
    second.write_text("define\nrule r2:\nwhen { $x isa mark; }\nthen { $x has done true; };", encoding="utf-8")

    output_path = tmp_path / "merged.dot"
    rc = main(
        [
            "--input",
            str(first),
            str(second),
            "--format",
            "dot",
            "--output",
            str(output_path),
        ]
    )

    assert rc == 0
    dot_text = output_path.read_text(encoding="utf-8")
    assert "rule__r1" in dot_text
    assert "rule__r2" in dot_text
    assert "rule__r1 -> rule__r2" in dot_text


def test_cli_uses_default_merged_output_path_for_multiple_inputs(tmp_path: Path):
    first = tmp_path / "workspace" / "schemas" / "a" / "first.tql"
    second = tmp_path / "workspace" / "schemas" / "b" / "second.tql"
    first.parent.mkdir(parents=True, exist_ok=True)
    second.parent.mkdir(parents=True, exist_ok=True)
    first.write_text("define\nrule r1:\nwhen { $x isa A; }\nthen { $x has mark true; };", encoding="utf-8")
    second.write_text("define\nrule r2:\nwhen { $x isa mark; }\nthen { $x has done true; };", encoding="utf-8")

    rc = main(
        [
            "--input",
            str(first),
            str(second),
            "--format",
            "dot",
        ]
    )

    assert rc == 0
    assert (tmp_path / "workspace" / "schemas_diagrams" / "merged_rules.dot").exists()


def test_cli_merges_multiple_inputs_with_mixed_absolute_and_relative_paths(tmp_path: Path):
    workspace = tmp_path / "workspace"
    first = workspace / "schemas" / "a" / "first.tql"
    second = workspace / "schemas" / "b" / "second.tql"
    first.parent.mkdir(parents=True, exist_ok=True)
    second.parent.mkdir(parents=True, exist_ok=True)
    first.write_text("define\nrule r1:\nwhen { $x isa A; }\nthen { $x has mark true; };", encoding="utf-8")
    second.write_text("define\nrule r2:\nwhen { $x isa mark; }\nthen { $x has done true; };", encoding="utf-8")

    previous_cwd = Path.cwd()
    try:
        os.chdir(workspace)
        rc = main(
            [
                "--input",
                str(first),
                str(Path("schemas") / "b" / "second.tql"),
                "--format",
                "dot",
            ]
        )
    finally:
        os.chdir(previous_cwd)

    assert rc == 0
    assert (workspace / "schemas_diagrams" / "merged_rules.dot").exists()


def test_direct_script_execution_writes_dot_output(tmp_path: Path):
    package_root = Path(__file__).resolve().parents[1]
    rules_path = tmp_path / "rules.tql"
    rules_path.write_text(_fixture_rule_path().read_text(encoding="utf-8"), encoding="utf-8")

    output_path = tmp_path / "rules.dot"
    completed = subprocess.run(
        [
            sys.executable,
            str(package_root / "ros_typedb_tools" / "typedb_rule_diagram.py"),
            "--input",
            str(rules_path),
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
    assert "digraph typedb_rule_graph" in output_path.read_text(encoding="utf-8")
