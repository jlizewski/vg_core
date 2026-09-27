from pathlib import Path

import pytest

from vg_tools import cli


def test_repo_root_finds_presets():
    root = cli.repo_root()
    assert (root / "CMakeLists.txt").is_file()


def test_repo_root_errors_outside_repo(tmp_path: Path):
    with pytest.raises(SystemExit):
        cli.repo_root(tmp_path)


def test_cpp_sources_lists_library_files():
    root = cli.repo_root()
    names = {p.name for p in cli.cpp_sources(root)}
    assert "version.hpp" in names
    assert "version.cpp" in names


def test_parser_defaults_to_debug_preset():
    args = cli.build_parser().parse_args(["build"])
    assert args.preset == "debug"
    assert args.func is cli.cmd_build


def test_parser_has_schemas_command():
    args = cli.build_parser().parse_args(["schemas"])
    assert args.func is cli.cmd_schemas
