from pathlib import Path

import pytest

from vg_tools import apps, cli


def make_app(root: Path, target: str, summary: str = "does things") -> None:
    (root / "apps").mkdir(exist_ok=True)
    (root / "apps" / f"{target}.cpp").write_text(f"// {target}: {summary}.\nint main() {{}}\n")


def test_discovers_repo_apps():
    found = {app.words: app for app in apps.discover(cli.repo_root())}
    assert found[("replay",)].target == "vg_replay"
    assert found[("mcap", "repair")].target == "vg_mcap_repair"
    assert found[("replay",)].help.startswith("build the full map package")
    assert found[("map",)].target == "vg_map"


def test_match_prefers_longest_words(tmp_path: Path):
    make_app(tmp_path, "vg_mcap")
    make_app(tmp_path, "vg_mcap_repair")
    found = apps.discover(tmp_path)
    assert apps.match(found, ["mcap", "repair", "a.mcap"]).target == "vg_mcap_repair"
    assert apps.match(found, ["mcap", "a.mcap"]).target == "vg_mcap"
    assert apps.match(found, ["build"]) is None


def test_split_options_keeps_app_arguments():
    preset, build, rest = apps.split_options(
        ["in.mcap", "--vg-preset", "debug", "--voxels", "v.ply", "--vg-no-build"]
    )
    assert (preset, build, rest) == ("debug", False, ["in.mcap", "--voxels", "v.ply"])
    assert apps.split_options(["--vg-preset=release"])[0] == "release"
    with pytest.raises(SystemExit):
        apps.split_options(["--vg-preset"])


def test_find_binary_checks_multi_config_dirs(tmp_path: Path):
    exe = tmp_path / "build" / "debug" / "Debug" / apps.binary_name("vg_replay")
    exe.parent.mkdir(parents=True)
    exe.write_text("")
    assert apps.find_binary(tmp_path, "debug", "vg_replay") == exe
    assert apps.find_binary(tmp_path, "release", "vg_replay") is None


def test_package_dir_of(tmp_path: Path):
    replay = apps.App("vg_replay", ("replay",), "")
    vg_map = apps.App("vg_map", ("map",), "")
    repair = apps.App("vg_mcap_repair", ("mcap", "repair"), "")
    pkg = str(tmp_path)
    assert apps.package_dir_of(replay, ["s.mcap", pkg, "--year", "2026"]) == tmp_path
    assert apps.package_dir_of(vg_map, [pkg, "ground"]) == tmp_path
    assert apps.package_dir_of(replay, ["s.mcap"]) is None
    assert apps.package_dir_of(replay, ["s.mcap", str(tmp_path / "missing")]) is None
    assert apps.package_dir_of(repair, ["s.mcap", pkg]) is None


def test_main_forwards_app_arguments(monkeypatch: pytest.MonkeyPatch):
    calls = []
    monkeypatch.setattr(apps, "run", lambda app, args, root: calls.append((app.target, args)) or 0)
    assert cli.main(["mcap", "repair", "in.mcap", "--force"]) == 0
    assert calls == [("vg_mcap_repair", ["in.mcap", "--force"])]


def test_parser_lists_apps():
    help_text = cli.build_parser().format_help()
    assert "replay" in help_text
    assert "mcap" in help_text
