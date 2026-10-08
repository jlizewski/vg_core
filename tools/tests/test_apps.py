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
    assert found[("replay",)].help.startswith("rebuild a map")


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


def test_choose_preset_prefers_built_then_configured(tmp_path: Path):
    assert apps.choose_preset(tmp_path, "vg_replay") == "release"
    for preset in ("debug", "release"):
        (tmp_path / "build" / preset).mkdir(parents=True)
        (tmp_path / "build" / preset / "CMakeCache.txt").write_text("")
    assert apps.choose_preset(tmp_path, "vg_replay") == "release"
    (tmp_path / "build" / "debug" / apps.binary_name("vg_replay")).write_text("")
    assert apps.choose_preset(tmp_path, "vg_replay") == "debug"


def test_main_forwards_app_arguments(monkeypatch: pytest.MonkeyPatch):
    calls = []
    monkeypatch.setattr(apps, "run", lambda app, args, root: calls.append((app.target, args)) or 0)
    assert cli.main(["mcap", "repair", "in.mcap", "--force"]) == 0
    assert calls == [("vg_mcap_repair", ["in.mcap", "--force"])]


def test_parser_lists_apps():
    help_text = cli.build_parser().format_help()
    assert "replay" in help_text
    assert "mcap" in help_text
