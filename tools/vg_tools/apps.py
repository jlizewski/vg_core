"""Run vg_core's C++ command-line apps as `vg` subcommands.

Every `apps/vg_<name>.cpp` is built as the CMake target `vg_<name>` and runs as
`vg <name>`, with underscores splitting the name into subcommand words:

    apps/vg_replay.cpp       ->  vg replay ...
    apps/vg_mcap_repair.cpp  ->  vg mcap repair ...

Arguments after the subcommand go to the app unchanged. Before running, the app
is built (incrementally) in the release preset if it's configured, else debug,
so it's never stale. Two options are taken out for `vg` itself:

    --vg-preset NAME   use this CMake preset's build
    --vg-no-build      run the existing binary without building it first

The help shown by `vg --help` is the app's first comment line,
`// vg_<name>: what it does`.
"""

from __future__ import annotations

import os
import subprocess
import sys
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

# Presets tried, in order, when --vg-preset isn't given.
PRESETS = ("release", "debug")
# Where multi-config generators (Visual Studio, Ninja Multi-Config) put binaries.
CONFIG_DIRS = ("", "Release", "Debug", "RelWithDebInfo", "MinSizeRel")


@dataclass(frozen=True)
class App:
    target: str  # CMake target and binary name, e.g. vg_mcap_repair
    words: tuple[str, ...]  # subcommand words, e.g. ("mcap", "repair")
    help: str


def discover(root: Path) -> list[App]:
    """The apps in `root`/apps, one per vg_<name>.cpp."""
    apps = []
    for source in sorted((root / "apps").glob("vg_*.cpp")):
        target = source.stem
        words = tuple(w for w in target.removeprefix("vg_").split("_") if w)
        if words:
            apps.append(App(target, words, _summary(source, target)))
    return apps


def _summary(source: Path, target: str) -> str:
    try:
        with source.open(encoding="utf-8") as f:
            first = f.readline().strip()
    except OSError:
        return ""
    prefix = f"// {target}:"
    return first[len(prefix) :].strip().rstrip(".") if first.startswith(prefix) else ""


def match(apps: Sequence[App], argv: Sequence[str]) -> App | None:
    """The app whose subcommand words start argv (the longest, if several do)."""
    best = None
    for app in apps:
        n = len(app.words)
        if tuple(argv[:n]) == app.words and (best is None or n > len(best.words)):
            best = app
    return best


def split_options(args: Sequence[str]) -> tuple[str | None, bool, list[str]]:
    """Take --vg-preset and --vg-no-build out of args: (preset, build, rest)."""
    preset = None
    build = True
    rest: list[str] = []
    it = iter(args)
    for arg in it:
        if arg == "--vg-no-build":
            build = False
        elif arg == "--vg-preset":
            preset = next(it, None)
            if preset is None:
                raise SystemExit("vg: --vg-preset needs a preset name")
        elif arg.startswith("--vg-preset="):
            preset = arg.split("=", 1)[1]
        else:
            rest.append(arg)
    return preset, build, rest


def binary_name(target: str) -> str:
    return target + ".exe" if os.name == "nt" else target


def find_binary(root: Path, preset: str, target: str) -> Path | None:
    build_dir = root / "build" / preset
    for config in CONFIG_DIRS:
        path = build_dir / config / binary_name(target)
        if path.is_file():
            return path
    return None


def configured(root: Path, preset: str) -> bool:
    return (root / "build" / preset / "CMakeCache.txt").is_file()


def choose_preset(root: Path, target: str) -> str:
    """The first preset that's configured, preferring one that already has the
    app built; release if none is configured yet."""
    for preset in PRESETS:
        if configured(root, preset) and find_binary(root, preset, target):
            return preset
    for preset in PRESETS:
        if configured(root, preset):
            return preset
    return PRESETS[0]


def _cmake(cmd: list[str], root: Path) -> int:
    # Build output goes to stderr so the app's own stdout stays clean.
    print("+", " ".join(cmd), file=sys.stderr, flush=True)
    return subprocess.call(cmd, cwd=root, stdout=sys.stderr)


def build(root: Path, preset: str, target: str) -> int:
    if not configured(root, preset):
        rc = _cmake(["cmake", "--preset", preset], root)
        if rc:
            return rc
    return _cmake(["cmake", "--build", "--preset", preset, "--target", target], root)


def run(app: App, args: Sequence[str], root: Path) -> int:
    """Build (unless --vg-no-build) and run app with args, from the current directory."""
    preset, do_build, rest = split_options(args)
    preset = preset or choose_preset(root, app.target)
    if do_build:
        rc = build(root, preset, app.target)
        if rc:
            print(f"vg: building {app.target} ({preset}) failed", file=sys.stderr)
            return rc
    binary = find_binary(root, preset, app.target)
    if binary is None:
        print(
            f"vg: {app.target} isn't built in build/{preset} "
            f"(run `vg build --preset {preset}` or drop --vg-no-build)",
            file=sys.stderr,
        )
        return 1
    try:
        return subprocess.call([str(binary), *rest])
    except KeyboardInterrupt:
        return 130
