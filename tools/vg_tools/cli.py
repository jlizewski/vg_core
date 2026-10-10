"""`vg` command: configure, build, test and format vg_core.

Run from anywhere inside the repository, e.g.:

    vg build            # configure (if needed) and build the debug preset
    vg test             # build, then run the C++ unit tests
    vg format --check   # verify clang-format on C++ sources
    vg schemas          # regenerate embedded MCAP schema descriptors
    vg heatmap sun.asc sun.png   # render a sun map (or any .asc grid) as a PNG
    vg preview garden/           # render every .asc in a directory as a PNG
    vg replay rec.mcap garden/   # build a map package (C++ app from apps/, see vg_tools/apps.py)
    vg map garden/ ground sun    # re-run map package stages from its saved 3D map
    vg mcap repair rec.mcap      # index a recording that was never closed
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from collections.abc import Sequence
from pathlib import Path

from vg_tools import apps

CPP_DIRS = ("apps", "include", "src", "tests")
CPP_SUFFIXES = {".h", ".hpp", ".cpp", ".cc"}
# Commands implemented here; a C++ app can't take one of these names.
BUILTINS = ("configure", "build", "test", "format", "schemas", "heatmap", "preview")


def repo_root(start: Path | None = None) -> Path:
    """Return the vg_core checkout containing `start` (default: this file)."""
    path = (start or Path(__file__)).resolve()
    for candidate in (path, *path.parents):
        if (candidate / "CMakePresets.json").is_file():
            return candidate
    raise SystemExit("vg: could not find the vg_core root (no CMakePresets.json)")


def find_root() -> Path:
    """The checkout containing the current directory, else the one these tools
    were installed from, so apps can be run on files anywhere."""
    try:
        return repo_root(Path.cwd())
    except SystemExit:
        return repo_root()


def run(cmd: Sequence[str], cwd: Path) -> int:
    print("+", " ".join(cmd), flush=True)
    return subprocess.call(list(cmd), cwd=cwd)


def cpp_sources(root: Path) -> list[Path]:
    return sorted(
        p
        for d in CPP_DIRS
        if (root / d).is_dir()
        for p in (root / d).rglob("*")
        if p.suffix in CPP_SUFFIXES
    )


def cmd_configure(args: argparse.Namespace, root: Path) -> int:
    return run(["cmake", "--preset", args.preset], root)


def cmd_build(args: argparse.Namespace, root: Path) -> int:
    if not (root / "build" / args.preset / "CMakeCache.txt").is_file():
        rc = cmd_configure(args, root)
        if rc:
            return rc
    return run(["cmake", "--build", "--preset", args.preset], root)


def cmd_test(args: argparse.Namespace, root: Path) -> int:
    rc = cmd_build(args, root)
    if rc:
        return rc
    return run(["ctest", "--preset", args.preset], root)


def cmd_format(args: argparse.Namespace, root: Path) -> int:
    clang_format = shutil.which("clang-format")
    if clang_format is None:
        print("vg: clang-format not found on PATH", file=sys.stderr)
        return 1
    files = [str(p.relative_to(root)) for p in cpp_sources(root)]
    if not files:
        return 0
    mode = ["--dry-run", "--Werror"] if args.check else ["-i"]
    return run([clang_format, *mode, *files], root)


def cmd_schemas(args: argparse.Namespace, root: Path) -> int:
    from vg_tools import schemas

    return schemas.generate(root)


def cmd_heatmap(args: argparse.Namespace, root: Path) -> int:
    from vg_tools import heatmap

    try:
        lo, hi = heatmap.write_heatmap(args.grid, args.png, args.scale, args.min, args.max)
    except (OSError, ValueError) as e:
        print(f"vg: {e}", file=sys.stderr)
        return 1
    print(f"{args.png}: {lo:g} (dark) to {hi:g} (bright)")
    return 0


def cmd_preview(args: argparse.Namespace, root: Path) -> int:
    from vg_tools import heatmap

    try:
        written = heatmap.write_previews(args.directory, args.scale)
    except (OSError, ValueError) as e:
        print(f"vg: {e}", file=sys.stderr)
        return 1
    for png in written:
        print(png)
    if not written:
        print(f"vg: no .asc grids in {args.directory}", file=sys.stderr)
        return 1
    return 0


def add_app_parsers(sub: argparse._SubParsersAction, app_list: Sequence[apps.App]) -> None:
    """List the C++ apps in --help. main() runs them before argparse sees their
    arguments, so these parsers are only for help (and `vg mcap` alone)."""
    groups: dict[tuple[str, ...], argparse._SubParsersAction] = {(): sub}
    for app in app_list:
        for i in range(1, len(app.words)):
            prefix = app.words[:i]
            if prefix not in groups:
                parent = groups.get(prefix[:-1])
                if parent is None or prefix[-1] in parent.choices:
                    break
                group = parent.add_parser(prefix[-1], help=f"{' '.join(prefix)} tools")
                groups[prefix] = group.add_subparsers(dest=" ".join(prefix), required=True)
        parent = groups.get(app.words[:-1])
        if parent is None or app.words[-1] in parent.choices:
            continue  # Hidden by a built-in command or another app.
        p = parent.add_parser(
            app.words[-1],
            help=f"{app.help} (runs {app.target})" if app.help else f"run {app.target}",
            add_help=False,
        )
        p.add_argument("args", nargs=argparse.REMAINDER)


def build_parser(root: Path | None = None) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="vg",
        description=__doc__.splitlines()[0],
        epilog="C++ apps take --vg-preset NAME and --vg-no-build; other arguments go to the app.",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    for name, func, help_text in (
        ("configure", cmd_configure, "configure a CMake preset"),
        ("build", cmd_build, "build a CMake preset (configures if needed)"),
        ("test", cmd_test, "build and run the C++ unit tests"),
    ):
        p = sub.add_parser(name, help=help_text)
        p.add_argument("--preset", default="debug", help="CMake preset (default: debug)")
        p.set_defaults(func=func)

    p = sub.add_parser("format", help="run clang-format on C++ sources")
    p.add_argument("--check", action="store_true", help="report issues without editing")
    p.set_defaults(func=cmd_format)

    p = sub.add_parser("schemas", help="regenerate embedded MCAP schema descriptors")
    p.set_defaults(func=cmd_schemas)

    p = sub.add_parser("heatmap", help="render an .asc grid such as a sun map as a heatmap PNG")
    p.add_argument("grid", type=Path, help="input ESRI ASCII grid (.asc)")
    p.add_argument("png", type=Path, help="output PNG")
    p.add_argument("--scale", type=int, default=4, help="pixels per cell (default: 4)")
    p.add_argument("--min", type=float, help="value shown darkest (default: grid minimum)")
    p.add_argument("--max", type=float, help="value shown brightest (default: grid maximum)")
    p.set_defaults(func=cmd_heatmap)

    p = sub.add_parser(
        "preview", help="render every .asc grid in a directory (e.g. a map package) as a PNG"
    )
    p.add_argument("directory", type=Path, help="directory holding .asc grids")
    p.add_argument("--scale", type=int, default=4, help="pixels per cell (default: 4)")
    p.set_defaults(func=cmd_preview)

    add_app_parsers(sub, apps.discover(root or repo_root()))
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    root = find_root()
    if argv and argv[0] not in BUILTINS:
        app = apps.match(apps.discover(root), argv)
        if app is not None:
            return apps.run(app, argv[len(app.words) :], root)
    args = build_parser(root).parse_args(argv)
    return args.func(args, root)


if __name__ == "__main__":
    sys.exit(main())
