"""`vg` command: configure, build, test and format vg_core.

Run from anywhere inside the repository, e.g.:

    vg build            # configure (if needed) and build the debug preset
    vg test             # build, then run the C++ unit tests
    vg format --check   # verify clang-format on C++ sources
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from collections.abc import Sequence
from pathlib import Path

CPP_DIRS = ("include", "src", "tests")
CPP_SUFFIXES = {".h", ".hpp", ".cpp", ".cc"}


def repo_root(start: Path | None = None) -> Path:
    """Return the vg_core checkout containing `start` (default: this file)."""
    path = (start or Path(__file__)).resolve()
    for candidate in (path, *path.parents):
        if (candidate / "CMakePresets.json").is_file():
            return candidate
    raise SystemExit("vg: could not find the vg_core root (no CMakePresets.json)")


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


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="vg", description=__doc__.splitlines()[0])
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

    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args, repo_root(Path.cwd()))


if __name__ == "__main__":
    sys.exit(main())
