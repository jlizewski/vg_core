"""Opening files in Foxglove (https://foxglove.dev), the desktop viewer for
MCAP recordings, from the tools (vg dashboard's "Foxglove" button).

Foxglove is found in its usual install places, or set VG_FOXGLOVE to its
executable. foxglove_layout.json beside this file is a starting layout (a 3D
view following the phone over the site, with the camera image beside it), to
import once in Foxglove: Layouts -> Import from file.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

LAYOUT = Path(__file__).with_name("foxglove_layout.json")
DOWNLOAD = "https://foxglove.dev/download"


def find() -> list[str] | None:
    """The command that starts Foxglove (a file to open goes after it), or None."""
    override = os.environ.get("VG_FOXGLOVE")
    if override:
        return [override]
    if sys.platform == "win32":
        registered = _registered_foxglove()
        if registered is not None:
            return [registered]
        # Its installer puts it in a folder of its own, named "desktop" by
        # recent versions.
        roots = [Path(os.environ[v]) for v in ("LOCALAPPDATA", "ProgramFiles") if v in os.environ]
        for root in roots:
            base = root / "Programs" if root.name.lower() == "local" else root
            for exe in sorted(base.glob("*/Foxglove*.exe")):
                return [str(exe)]
        return None
    if sys.platform == "darwin":
        for app in ("Foxglove", "Foxglove Studio"):
            if Path(f"/Applications/{app}.app").exists():
                return ["open", "-a", app]
        return None
    for name in ("foxglove-studio", "foxglove"):
        if path := shutil.which(name):
            return [path]
    return None


def _registered_foxglove() -> str | None:
    """Foxglove's executable, from the command Windows opens .mcap files with
    when that's Foxglove (its installer registers it)."""
    import winreg

    try:
        prog_id = winreg.QueryValue(winreg.HKEY_CLASSES_ROOT, ".mcap")
        command = winreg.QueryValue(winreg.HKEY_CLASSES_ROOT, rf"{prog_id}\shell\open\command")
    except OSError:
        return None
    exe = command.split('"')[1] if command.startswith('"') else command.split(" ")[0]
    return exe if "foxglove" in Path(exe).name.lower() and Path(exe).is_file() else None


def _mcap_has_an_app() -> bool:
    """Whether Windows has an app for .mcap files (without one, opening a file
    asks which app to use instead of failing)."""
    import winreg

    try:
        winreg.CloseKey(winreg.OpenKey(winreg.HKEY_CLASSES_ROOT, ".mcap"))
        return True
    except OSError:
        return False


def open_file(path: Path) -> str:
    """Opens path in Foxglove; what happened, for a log."""
    command = find()
    if command is not None:
        subprocess.Popen([*command, str(path)], stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)
        return f"opened {path} in Foxglove"
    if sys.platform == "win32" and _mcap_has_an_app():
        try:  # Whatever .mcap files open with.
            os.startfile(path)  # type: ignore[attr-defined]
            return f"opened {path} with the app for .mcap files"
        except OSError:
            pass
    return (f"Foxglove isn't installed (get it from {DOWNLOAD}, or set VG_FOXGLOVE to it); "
            f"open {path} in it")
