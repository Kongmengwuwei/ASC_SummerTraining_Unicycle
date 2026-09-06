"""Reproducible Windows bundle, isolated from unrelated PATH DLLs."""
import os
from pathlib import Path
import shutil
import sys


def main():
    if sys.platform != "win32":
        raise SystemExit("Build this Windows application on Windows.")
    root = Path(__file__).resolve().parents[1]
    os.chdir(root)
    windows = Path(os.environ.get("SystemRoot", r"C:\Windows"))
    # A globally installed Poppler/Conda can provide a different icuuc.dll.
    # Qt uses the Windows ICU API; collecting that unrelated DLL breaks startup.
    os.environ["PATH"] = os.pathsep.join(map(str, (
        windows / "System32", windows, Path(sys.executable).parent,
        Path(sys.base_prefix), Path(sys.base_prefix) / "DLLs",
    )))
    from PyInstaller.__main__ import run
    run([
        "--clean", "--noconfirm", "--windowed", "--onedir",
        "--name", "EmbeddedStation", "--collect-all", "pyqtgraph",
        "--collect-all", "OpenGL", "--add-data", "app/profiles;app/profiles",
        "run_station.py",
    ])
    import PySide6
    qt = Path(PySide6.__file__).parent
    destination = root / "dist/EmbeddedStation/_internal"
    # The Python distribution may bundle an older VC runtime than current Qt.
    # Use the redistributable runtime shipped with Qt consistently at bundle root.
    for pattern in ("VCRUNTIME140*.dll", "MSVCP140*.dll"):
        for library in qt.glob(pattern):
            shutil.copy2(library, destination / library.name)
    if (destination / "icuuc.dll").exists():
        raise SystemExit("Unexpected private ICU runtime in bundle; inspect DLL sources before release.")
    print(f"Ready: {root / 'dist/EmbeddedStation/EmbeddedStation.exe'}")


if __name__ == "__main__":
    main()
