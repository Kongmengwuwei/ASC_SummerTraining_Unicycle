"""Build this project's snapshot in the real ADS workbench (Windows)."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import zipfile

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_ADS = Path(r"D:\AURIX Development Stdio\AURIX-Studio-1.10.10")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare_plugin(ads, cache):
    source = Path(__file__).with_name("BuildStartup.java")
    jar = cache / "local.ads.build_1.0.0.jar"
    stamp = cache / "bridge.sha256"
    key = digest(source) + str(ads) + digest(ads / "configuration/config.ini")
    if jar.exists() and stamp.exists() and stamp.read_text() == key:
        return jar
    classes = cache / "classes"
    classes.mkdir(exist_ok=True)
    plugins = ads / "plugins"
    java = next(plugins.glob("org.eclipse.justj.openjdk*/jre/bin/java.exe"))
    compiler = next(plugins.glob("org.eclipse.jdt.core.compiler.batch_*.jar"))
    # ECJ is included with ADS, so no separately installed JDK is required.
    cp = os.pathsep.join(str(p) for p in plugins.glob("*.jar"))
    argfile = cache / "compiler.args"
    argfile.write_text('-17 -proc:none -classpath "' + cp.replace(chr(92), '/') +
                       '" -d "' + classes.as_posix() + '" "' + source.as_posix() + '"')
    result = subprocess.run([str(java), "-jar", str(compiler), "@" + str(argfile)],
                            capture_output=True, text=True, timeout=120)
    if result.returncode:
        raise RuntimeError("Bridge compilation failed:\n" + result.stdout + result.stderr)
    manifest = (
        "Manifest-Version: 1.0\nBundle-ManifestVersion: 2\n"
        "Bundle-SymbolicName: local.ads.build;singleton:=true\n"
        "Bundle-Version: 1.0.0\nBundle-Name: Local ADS Build Automation\n"
        "Require-Bundle: org.eclipse.ui,org.eclipse.core.runtime,\n"
        " org.eclipse.core.resources,org.eclipse.cdt.managedbuilder.core,\n"
        " org.eclipse.cdt.ui,org.eclipse.jface.text\n"
        "Bundle-RequiredExecutionEnvironment: JavaSE-17\n\n"
    )
    with zipfile.ZipFile(jar, "w", zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("META-INF/MANIFEST.MF", manifest)
        archive.writestr("plugin.xml", '<?xml version="1.0"?><plugin><extension '
                         'point="org.eclipse.ui.startup"><startup '
                         'class="local.ads.BuildStartup"/></extension></plugin>')
        for f in classes.rglob("*.class"):
            archive.write(f, f.relative_to(classes).as_posix())
    stamp.write_text(key)
    return jar


def configuration(ads, run, jar):
    config = run / "configuration"
    simple = config / "org.eclipse.equinox.simpleconfigurator"
    simple.mkdir(parents=True)
    lines = []
    for line in (ads / "configuration/org.eclipse.equinox.simpleconfigurator/bundles.info").read_text().splitlines():
        if line and not line.startswith("#"):
            fields = line.split(",")
            fields[2] = (ads / fields[2]).as_uri()
            line = ",".join(fields)
        lines.append(line)
    lines.append("local.ads.build,1.0.0," + jar.as_uri() + ",4,false")
    (simple / "bundles.info").write_text("\n".join(lines) + "\n")
    original = (ads / "configuration/config.ini").read_text()
    # Absolute installation paths; runtime caches go only in this run directory.
    original = original.replace("file\\:plugins/", "file\\:" + ads.as_posix() + "/plugins/")
    original = original.replace("reference\\:file\\:org.eclipse.", "reference\\:file\\:" + ads.as_posix() + "/plugins/org.eclipse.")
    (config / "config.ini").write_text(original)
    return config


def inputs(root):
    files = [root / ".project", root / ".cproject"]
    files.extend(p for p in root.iterdir() if p.is_file() and p.suffix.lower() in
                 {".c", ".cpp", ".h", ".hpp", ".inc", ".lsl", ".s", ".asm"})
    for directory in ("code", "user", "libraries", ".settings"):
        files.extend(p for p in (root / directory).rglob("*") if p.is_file())
    return {p.relative_to(root).as_posix(): digest(p) for p in sorted(files)}


def main():
    global ROOT
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ads", type=Path, default=Path(os.environ.get("ADS_HOME", DEFAULT_ADS)))
    parser.add_argument("--clean", action="store_true", help="Clean and rebuild Debug")
    parser.add_argument("--timeout", type=int, default=240)
    parser.add_argument("--project", type=Path, default=ROOT, help="Firmware root (defaults to this repository)")
    args = parser.parse_args()
    ROOT = args.project.resolve()
    if not (ROOT / ".project").is_file() or not (ROOT / ".cproject").is_file():
        raise RuntimeError("Project must contain .project and .cproject")
    output = ROOT / ".ads" / "build"
    output.mkdir(parents=True, exist_ok=True)
    latest = output / "latest.json"
    ads = args.ads.resolve()
    if not (ads / "AURIX-studioc.exe").is_file():
        raise RuntimeError(f"ADS missing: {ads}; set ADS_HOME or --ads")
    key = hashlib.sha256(str(ROOT).lower().encode()).hexdigest()[:12]
    cache = Path(tempfile.gettempdir()) / "unicycle-ads-build" / key
    cache.mkdir(parents=True, exist_ok=True)
    # Exclusive OS file lock, released even after a crash.
    import msvcrt
    lock = (cache / "build.lock").open("a+b")
    lock.seek(0)
    if lock.read(1) == b"":
        lock.write(b"0")
        lock.flush()
    lock.seek(0)
    try:
        msvcrt.locking(lock.fileno(), msvcrt.LK_NBLCK, 1)
    except OSError:
        raise RuntimeError("Another ADS build is using this project's cache")
    latest.write_text(json.dumps({"success": False, "status": "running"}), encoding="utf-8")
    run = cache / (time.strftime("%Y%m%d-%H%M%S") + "-" + str(os.getpid()))
    run.mkdir()
    print(f"ADS build run: {run}", flush=True)
    snapshot = cache / "project"
    snapshot.mkdir(exist_ok=True)
    current = inputs(ROOT)
    manifest = cache / "inputs.json"
    (run / "inputs.json").write_text(json.dumps(current, indent=2))
    previous = json.loads(manifest.read_text()) if manifest.exists() else {}
    changed = [name for name, value in current.items() if previous.get(name) != value]
    removed = previous.keys() - current.keys()
    for name in removed:
        (snapshot / name).unlink(missing_ok=True)
    for name in current:
        dest = snapshot / name
        if name in changed or not dest.exists():
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(ROOT / name, dest)
            os.utime(dest, None)  # Reverted files may have old source timestamps.
    manifest.write_text(json.dumps(current, indent=2))
    latest.write_text(json.dumps({"success": False, "status": "running", "run": str(run)}), encoding="utf-8")
    jar = prepare_plugin(ads, cache)
    config = configuration(ads, run, jar)
    result_path = run / "result.txt"
    workspace = run / "workspace"
    prefs = run / "preferences.ini"
    prefs.write_text("org.eclipse.ui/showIntro=false\norg.eclipse.ui/SHOW_PROGRESS_ON_STARTUP=false\n"
                     "org.eclipse.core.resources/description.autobuilding=false\n"
                     "org.eclipse.cdt.core/indexerId=org.eclipse.cdt.core.nullindexer\n")
    command = [str(ads / "AURIX-studioc.exe"), "-nosplash", "-consoleLog",
               "-configuration", str(config), "-data", str(workspace),
               "-pluginCustomization", str(prefs), "-vmargs",
               "-Dlocal.ads.project=" + str(snapshot),
               "-Dlocal.ads.result=" + str(result_path),
               "-Dlocal.ads.clean=" + str(args.clean or bool(removed)).lower()]
    started = time.time()
    with (run / "launcher.log").open("w", encoding="utf-8") as log:
        startup = subprocess.STARTUPINFO()
        startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        startup.wShowWindow = 0
        process = subprocess.Popen(command, cwd=ads, stdout=log, stderr=subprocess.STDOUT,
                                   startupinfo=startup)
        try:
            process.wait(timeout=args.timeout)
        except subprocess.TimeoutExpired:
            subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"],
                           capture_output=True)
            raise RuntimeError(f"ADS timed out after {args.timeout}s; logs: {run}")
    result = result_path.read_text(encoding="utf-8") if result_path.exists() else "Missing build result"
    ok = process.returncode == 0 and result.startswith("STATUS=OK\n")
    console = (run / "build.log").read_text(encoding="utf-8") if (run / "build.log").exists() else ""
    ok = ok and bool(re.search(r"Build Finished\.\s+0 errors", console))
    artifacts = []
    for ext in ("elf", "hex", "map"):
        f = snapshot / "Debug" / ("ASC_SummerTraining_Unicycle." + ext)
        valid = f.is_file() and f.stat().st_size > 0
        if args.clean or changed or removed:
            valid = valid and f.stat().st_mtime >= started if f.exists() else False
        ok = ok and valid
        if f.exists():
            artifacts.append({"path": str(f), "bytes": f.stat().st_size, "sha256": digest(f)})
    if inputs(ROOT) != current:
        ok = False
        result += "\nSource changed during build; rerun to test the current version."
    summary = {"success": ok, "exit_code": process.returncode,
               "seconds": round(time.time() - started, 1), "run": str(run),
               "build_log": str(run / "build.log"),
               "input_sha256": hashlib.sha256(json.dumps(current, sort_keys=True).encode()).hexdigest(),
               "changed_inputs": changed, "removed_inputs": list(removed),
               "artifacts": artifacts, "result": result}
    (run / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    latest.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(result.strip())
    print(f"{'PASS' if ok else 'FAIL'} in {summary['seconds']}s; summary: {output / 'latest.json'}")
    return 0 if ok else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
