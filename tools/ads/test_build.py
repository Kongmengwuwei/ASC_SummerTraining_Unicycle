"""Integration check: compile failure and recovery, without editing firmware."""
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

def main():
    """Explicit opt-in integration run: launches ADS twice."""
    runner = Path(__file__).with_name("build.py")
    spec = importlib.util.spec_from_file_location("ads_build", runner)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    fixture = Path(tempfile.mkdtemp(prefix="unicycle-ads-selftest-"))
    for name in module.inputs(module.ROOT):
        dest = fixture / name
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(module.ROOT / name, dest)
    probe = fixture / "code/ads_build_failure_probe.c"
    probe.write_text("#error ADS_BUILD_EXPECTED_FAILURE\n")
    command = [sys.executable, str(runner), "--project", str(fixture)]
    failed = subprocess.run(command).returncode
    first = json.loads((fixture / ".ads/build/latest.json").read_text())
    assert failed != 0 and not first["success"], first
    assert "ADS_BUILD_EXPECTED_FAILURE" in Path(first["run"], "build.log").read_text(), first
    probe.unlink()
    recovered = subprocess.run(command).returncode
    second = json.loads((fixture / ".ads/build/latest.json").read_text())
    assert recovered == 0 and second["success"], second
    assert "code/ads_build_failure_probe.c" in second["removed_inputs"], second
    print("PASS: deliberate compiler error rejected; removal and rebuild recovered")
    print("Self-test evidence:", fixture / ".ads/build/latest.json")


if __name__ == "__main__":
    main()
