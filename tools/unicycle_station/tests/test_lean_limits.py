"""Host checks for the real lean function and public parameter boundaries.

These checks do not model the vehicle, the IMU, or target compiler behavior.
"""
from pathlib import Path
import json
import re
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[3]


def _macros():
    source = (ROOT / "code/board_config.h").read_text(encoding="utf-8")
    return dict(re.findall(r"^#define\s+(\w+)\s+([^\r\n/]+)", source, re.M))


def _number(value, macros):
    value = value.strip().strip("()").strip()
    if value in macros:
        return _number(macros[value], macros)
    return float(value.rstrip("fFuUlL"))


def _bounds():
    source = (ROOT / "code/param.c").read_text(encoding="utf-8")
    macros = _macros()
    return {
        key: tuple(_number(v, macros) for v in re.search(
            r'\{\s*"' + key + r'",\s*&g_param\.\w+,\s*1,\s*([^,]+),\s*([^}]+)\}',
            source,
        ).groups())
        for key in ("lean_roll_kp", "lean_max_angle")
    }


def _named_parameters(value):
    if isinstance(value, dict):
        if value.get("name") in ("lean_roll_kp", "lean_max_angle") and "max" in value:
            yield value
        for child in value.values():
            yield from _named_parameters(child)
    elif isinstance(value, list):
        for child in value:
            yield from _named_parameters(child)


@pytest.mark.parametrize("relative", [
    "tools/unicycle_station/app/profiles/tc264_unicycle/profile.json",
    "tools/unicycle_station/examples/default.workspace.json",
])
def test_public_lean_ranges_and_defaults_match_firmware(relative):
    bounds = _bounds()
    assert bounds == {"lean_roll_kp": (0, 100), "lean_max_angle": (0, 10)}
    defaults = {"lean_roll_kp": 2.5, "lean_max_angle": 5.0}
    parameters = list(_named_parameters(json.loads((ROOT / relative).read_text(encoding="utf-8"))))
    assert len(parameters) == 2
    for parameter in parameters:
        assert (parameter["min"], parameter["max"]) == bounds[parameter["name"]]
        assert parameter["value"] == defaults[parameter["name"]]
    macros = _macros()
    assert _number(macros["DIRECTION_ROLL_KP_DEFAULT"], macros) == 2.5
    assert _number(macros["LEAN_MAX_ANGLE_DEFAULT"], macros) == 5
    assert _number(macros["DIRECTION_LEAN_LIMIT"], macros) == bounds["lean_max_angle"][1]
    assert _number(macros["ROLL_TARGET_LIMIT"], macros) == 10


def test_real_lean_function_limits_gating_and_slew(tmp_path):
    gcc = shutil.which("gcc") or "D:/Tools/Dev-Cpp/MinGW64/bin/gcc.exe"
    if not Path(gcc).exists():
        pytest.skip("Optional GCC host compiler not installed")
    control = (ROOT / "code/control.c").read_text(encoding="utf-8")
    function = re.search(r"static float lean_offset_update\(float direction_output\)\s*\{.*?^\}", control, re.M | re.S)
    assert function
    macros = _macros()
    definitions = "\n".join(
        f"#define {name} {value.strip()}" for name, value in macros.items()
        if name.startswith("DIRECTION_LEAN_") or name == "DIRECTION_ROLL_KP_MAX"
    )
    source = r'''
#include <assert.h>
#include <math.h>
static float gain = 2.5f, maximum = 5.0f;
static float g_lean_offset, g_vision_direction_camera;
static int s_run_active = 1, speed = 280;
#define DIRECTION_ROLL_KP gain
#define LEAN_MAX_ANGLE maximum
static int Y_Motor_GetSpeed20ms(void) { return speed; }
static float constrain_float(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
'''
    source += definitions + "\n" + function.group() + r'''
static void settle(float direction) {
    for (int i = 0; i < 300; ++i) {
        float next = lean_offset_update(direction);
        assert(fabsf(next - g_lean_offset) <= DIRECTION_LEAN_SLEW + 1e-5f);
        assert(fabsf(next) <= DIRECTION_LEAN_LIMIT + 1e-5f);
        g_lean_offset = next;
    }
}
int main(void) {
    g_vision_direction_camera = 1001.0f;
    settle(35.0f / 200.0f);
    assert(fabsf(g_lean_offset - 1.225f) < 1e-4f); /* unchanged default */
    gain = DIRECTION_ROLL_KP_MAX;
    maximum = 10.0f;
    settle(35.0f / 200.0f);
    assert(fabsf(g_lean_offset - 10.0f) < 1e-4f); /* new range reachable */
    maximum = 99.0f;
    settle(35.0f / 200.0f);
    assert(fabsf(g_lean_offset - 10.0f) < 1e-4f); /* hard cap remains */
    settle(-35.0f / 200.0f);
    assert(fabsf(g_lean_offset + 10.0f) < 1e-4f);
    g_vision_direction_camera = 1000.0f;
    settle(-35.0f / 200.0f);
    assert(g_lean_offset == 0.0f); /* strict gate */
    g_vision_direction_camera = -1001.0f;
    maximum = 5.0f;
    settle(35.0f / 200.0f);
    assert(fabsf(g_lean_offset - 5.0f) < 1e-4f);
    s_run_active = 0;
    settle(35.0f / 200.0f);
    assert(g_lean_offset == 0.0f);
    s_run_active = 1;
    speed = 0;
    settle(35.0f / 200.0f);
    assert(g_lean_offset == 0.0f);
    speed = 280;
    maximum = 0.0f;
    settle(35.0f / 200.0f);
    assert(g_lean_offset == 0.0f);
    return 0;
}
'''
    c_file = tmp_path / "lean_contract.c"
    c_file.write_text(source, encoding="utf-8")
    exe = tmp_path / "lean_contract.exe"
    build = subprocess.run([gcc, "-std=c99", "-Wall", "-Wextra", str(c_file), "-o", str(exe)], capture_output=True, text=True)
    assert build.returncode == 0, build.stdout + build.stderr
    run = subprocess.run([str(exe)], capture_output=True, text=True)
    assert run.returncode == 0, run.stdout + run.stderr