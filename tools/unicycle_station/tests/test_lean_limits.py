"""Execute extracted firmware paths; no vehicle dynamics or target timing model."""
from pathlib import Path
import json
import re
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[3]
NAMES = ('lean_turn_kp', 'lean_speed_kp', 'lean_slew_dps', 'lean_max_angle')


def _function(source, name):
    match = re.search(r'^(?:static )?(?:float|void|uint8|uint32) ' + name + r'\([^\n]*\)\s*\{.*?^\}', source, re.M | re.S)
    assert match, name
    return match.group()


def _compile_run(tmp_path, source):
    gcc = shutil.which('gcc') or 'D:/Tools/Dev-Cpp/MinGW64/bin/gcc.exe'
    if not Path(gcc).exists():
        pytest.skip('Optional GCC host compiler not installed')
    c_file = tmp_path / 'contract.c'
    c_file.write_text(source, encoding='utf-8')
    exe = tmp_path / 'contract.exe'
    build = subprocess.run([gcc, '-std=c99', '-Wall', '-Wextra',
        '-I', str(ROOT/'tools/unicycle_station/tests/firmware_host'),
        '-I', str(ROOT/'code'), str(c_file), '-o', str(exe)], capture_output=True, text=True)
    assert build.returncode == 0, build.stdout + build.stderr
    run = subprocess.run([str(exe)], capture_output=True, text=True)
    assert run.returncode == 0, run.stdout + run.stderr


@pytest.mark.parametrize('relative', [
    'tools/unicycle_station/app/profiles/tc264_unicycle/profile.json',
    'tools/unicycle_station/examples/default.workspace.json',
])
def test_public_lean_ranges_and_defaults_match_firmware(relative):
    data = json.loads((ROOT/relative).read_text(encoding='utf-8'))
    profile = data.get('profile', data)
    params = {p['name']: p for p in profile['mock_parameters']}
    source = (ROOT/'code/param.c').read_text(encoding='utf-8')
    macros = dict(re.findall(r'^#define\s+(\w+)\s+([^\r\n/]+)',
        (ROOT/'code/board_config.h').read_text(encoding='utf-8'), re.M))
    def number(value):
        value = value.strip().strip('()').strip()
        return number(macros[value]) if value in macros else float(value.rstrip('fFuUlL'))
    expected = dict(zip(NAMES, [(0, .1, .01), (0, 2, 1), (1, 60, 24), (0, 10, 5)]))
    assert len(params) == len(re.findall(r'\{\s*"[^"]+",\s*&g_param\.', source))
    assert 'lean_roll_kp' not in params and 'lean_roll_kp' not in profile['parameters']
    for name, (lo, hi, default) in expected.items():
        bounds = re.search(r'\{\s*"'+name+r'",\s*&g_param\.\w+,\s*1,\s*([^,]+),\s*([^}]+)\}', source)
        assert tuple(number(v) for v in bounds.groups()) == (lo, hi)
        macro = re.search(r'g_param\.'+name+r'\s*=\s*(\w+);', source).group(1)
        assert number(macro) == default
        p = params[name]
        assert (p['min'], p['max'], p['value']) == (lo, hi, default)
        assert p['flags'] == 21
        assert profile['parameters'][name]['default'] == default
    assert number(macros['ROLL_TARGET_LIMIT']) == 10
    assert number(macros['LEAN_DT_S']) == number(macros['CTRL_DIV_ATT']) * number(macros['CTRL_PERIOD_MS']) / 1000


def test_real_lean_function_steady_turn_speed_slew_and_invalid_inputs(tmp_path):
    control = (ROOT/'code/control.c').read_text(encoding='utf-8')
    source = r'''
#include "board_config.h"
#include <assert.h>
#include <math.h>
static float turn=LEAN_TURN_KP_DEFAULT, gain=LEAN_SPEED_KP_DEFAULT;
static float maximum=LEAN_MAX_ANGLE_DEFAULT, slew_setting=LEAN_SLEW_DPS_DEFAULT;
#define LEAN_TURN_KP turn
#define LEAN_SPEED_KP gain
#define LEAN_MAX_ANGLE maximum
#define LEAN_SLEW_DPS slew_setting
#define TRACK_MODE_HOLD 3
static float g_lean_offset, s_lean_speed_mps, s_direction_yaw_rate_cmd, speed;
static int s_run_active=1, s_run_vision_armed=1, g_track_valid=1, g_vision_ipm_ok=1;
static int g_vision_age_ms, s_direction_last_mode;
static float Y_Motor_GetSpeedMps(void) { return speed; }
static int ctrl_is_finite(float v) { return isfinite(v); }
static float constrain_float(float v,float lo,float hi) { return v<lo?lo:(v>hi?hi:v); }
'''
    source += _function(control, 'lean_offset_update') + r'''
static void near(float actual,float expected) {
    static int check; ++check;
    if (!(fabsf(actual-expected)<2e-4f)) fprintf(stderr,"check %d: actual=%g expected=%g\n",check,actual,expected);
    assert(fabsf(actual-expected)<2e-4f);
}
static void tick(void) {
    float before=g_lean_offset;
    float next=lean_offset_update();
    assert(isfinite(next) && fabsf(next)<=10.00001f);
    if (isfinite(before) && fabsf(before)<=10)
        assert(fabsf(next-before)<=LEAN_SLEW_DPS_MAX*LEAN_DT_S+1e-5f);
    g_lean_offset=next;
}
static void settle(float v,float r) {
    speed=v;s_direction_yaw_rate_cmd=r;
    for(int i=0;i<6000;i++) tick();
}
static void defaults(void) {
    turn=LEAN_TURN_KP_DEFAULT;gain=LEAN_SPEED_KP_DEFAULT;
    maximum=LEAN_MAX_ANGLE_DEFAULT;slew_setting=LEAN_SLEW_DPS_DEFAULT;
    g_lean_offset=s_lean_speed_mps=0;
    s_run_active=s_run_vision_armed=g_track_valid=g_vision_ipm_ok=1;
    g_vision_age_ms=s_direction_last_mode=0;
}
int main(void) {
    /* Sustained turns remain banked without heading tracking error / pixel gate. */
    settle(.6f,30);float expected=28*(.01f+.6f/LEAN_GRAVITY_MPS2);
    near(g_lean_offset,expected);
    settle(.6f,-30);near(g_lean_offset,-expected);
    settle(-.6f,30);near(g_lean_offset,-expected);
    settle(-.6f,-30);near(g_lean_offset,expected);
    settle(1,0);near(g_lean_offset,0); /* no standalone forward speed bias */
    settle(0,90);near(g_lean_offset,0);
    settle(.04f,90);near(g_lean_offset,0);
    settle(.175f,30);near(g_lean_offset,.5f*28*(.01f+.175f/LEAN_GRAVITY_MPS2));
    settle(.6f,2);near(g_lean_offset,0);
    settle(.6f,2.01f);near(g_lean_offset,.01f*(.01f+.6f/LEAN_GRAVITY_MPS2));
    turn=0;settle(.6f,30);near(g_lean_offset,28*.6f/LEAN_GRAVITY_MPS2);
    gain=0;turn=.01f;settle(.6f,30);near(g_lean_offset,.28f);
    turn=0;settle(.6f,30);near(g_lean_offset,0);
    defaults();settle(1,60);near(g_lean_offset,5);
    maximum=10;settle(1,60);near(g_lean_offset,58*(.01f+1/LEAN_GRAVITY_MPS2));
    maximum=99;turn=99;gain=99;settle(1.5f,120);near(g_lean_offset,10);
    settle(1.5f,-120);near(g_lean_offset,-10);
    maximum=0;settle(1,60);near(g_lean_offset,0);
    /* Slew units are degrees/sec at 5 ms, including smooth reversal / exit. */
    defaults();speed=1;s_lean_speed_mps=1;s_direction_yaw_rate_cmd=60;
    tick();near(g_lean_offset,.12f);
    slew_setting=60;tick();near(g_lean_offset,.42f);
    slew_setting=1;tick();near(g_lean_offset,.425f);
    slew_setting=0;tick();near(g_lean_offset,.430f); /* runtime corrupt bound clamps */
    slew_setting=NAN;tick();near(g_lean_offset,.55f);
    slew_setting=24;s_run_active=0;tick();near(g_lean_offset,.43f);near(s_lean_speed_mps,0);
    settle(1,60);near(g_lean_offset,0);
    int *gates[]={&s_run_active,&s_run_vision_armed,&g_track_valid,&g_vision_ipm_ok};
    for(int j=0;j<4;j++) {
        defaults();settle(.6f,30);*gates[j]=0;settle(.6f,30);
        near(g_lean_offset,0);near(s_lean_speed_mps,0);
        *gates[j]=1;settle(.6f,30);near(g_lean_offset,expected);
    }
    defaults();g_vision_age_ms=VISION_LINK_TIMEOUT_MS-1;settle(.6f,30);near(g_lean_offset,expected);
    g_vision_age_ms++;settle(.6f,30);near(g_lean_offset,0);
    defaults();s_direction_last_mode=TRACK_MODE_HOLD;settle(.6f,30);near(g_lean_offset,0);
    defaults();settle(NAN,30);near(g_lean_offset,0);
    settle(INFINITY,30);near(g_lean_offset,0);
    settle(1,NAN);near(g_lean_offset,0);
    turn=NAN;settle(1,30);near(g_lean_offset,0);
    defaults();gain=NAN;settle(1,30);near(g_lean_offset,0);
    defaults();maximum=NAN;settle(1,30);near(g_lean_offset,0);
    defaults();g_lean_offset=NAN;s_lean_speed_mps=NAN;settle(.6f,30);near(g_lean_offset,expected);
    /* Filtering cannot retain old motion on a new run/reset. */
    defaults();settle(1,60);s_run_active=0;tick();near(s_lean_speed_mps,0);
    s_run_active=1;settle(0,60);near(g_lean_offset,0);
    return 0;
}
'''
    _compile_run(tmp_path, source)


def test_real_flash_load_preserves_old_tuning_and_ignores_old_lean_gain(tmp_path):
    param = (ROOT/'code/param.c').read_text(encoding='utf-8')
    table = re.search(r'const param_desc_t g_param_table\[\] =\s*\{.*?^\};', param, re.M | re.S).group()
    flash_defines = '\n'.join(re.findall(r'^#define PARAM_(?:FLASH_\w+|MAGIC_INDEX|COUNT_INDEX|FIRST_RECORD_INDEX|MAX_RECORDS)\s+[^\n]+', param, re.M))
    source = r'''
#include "param.h"
#include "board_config.h"
#include <assert.h>
#include <math.h>
param_t g_param;
volatile uint32 g_param_revision;
typedef union { float float_type; uint32 uint32_type; int32 int32_type; } flash_data_union;
#define EEPROM_PAGE_LENGTH 512
static flash_data_union flash_union_buffer[EEPROM_PAGE_LENGTH];
static uint8 flash_check(uint32 sector,uint32 page) { (void)sector;(void)page;return 1; }
static void flash_read_page_to_buffer(uint32 sector,uint32 page) { (void)sector;(void)page; }
'''
    source += table + '\n#define PARAM_TABLE_NUM (sizeof(g_param_table)/sizeof(g_param_table[0]))\n' + flash_defines + '\n'
    for name in ('param_float_is_finite','param_crc32_words','param_name_key','param_keys_unique',
                 'param_apply_record','param_normalize_dirs','param_load_defaults','param_init'):
        source += _function(param, name) + '\n'
    source += r'''
static void record(int index,const char *name,float value) {
    flash_union_buffer[2+2*index].uint32_type=param_name_key(name,1);
    flash_union_buffer[3+2*index].float_type=value;
}
int main(void) {
    assert(param_keys_unique());
    assert(PARAM_TABLE_NUM==74 && PARAM_TABLE_NUM<=PARAM_MAX_RECORDS);
    flash_union_buffer[0].uint32_type=PARAM_MAGIC;
    flash_union_buffer[1].uint32_type=5;
    record(0,"lean_roll_kp",100);record(1,"lean_max_angle",8);
    record(2,"p_angle_kp",17);record(3,"r_rcy_kp",.0016f);record(4,"roll_zero_init",.6f);
    flash_union_buffer[12].uint32_type=param_crc32_words((const uint32*)flash_union_buffer,12);
    param_init();
    assert(g_param.lean_turn_kp==LEAN_TURN_KP_DEFAULT);
    assert(g_param.lean_speed_kp==LEAN_SPEED_KP_DEFAULT);
    assert(g_param.lean_slew_dps==LEAN_SLEW_DPS_DEFAULT);
    assert(g_param.lean_max_angle==8 && g_param.p_angle_kp==17);
    assert(g_param.r_rcy_kp==.0016f && g_param.roll_zero_init==.6f);
    /* A new valid record wins over defaults and is clamped by current bounds. */
    record(0,"lean_speed_kp",99);
    flash_union_buffer[12].uint32_type=param_crc32_words((const uint32*)flash_union_buffer,12);
    param_init();assert(g_param.lean_speed_kp==LEAN_SPEED_KP_MAX);
    /* Corrupt CRC rejects the page, not just its renamed gain. */
    flash_union_buffer[12].uint32_type^=1;
    param_init();assert(g_param.lean_speed_kp==LEAN_SPEED_KP_DEFAULT);
    assert(g_param.lean_max_angle==LEAN_MAX_ANGLE_DEFAULT);
    return 0;
}
'''
    _compile_run(tmp_path, source)