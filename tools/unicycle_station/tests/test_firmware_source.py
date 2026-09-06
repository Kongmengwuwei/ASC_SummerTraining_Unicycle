from pathlib import Path
import re
import shutil
import subprocess
import pytest

def test_mcu_protocol_host_contract(tmp_path):
    root=Path(__file__).parents[1]
    gcc=shutil.which("gcc") or "D:/Tools/Dev-Cpp/MinGW64/bin/gcc.exe"
    if not Path(gcc).exists():pytest.skip("Optional GCC host compiler not installed")
    exe=tmp_path/"firmware_contract.exe"
    result=subprocess.run([gcc,"-std=c99","-Wall","-Wextra","-I",str(root/"tests/firmware_host"),str(root/"tests/firmware_host/harness.c"),"-o",str(exe)],capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    result=subprocess.run([str(exe)],capture_output=True,text=True)
    assert result.returncode==0,result.stdout+result.stderr

def test_isr_bounded_no_configuration_work():
    root=Path(__file__).parents[3]
    text=(root/"code/vofa.c").read_text(encoding="utf-8")
    tick=text.split("void vofa_tick1ms(void)")[1].split("void vofa_cmd_poll(void)")[0]
    assert "i < VOFA_TX_FIFO_DEPTH" in tick
    assert "cfg_execute" not in tick and "snprintf" not in tick and "param_save(" not in tick
    control=(root/"code/control.c").read_text(encoding="utf-8")
    loop=control.split("void control_loop(void)")[1].split("uint8 control_camera_debug_start")[0]
    assert "cfg_" not in loop
