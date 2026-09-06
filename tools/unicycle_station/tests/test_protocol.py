import math
import pytest
from app.protocols.firewater import FireWaterParser, UptimeTracker
from app.core.models import Frame


def runline(stamp=0,flags=0):
    values=[stamp]+list(range(1,24))+[flags]
    return ("run:"+",".join(map(str,values))+"\n").encode()

def parse(data):
    p=FireWaterParser();p.feed(data);return p.parsed_frames()


def test_att_run_continuous_yaw():
    frames=parse(b"att:1,2,1080\n"+runline(42,19))
    assert frames[0].values==[1,2,1080]
    assert len(frames[1].values)==25 and frames[1].values[0]==42

def test_partial_glued_and_crlf():
    p=FireWaterParser();p.feed(b"att:1,");assert p.parsed_frames()==[]
    p.feed(b"2,3\r\n\ratt:4,5,6\ratt:7,8,9\n\n")
    assert [x.values for x in p.parsed_frames()]==[[1,2,3],[4,5,6],[7,8,9]]

@pytest.mark.parametrize("data",[b"att:1,2\n",b"att:1,2,3,4\n",b"att:nan,2,3\n",b"att:inf,2,3\n",b"att:1e999,2,3\n",b"att:1x,2,3\n",b"att:,,\n",b"xx\xff\n"])
def test_bad_fields(data):assert parse(data)[0].error

def test_unknown_numeric_tag_and_raw_unknown_line():
    a,b=parse(b"custom:1,2,3,4\nhello world\n")
    assert a.tag=="custom" and len(a.values)==4
    assert b.raw=="hello world" and b.error

def test_bound_and_recovery():
    p=FireWaterParser();p.feed(b"x"*100000+b"\natt:1,2,3\n")
    frames=p.parsed_frames();assert frames[0].error=="LINE_TOO_LONG" and not frames[1].error
    assert len(p.buffer)==0

def test_uptime_wrap_and_gap():
    t=UptimeTracker();t.update(0xfffffff0)
    seconds,missing,event=t.update(4)
    assert t.wraps==1 and missing==0 and seconds==.02
    seconds,missing,event=t.update(64)
    assert missing==2 and event=="TELEMETRY_GAP" and t.lost==2
    t.update(10);assert t.resets==1

def test_encoder_no_remote_start_or_injection():
    p=FireWaterParser()
    assert p.encode_command("stop",0)==b"stop\r\n"
    assert p.encode_command("set",7,"r_rate_kp",12)==b"cfg:set,7,r_rate_kp,12\r\n"
    for op,args in [("start",()),("set",("r_rate_kp\r\nstop",1)),("set",("r_rate_kp","nan")),("get",("x"*64,))]:
        with pytest.raises(ValueError):p.encode_command(op,1,*args)

@pytest.mark.parametrize("line",["par:1,x,float,nan,0,1,Other,.1,4", "par:1,x,float,0,1,0,Other,.1,4", "par:1,x,bogus,0,0,1,Other,.1,4", "rsp:0,ok,set,x,1"])
def test_invalid_schema_and_seq(line):assert parse((line+"\n").encode())[0].error
