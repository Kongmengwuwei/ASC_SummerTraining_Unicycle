from pathlib import Path
import base64
import json
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from app.core.models import DeviceProfile,write_document
from app.protocols.firewater import FireWaterParser
from app.transports.mock import MockTransport

root=Path(__file__).resolve().parents[1]
profile=DeviceProfile.load(root/"app/profiles/tc264_unicycle/profile.json")
mock=MockTransport(profile,faults=True);mock.open()
parser=FireWaterParser();output=root/"examples/demo_session";output.mkdir(parents=True,exist_ok=True)
with (output/"raw.txt").open("wb") as raw,(output/"frames.jsonl").open("w",encoding="utf-8") as f:
    def log(kind,t,**data):f.write(json.dumps(dict(schema_version=1,kind=kind,time=t,pc_time=1788624000+t,**data),ensure_ascii=False)+"\n")
    log("event",0,event="CONNECTED",message="Mock 演示会话：软件数据，不来自实车")
    mock.write(b"cfg:hello,1\r\ncfg:schema,2\r\n")
    for i in range(1600):
        t=i*.02
        if i==600:mock.command_fault="reject";mock.write(b"cfg:set,3,r_rate_kp,12\r\n");log("event",t,event="PARAMETER",message="Mock 拒绝示例")
        if i==1400:mock.write(b"stop\r\n");log("event",t,event="STOP_SENT",message="软件停车命令")
        mock.telemetry();data=bytes(mock.queue);mock.queue.clear()
        raw.write(data);log("raw",t,base64=base64.b64encode(data).decode())
        parser.feed(data,received=t)
        for frame in parser.parsed_frames():
            log("frame",t,tag=frame.tag,values=frame.values,raw=frame.raw,error=frame.error,mcu_uptime=frame.values[0] if frame.tag in ("run","stat") else None)
    log("event",32,event="DISCONNECTED",message="Mock 示例结束；停车后保留 4 秒")
write_document(output/"session.json","session",dict(profile=profile.data,created=1788624000,protocol="1",firmware="mock-0.1",param_revision=1,description="模拟数据，包含回绕、丢帧、无效行、拒绝和停车；不是硬件测量"))
print(output)
