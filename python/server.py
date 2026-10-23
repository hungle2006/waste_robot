import os
import secrets
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Optional

from fastapi import Depends, FastAPI, Header, HTTPException
from fastapi.responses import FileResponse, RedirectResponse
from pydantic import BaseModel, ConfigDict, StrictInt

from brain import Brain
from transport import SerialTransport, SimTransport

ROOT = Path(__file__).resolve().parent
TOKEN = os.environ.get("ROBOT_TOKEN", "")
SIMULATION = os.environ.get("ROBOT_SIMULATION") == "1"
brain = Brain(SimTransport() if SIMULATION else SerialTransport(
    os.environ.get("ARDUINO_PORT", "COM3" if os.name=="nt" else "/dev/ttyACM0")),
    os.environ.get("ROBOT_CONFIG",str(ROOT/"robot_params.py")), SIMULATION)


@asynccontextmanager
async def lifespan(app):
    if len(TOKEN)<24: raise RuntimeError("ROBOT_TOKEN cần ít nhất 24 ký tự")
    brain.start_monitor()
    yield
    brain.close()


app=FastAPI(title="Waste Robot · Python Brain Mecanum",version="3.16.14",lifespan=lifespan)


def auth(authorization: Optional[str]=Header(default=None)):
    if not TOKEN or not secrets.compare_digest(authorization or "", "Bearer "+TOKEN):
        raise HTTPException(401,"Sai khóa truy cập")


def call(fn,*args):
    try: return fn(*args)
    except ConnectionError as exc:
        brain.stop()
        raise HTTPException(503,str(exc)) from exc
    except ValueError as exc: raise HTTPException(409,str(exc)) from exc


class Command(BaseModel):
    model_config=ConfigDict(extra="forbid")
    epoch: StrictInt
    command: Optional[str]=None
    joint: Optional[str]=None
    pulse_us: Optional[StrictInt]=None
    speed: Optional[StrictInt]=None


class Pose(BaseModel):
    model_config=ConfigDict(extra="forbid")
    epoch: StrictInt
    name: str
    values: dict[str,StrictInt]


@app.get("/")
def root(): return RedirectResponse("/ui/robot-control.html")


@app.get("/ui/robot-control.html")
def ui(): return FileResponse(ROOT.parent/"web"/"robot-control.html", headers={"Cache-Control":"no-store"})


@app.post("/connect",dependencies=[Depends(auth)])
def connect(): return call(brain.connect)


@app.get("/status",dependencies=[Depends(auth)])
def status(): return brain.snapshot()


@app.post("/command",dependencies=[Depends(auth)])
def command(p:Command):
    if p.command == 'AUTO REAR ON':
        raise HTTPException(409,'Tự phân loại D18 cần camera/Modal: chạy python main.py và bật trên app robot')
    if (p.command is not None)==(p.joint is not None):
        raise HTTPException(422,"Nhập một lệnh hoặc một khớp")
    if (p.joint is None)!=(p.pulse_us is None):
        raise HTTPException(422,"Khớp phải đi cùng pulse_us")
    return call(brain.command,p.command,p.epoch,p.joint,p.pulse_us,p.speed)


@app.post("/poses",dependencies=[Depends(auth)])
def pose(p:Pose): return call(brain.save_pose,p.name,p.values,p.epoch)


@app.get("/health")
def health(): return {"running":True,"simulation":SIMULATION}
