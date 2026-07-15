from fastapi import APIRouter, Request, WebSocket
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from pydantic import BaseModel
from typing import Dict

import asyncio
import os
import cv2

import ros_service

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

router = APIRouter()
source_node = None
end_node = None

templates = Jinja2Templates(directory=os.path.join(BASE_DIR, "templates"))

@router.get("/", response_class=HTMLResponse)
def home(request: Request):
    return templates.TemplateResponse(
        "index.html",
        {"request": request}
    )

class Command(BaseModel):
    cmd: str

class ButtonPress(BaseModel):
    button: str

class ActionGains(BaseModel):
    k_tangent: float
    k_offset: float

class HeadingGains(BaseModel):
    # Per-action heading tuning keyed by action name (default/left/straight/right).
    actions: Dict[str, ActionGains]

class VScale(BaseModel):
    velocity: float



@router.post("/button")
def button(data: ButtonPress):
    global source_node, end_node
    
    if data.button == "reset":
        source_node = -1
        end_node = -1
        ros_service.publish_route(source_node, end_node)
        return {"status": "Reset duckiebot"}
    
    # Nummernbuttons
    if data.button.startswith("bot"):

        number = int(data.button.replace("bot", ""))
        source_node = number
        return {"status": f"Source = {number}"}

    elif data.button.startswith("flag"):

        number = int(data.button.replace("flag", ""))
        end_node = number
        ros_service.publish_route(source_node, end_node)

        return {
            "status": "Published",
            "source": source_node,
            "end": end_node
        }

    return {"error": "Invalid state"}

@router.post("/command")
def command(data: Command):

    ros_service.publish_command(data.cmd)

    return {"status": data.cmd}

@router.post("/controller")
def controller(data: HeadingGains):

    ros_service.publish_heading_pid(data.actions)

    return {
        "status": "ok",
        "actions": {
            name: {"k_tangent": g.k_tangent, "k_offset": g.k_offset}
            for name, g in data.actions.items()
        },
    }

@router.post("/velocity")
def velocity(data: VScale):

    ros_service.publish_velocity(data.velocity)

    return {"status": "ok", "velocity": data.velocity}


# Websocket stream
@router.websocket("/ws/")
async def ws_stream(websocket: WebSocket):

    await websocket.accept()

    while True:

        if ros_service.latest_frame is None:
            await asyncio.sleep(0.01)
            continue

        try:
            _, buffer = cv2.imencode(
                ".jpg",
                ros_service.latest_frame
            )

            await websocket.send_bytes(buffer.tobytes())

        except Exception:
            break

        await asyncio.sleep(0.03)

async def _stream_jpeg(websocket: WebSocket, attr: str):
    """Stream the latest raw JPEG bytes held under ros_service.<attr> to the browser."""
    await websocket.accept()

    while True:
        frame = getattr(ros_service, attr)
        if frame is None:
            await asyncio.sleep(0.03)
            continue

        try:
            await websocket.send_bytes(frame)
        except Exception:
            break

        await asyncio.sleep(0.05)

@router.websocket("/ws/bev")
async def ws_bev(websocket: WebSocket):
    await _stream_jpeg(websocket, "latest_bev")

@router.websocket("/ws/trajectory")
async def ws_trajectory(websocket: WebSocket):
    await _stream_jpeg(websocket, "latest_trajectory")

@router.websocket("/ws/path_points")
async def ws_path_points(websocket: WebSocket):
    await websocket.accept()

    last_data = None

    while True:
        try:
            if ros_service.latest_path_points != last_data:
                last_data = ros_service.latest_path_points.copy()

                await websocket.send_json({
                    "path_points": last_data
                })

            await asyncio.sleep(0.05)

        except Exception:
            break