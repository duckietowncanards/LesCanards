import os

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles

from routes import router
from ros_service import startup_ros

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

app = FastAPI()

app.mount(
    "/static",
    StaticFiles(directory=os.path.join(BASE_DIR, "static")),
    name="static",
)

app.include_router(router)

@app.on_event("startup")
def startup():
    startup_ros()