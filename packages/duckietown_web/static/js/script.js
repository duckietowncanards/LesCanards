
async function press(button) {
    try {
        const response = await fetch("/button", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({
                button: button
            })
        });

        const data = await response.json();
        console.log(data);

    } catch (err) {
        console.error(err);
    }
}

async function send(cmd) {
    try {
        const response = await fetch("/command", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({
                cmd: cmd
            })
        });

        const data = await response.json();
        console.log(data);

        if (cmd === "reset") {
            resetUI();
        }
    } catch (err) {
        console.error(err);
    }

}

// Canonical action order, must match COMMAND_NAMES on the Python side.
const PID_ACTIONS = ["default", "left", "straight", "right"];

// Collect the k_tangent, k_offset for every action and send to the duckiebot in one request.
async function applyPidActions() {

    const actions = {};

    for (const action of PID_ACTIONS) {
        actions[action] = {
            k_tangent: parseFloat(document.getElementById(`k_tangent_${action}`).value),
            k_offset: parseFloat(document.getElementById(`kc_${action}`).value),
        };
    }

    try {
        const response = await fetch("/controller", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({ actions: actions }),
        });

        console.log(await response.json());
    } catch (err) {
        console.error(err);
    }
}

async function applyVScale() {

    const payload = {
        velocity: parseFloat(document.getElementById("velocity").value),
    };

    try {
        const response = await fetch("/velocity", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify(payload),
        });

        console.log(await response.json());
    } catch (err) {
        console.error(err);
    }
}

const ws = new WebSocket(`ws://${window.location.host}/ws/`);

ws.binaryType = "blob";

ws.onmessage = (event) => {
    const img = document.getElementById("cam");

    const url = URL.createObjectURL(event.data);
    img.src = url;

    img.onload = () => {
        URL.revokeObjectURL(url);
    };
};

ws.onopen = () => {
    console.log("WebSocket connected");
};

ws.onclose = () => {
    console.log("WebSocket disconnected");
};

ws.onerror = (err) => {
    console.error("WebSocket error:", err);
};

// Stream a websocket into an <img>
function streamJpegTo(path, imgId) {
    const sock = new WebSocket(`ws://${window.location.host}${path}`);
    sock.binaryType = "blob";
    sock.onmessage = (event) => {
        const img = document.getElementById(imgId);
        if (!img) return;
        const url = URL.createObjectURL(event.data);
        img.src = url;
        img.onload = () => URL.revokeObjectURL(url);
    };
}

// Model-prediction panels: BEV segmentation (carla1) and trajectory (carla2).
streamJpegTo("/ws/bev", "bev");
streamJpegTo("/ws/trajectory", "traj");

const NODE_COUNT = 13;

// Show only the clicked image of a given type (bot or flag), hide the rest.
function isolate(prefix, id) {

    for (let i = 1; i <= NODE_COUNT; i++) {
        const el = document.getElementById(prefix + i);
        if (el) el.setAttribute("visibility", i === id ? "visible" : "hidden");
    }

    if(prefix == "bot"){
        for (let i = 1; i <= NODE_COUNT; i++){
            const el = document.getElementById("flag" + i);
            if (el) el.setAttribute("visibility", i === id ? "hidden" : "visible");
        }
    }
}

function bindImage(prefix, i) {
    const el = document.getElementById(prefix + i);
    if (!el) return;
    if(prefix == "bot"){
        el.setAttribute("visibility", "visible");
    }
    else{
        el.setAttribute("visibility", "hidden");
    }
    el.style.cursor = "pointer";

    el.addEventListener("mouseenter", () => {
        el.style.opacity = "0.6";
    });

    el.addEventListener("mouseleave", () => {
        el.style.opacity = "1";
    });

    el.addEventListener("click", () => {
        press(prefix + i);
        isolate(prefix, i);
    });
}

function bindNodes() {
    for (let i = 1; i <= NODE_COUNT; i++) {
        bindImage("bot", i);
        bindImage("flag", i);
    }
}


function resetUI() {
    for (let i = 1; i <= NODE_COUNT; i++) {
        const bot = document.getElementById("bot" + i);
        const flag = document.getElementById("flag" + i);

        if (bot) {
            bot.setAttribute("visibility", "visible");
            bot.style.opacity = "1";
        }

        if (flag) {
            flag.setAttribute("visibility", "hidden");
            flag.style.opacity = "1";
        }

        for(let j = 1; j <= NODE_COUNT; j++){
             if(i == j){
                continue;
            }
            const routeFull = document.getElementById("s" + i + "g" + j + "full");
            const routeEmpty = document.getElementById("s" + i + "g" + j + "empty");
            if (routeFull) routeFull.setAttribute("visibility", "hidden");
            if (routeEmpty) routeEmpty.setAttribute("visibility", "hidden");
        }
    }
}

fetch("/static/images/final_map.svg")
  .then(res => res.text())
  .then(svg => {
      document.getElementById("map").innerHTML = svg;
      bindNodes();
  });

const pathSocket = new WebSocket(`ws://${window.location.host}/ws/path_points`);

pathSocket.onmessage = (event) => {
    const data = JSON.parse(event.data);

    console.log(data.path_points);

    for (let i = 0; i <= NODE_COUNT; i++) {
        for (let j = 0; j <= NODE_COUNT; j++) {

            if(i == j){
                continue;
            }
            if(data.path_points.includes(i) && data.path_points.includes(j) && data.path_points[data.path_points.indexOf(i) + 1] === j){
                let obj;
                if(j == data.path_points[data.path_points.length - 1]){
                    obj = document.getElementById("s" + i + "g" + j + "full");
                }
                else{
                    obj = document.getElementById("s" + i + "g" + j + "empty");
                }
                if (obj) obj.setAttribute("visibility", "visible");
            }
            else{
                const emptyObj = document.getElementById("s" + i + "g" + j + "empty");
                const fullObj = document.getElementById("s" + i + "g" + j + "full");
                if (emptyObj) emptyObj.setAttribute("visibility", "hidden");
                if (fullObj) fullObj.setAttribute("visibility", "hidden");
                }
        }
    }
};

const WRAPPER_W = 1920;
const WRAPPER_H = 1080;

function fitWrapper() {
    const wrapper = document.querySelector(".wrapper");
    if (!wrapper) return;

    const scale = Math.min(
        window.innerWidth / WRAPPER_W,
        window.innerHeight / WRAPPER_H
    );

    wrapper.style.transform = `scale(${scale})`;
}

window.addEventListener("resize", fitWrapper);
fitWrapper();

