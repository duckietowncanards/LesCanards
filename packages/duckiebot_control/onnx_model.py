#!/usr/bin/env python
"""Standalone interactive tester for an exported LeE2E ONNX model.

Its only heavy deps are onnxruntime, numpy and opencv; the command-id table it shares with the
rest of the stack lives in the dependency-free solution.commands. Opens a pygame window with the
same three panels as eval.domain_gap
(input | predicted BEV | predicted trajectory) plus a header with the predicted action and speed.
Arrow keys change the command fed to the model (LEFT / RIGHT / UP = straight, DOWN = default);
ESC or Q quits. The normalization stats and speed timestep are read from the ONNX metadata that
eval.export_onnx embeds.

    python -m src.eval.test_onnx_model model.onnx frame.jpg
"""

from __future__ import annotations

from duckiebot_control.common_imports import *

IMG_SIZE = 224              # the model's fixed input size (aggregation.config.TargetImageConfig)
DEFAULT_TEMPORAL_DT = 0.25  # seconds between temporal points (collect.py default: 4 Hz)
LOOKAHEAD_INDEX = 2         # spatial waypoint highlighted as the steer target
PANEL = 256                 # each of the three panels is PANEL x PANEL
HUD_H = 40                  # header strip height for the action / speed text
TRAJ_RANGE_M = 10.0         # +/- meters shown around the car in the trajectory panel

# arrow key name -> command id; the DOWN key selects DEFAULT (the "just follow the lane" class).
KEY_TO_COMMAND = {"left": LEFT, "right": RIGHT, "up": STRAIGHT, "down": DEFAULT}
# BEV class colors in BGR, index = class id 0..7 (background, center_lane, side_lane, asphalt,
# stop_lane, sign, bot, duck) - copied from the project palette as plain data.
BEV_COLORS_BGR = np.array(
    [(30, 30, 30), (0, 215, 255), (255, 255, 255), (60, 60, 60),
     (30, 30, 200), (255, 120, 60), (0, 140, 255), (255, 0, 255)], dtype=np.uint8)
TRAJ_RANGE_M = 10.0  # +/- meters shown around the car, same window as the eval viewer

# BGR, values from lee2e's aggregation.command.COMMAND_COLORS_BGR — cv2 draws BGR
# natively, so unlike the pygame viewers no channel reversal is needed.
COMMAND_COLORS_BGR = {
    DEFAULT: (180, 180, 180),  # grey
    LEFT: (255, 80, 0),        # blue
    STRAIGHT: (0, 200, 0),     # green
    RIGHT: (0, 80, 255),       # red
}

def letterbox(image: np.ndarray, size: int = IMG_SIZE) -> np.ndarray:
    """Aspect-preserving resize + centre-pad to size x size - a faithful copy of the project's
    aggregation.imaging.letterbox, so the ONNX input matches what the model was trained on."""
    h, w = image.shape[:2]
    scale = size / max(h, w)
    new_w, new_h = round(w * scale), round(h * scale)
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((size, size, 3), np.uint8)
    y0, x0 = (size - new_h) // 2, (size - new_w) // 2
    canvas[y0 : y0 + new_h, x0 : x0 + new_w] = resized
    return canvas


def denorm(points: np.ndarray, c: dict, keys: tuple[str, str, str, str]) -> np.ndarray:
    """Inverse training normalization ((n + 1) / 2 * span + lo) for the (x_min, y_min, x_max, y_max)
    style key group `keys` in the norm constants `c`."""
    lo = np.array([c[keys[0]], c[keys[1]]])
    span = np.array([c[keys[2]] - c[keys[0]], c[keys[3]] - c[keys[1]]])
    return (points + 1.0) * 0.5 * span + lo


def colorize_bev(bev_idx: np.ndarray, size: int = 256) -> np.ndarray:
    """Map a (H, W) class-index BEV map to a BGR image via BEV_COLORS_BGR, nearest-upscaled to
    size x size. Mirrors eval.domain_gap's BEV panel."""
    colored = BEV_COLORS_BGR[np.asarray(bev_idx, np.int32) % len(BEV_COLORS_BGR)]
    return cv2.resize(colored, (size, size), interpolation=cv2.INTER_NEAREST)


def render_trajectory(spatial: np.ndarray, temporal: np.ndarray | None, size: int = 256,
                      range_m: float = TRAJ_RANGE_M, command: int = DEFAULT,
                      target_index: int | None = None,
                      selected_temporal_index: int | None = None) -> np.ndarray:
  
    canvas = np.full((size, size, 3), 20, np.uint8)
    scale = size / (2 * range_m)
    # Origin near the bottom edge (not centered) so the whole forward trajectory fits in
    # view instead of being cut off at the panel's top half - same layout as the
    # aggregation/eval PygameViewer trajectory panels.
    cx, cy = size // 2, size - 15

    def to_px(x: float, y: float) -> tuple[int, int]:
        return int(cx + y * scale), int(cy - x * scale)

    # duckiebot-frame axes through the origin: the vertical line is x (forward), the horizontal
    # one is y (left positive), with a tick every meter to give the dots a distance scale.
    axis_color = (70, 70, 70)
    cv2.line(canvas, (cx, 0), (cx, size - 1), axis_color, 1)
    cv2.line(canvas, (0, cy), (size - 1, cy), axis_color, 1)
    for m in range(1, int(range_m) + 1):
        d = int(m * scale)
        for px, py in ((cx, cy - d), (cx, cy + d), (cx - d, cy), (cx + d, cy)):
            if 0 <= px < size and 0 <= py < size:
                cv2.drawMarker(canvas, (px, py), axis_color, cv2.MARKER_CROSS, 4, 1)

    # Temporal trajectory first (underneath), connected point to point so the spacing
    # (= speed) reads at a glance; the point the target speed is read at marked magenta.
    if temporal is not None:
        prev = None
        for i, (x, y) in enumerate(np.asarray(temporal)):
            pt = to_px(x, y)
            is_selected = i == selected_temporal_index
            color = (255, 0, 255) if is_selected else (255, 200, 0)  # magenta / cyan
            if prev is not None:
                cv2.line(canvas, prev, pt, (255, 200, 0), 1)
            cv2.circle(canvas, pt, 5 if is_selected else 2, color, -1)
            prev = pt

    # Spatial waypoints as plain dots, colored by the active navigation command
    # (grey DEFAULT / blue LEFT / green STRAIGHT / red RIGHT, as in aggregation.visualize);
    # the point the lateral controller is chasing marked yellow.
    dot_color = COMMAND_COLORS_BGR[command]
    for i, (x, y) in enumerate(np.asarray(spatial)):
        is_target = i == target_index
        color = (0, 255, 255) if is_target else dot_color
        cv2.circle(canvas, to_px(x, y), 5 if is_target else 3, color, -1)

    cv2.circle(canvas, (cx, cy), 4, (255, 255, 255), -1)  # car origin
    return canvas


class OnnxModel:
    """ONNX session + the norm/timestep read from its embedded metadata; turns a BGR frame +
    command into meters-space predictions (spatial, temporal, BEV, action, speed)."""

    def __init__(self, onnx_path: str, providers: list[str] | None = None):
        available = ort.get_available_providers()
        print(f"[OnnxModel] onnxruntime {ort.__version__}")
        print(f"[OnnxModel] available providers: {available}")

        if providers is None:
            # Use the CUDA GPU on the Jetson, fall back to CPU. TensorRT is deliberately not
            # requested: it needs the (uninstalled) libnvinfer libs on top of CUDA and would only
            # spew load-failure warnings before falling back. Only request providers that are
            # actually compiled in, else the session errors out.
            preferred = ["CUDAExecutionProvider", "CPUExecutionProvider"]
            providers = [p for p in preferred if p in available]
        print(f"[OnnxModel] requesting providers: {providers}")

        self.session = ort.InferenceSession(onnx_path, providers=providers)
        active = self.session.get_providers()
        print(f"[OnnxModel] active providers: {active}")
        if active and active[0] == "CPUExecutionProvider":
            print("[OnnxModel] WARNING: running on CPU - GPU execution provider not in use")

        meta = self.session.get_modelmeta().custom_metadata_map
        if "trajectory_norm" not in meta:
            raise SystemExit("ONNX has no embedded norm stats; re-export with eval.export_onnx")
        self.norm = json.loads(meta["trajectory_norm"])
        self.dt = float(meta.get("temporal_dt", DEFAULT_TEMPORAL_DT))

    def predict(self, bgr: np.ndarray, command: int):
        letterboxed = letterbox(bgr)
        image_input = letterboxed.transpose(2, 0, 1)[None].astype(np.float32) / 255.0  # NCHW, BGR, [0,1]
        out = dict(zip(
            (o.name for o in self.session.get_outputs()),
            self.session.run(None, {"image": image_input, "command": np.array([command], np.int64)})))
        spatial = denorm(out["spatial"][0], self.norm, ("x_min", "y_min", "x_max", "y_max"))
        temporal = denorm(out["temporal"][0], self.norm, ("tx_min", "ty_min", "tx_max", "ty_max"))
        bev_idx = out["bev"][0].argmax(0)
        # Softmax, not a min-shift-and-normalize: the latter pins the lowest-scoring class to
        # exactly 0.0 and rescales the rest by the logit spread, so its magnitudes track how far
        # apart the logits happen to be rather than the model's confidence.
        logits = out["command_logits"][0]
        rospy.logwarn(f"My logits: {np.round(logits,2)}")
        action = np.exp(logits - np.max(logits))
        action = action / np.sum(action)
        speed_kmh = float(np.linalg.norm(temporal[0]) / self.dt * 3.6)  # dt-ahead point distance / dt
        return letterboxed, spatial, temporal, bev_idx, action, speed_kmh
