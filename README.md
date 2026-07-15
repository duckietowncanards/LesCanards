# ROS-Based System Architecture Integrating DuckAD

This project provides a complete ROS-based implementation for deploying **DuckAD** on a real Duckietown robot.

## Features

- A comprehensive web interface, accessible at:
  ```
  http://<ROBOTNAME>.local:8001
  ```
- Direct integration of the `model.onnx` neural network for real-time inference.
- Real-time autonomous navigation by sending driving commands based on a **predefined map**.
- The current implementation is optimized for a specific map layout. However, the path-planning framework isn't fixed and can be extended to support additional maps by modifying the `path_planning` directory.
- Designed for deployment on a physical Duckiebot.

## Getting Started

### Build the project

```bash
dts devel build -H canards.local
```

### Run the project

```bash
dts devel run -H canards.local -L path
```

## Requirements

- Duckietown Shell (`dts`)
- ROS
- A Duckietown robot
- A trained `model.onnx` model