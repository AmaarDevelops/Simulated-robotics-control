# 🤖 Real-Time 1D Object Tracking and Proportional Control

## Project Overview

This project transitions the principles of real-time computer vision from analysis (Fitness Project) to **autonomous control**. It simulates a robotic turret or arm designed to lock onto and track a designated object (or person) by converting the object's horizontal screen position into a Proportional Control speed command.

The system uses **YOLOv8 Segmentation** to isolate the target and **Image Moments** to calculate the Centroid ($C_x$), generating a velocity command intended to drive the target to the center of the frame.

### Key Features

* **Closed-Loop Control:** Implements a Vision $\rightarrow$ Processing $\rightarrow$ Command feedback loop essential for all robotics and autonomous systems.
* **Proportional (P) Control:** Uses the $P$-Controller logic: $\text{Command} = K_p \times \text{Error}$ for smooth, real-time adjustments.
* **Real-Time Metrics:** Continuously calculates the pixel error and the required motor speed.
* **Simulated Hardware Output:** Prints the motor commands to the console, demonstrating a clean interface for integration with actual serial communication (e.g., Arduino, Raspberry Pi).

## 🚀 Getting Started

### Prerequisites

You need Python 3.8+ and the following libraries.

``bash

pip install numpy opencv-python ultralytics

Save the entire Python script as robot_tracker.py.

Run the script from your terminal:


python robot_tracker.py

The webcam feed will open. Move an object (like a colored ball or your hand) across the screen.

Observe the printed output: the Command value will be positive when the object is on the right of the center line (turn right) and negative when it's on the left (turn left).

Press 'q' to stop the simulation.

🧠 The Control System Breakdown

The system's "brain" is the Proportional Control Loop, which converts the object's position error into an actuator command.

## 🧠 The Control System Breakdown

The system's "brain" is the Proportional Control Loop, which converts the object's position error into an actuator command.

| Stage | Input Data | Core Formula / Logic | Purpose |
| :--- | :--- | :--- | :--- |
| **1. Vision (Input)** | `best_mask` (from YOLO) | $\mathbf{C_{x}} = \mathbf{M_{10} / M_{00}}$ | Find the target's current horizontal position ($C_x$). |
| **2. Error Calculation** | $C_x$ and $\text{Target}_{\text{X}}$ (center of frame) | $\text{Error} = C_{x} - \text{Target}_{\text{X}}$ | Quantify how far off-center the object is in pixels. |
| **3. Control Law** | Error and $\mathbf{K_p}$ (Proportional Gain) | $\text{Command} = K_p \times \text{Error}$ | Calculate the motor speed required to reduce the error to zero.  |
| **4. Output (Simulated)** | Command Speed | `print()` and `np.clip()` | Ensure the command is within safe limits (e.g., -100 to 100) and simulate sending it to a motor. |
