---

# SAFRS Camera Raspberry Pi

## AI Vision & Target Tracking Node (UDP Cluster Version)

The **Camera Raspberry Pi** is responsible for **visual perception, object detection, and target tracking** within the SAFRS AGV UDP-based distributed robotics system.

This node directly interfaces with a **USB Camera**, utilizes **TensorFlow Lite (TFLite)** for efficient edge inference, and calculates **targeting error vectors** to guide the turret or robot chassis, while publishing detection status and end signals back to the cluster.

---

## 🧠 Role in SAFRS System

Camera Raspberry Pi acts as the **"Eyes" of the system**.

### Responsibilities

* Capture video frames from **USB Camera** (640x480 @ 30fps)
* Perform **Object Detection** (EfficientDet-Lite1) to find potential targets
* Perform **Classification** (MobileNet + Color Heuristics) to distinguish **Ally vs. Enemy**
* Calculate **visual error (X/Y offset)** from the image center
* **Publish Data:**

| Topic Name | Description |
| --- | --- |
| `/error_xy` | Targeting correction vector |
| `/detect` | Target information |
| `/end` | Task completion signal |

* Manage **Camera State (ON/OFF)** based on System Mode

---

## 📡 Communication Overview

### Subscribed Topics

* `/mode` (`system_interfaces/msg/SystemMode`)
* Controls FSM (BOOT, STANDBY, TRACK_ALLY, TRACK_ENEMY, NAVI)



### Published Topics

* `/error_xy` (`geometry_msgs/msg/Vector3`)
* `x`: Horizontal error (pixels from center)
* `y`: Vertical error (pixels from center)
* `z`: Status flag (0.0 = Tracking, -1.0 = Lost)


* `/detect` (`system_interfaces/msg/Detect`)
* Label, Center X, Center Y


* `/end` (`system_interfaces/msg/EndSignal`)
* Triggered when target is centered and confidence is high



---

## 📁 Directory Structure

```text
camera_client_cluster2/
├── model/
│   ├── EfficientDet-Lite1.tflite           # Object Detection Model
│   └── monkey_classifier_quant_int8.tflite # Classification Model
│
├── src/
│   └── camera_client_node3.py              # Main Application (Provided Code)
│
├── package.xml
└── setup.py

```

---

## ⚙️ Configuration & Constants

### 1️⃣ Model Configuration

Hardcoded paths in `CameraNode`:

* **Detection:** `EfficientDet-Lite1.tflite` (Input: Dynamic/384x384)
* **Classification:** `monkey_classifier_quant_int8.tflite` (Input: 224x224)

### 2️⃣ Camera Settings

* Resolution: **640x480**
* Exposure: **-5** (Darkened for better color segmentation)
* Auto Exposure: **Enabled**

### 3️⃣ Logic Thresholds

* `CENTER_THRESHOLD`: **25 pixels** (Tolerance for "aimed" state)
* `CAMERA_OFF_DELAY`: **5.0 seconds** (Cool-down before releasing resource)

---

## 🚀 How to Run

### Launch Camera Node

```bash
ros2 run camera_client_cluster2 camera_client_node3

```

If the camera is connected correctly, you should see:

* `[INFO] CameraNode started...`
* `[INFO] Camera ON` (When mode switches to STANDBY/TRACK)
* A GUI window "Camera View" showing the feed with bounding boxes.

---

## 🧩 Internal Control Flow

```text
Camera Frame (640x480)
    ↓
Resize (to Model Input, e.g., 384x384)
    ↓
TFLite Inference (EfficientDet)
    ↓
Bounding Box Extraction
    ↓
ROI Crop & Color Analysis (Red/Blue Ratio)
    ↓
Secondary Inference (MobileNet Classifier)
    ↓
Logic: Determine ALLY / ENEMY
    ↓
Calculate Error (Center - Object)
    ↓
Publish /error_xy & /detect

```

---

## ⚠️ Notes & Best Practices

* **Lighting matters:** The camera exposure is set low (-5) to highlight colored LEDs/markers. Ensure consistent lighting.
* **Thread Safety:** Image capture and Inference run in separate threads (`capture_loop`, `inference_loop`) using `threading.Lock()` to prevent race conditions.
* **Model Paths:** Ensure absolute paths to `.tflite` files are correct in the script.
* **Performance:**
* Raspberry Pi 4: Expect ~5-10 FPS inference.
* PC/Jetson: Expect 30+ FPS.



---

## 🔧 Hardware Assumptions

* **Camera:** Standard USB Webcam (Video0)
* **Compute:** Raspberry Pi 4 or equivalent (ARM64)
* **Dependencies:**
* `tflite_runtime` or `tensorflow`
* `opencv-python`
* `ros2-humble` (or compatible)



---

## 📜 License

SAFRS Robotics Platform

License: MIT

---

## 🙋 Maintainer

**지윤목장**

SAFRS Robotics Team

---
