````md
# :로켓: SAFRS AGV Robotics Platform — Camera Pi Overview
**Unified Multi-Raspberry-Pi Cluster for an Autonomous Ground Vehicle**
The SAFRS AGV platform is a **distributed robotics system** using four Raspberry Pi units.
This document is the **Camera Pi (A-Pi) overview** based on `camera_client_node3.py`.
---
## :뇌: System Architecture (4-RPi Cluster)
### :압정: Role Overview
| Raspberry Pi | Role | Responsibilities |
|--------------|------|-----------------|
| **Main Pi** | Central Brain | Mode control publish (`/system_mode`), decision logic (ALLY/ENEMY), system orchestration |
| **LiDAR Pi** | Mapping & LaserScan | LiDAR driver, SLAM, `/scan` |
| **Camera Pi (A-Pi)** | Vision System | **USB camera capture → MediaPipe Pose ROI → TFLite(INT8)+HSV classify → publish `/detect`, `/error_xy`, `/end`** |
| **Motor Pi** | Hardware Control | `/error_xy` subscribe, gimbal/trigger control, `/odom` publishing |
---
### :자오선이_있는_지구: Cluster Communication Overview
| Source Pi | Main Pi Receives               | Main Pi Publishes |
|-----------|--------------------------------|-------------------|
| Camera Pi | `/detect`, `/error_xy`, `/end` | `/system_mode`    |
| LiDAR Pi  | `/scan`                        | —                 |
| Motor Pi  | `/odom`, `/tf`                 | —                 |
---
### :전기_플러그: Default ZMQ Port Map
| Function       | Port |
|----------------|------|
| Camera → Main  | —    |
| LiDAR → Main   | —    |
| Main → Motor   | —    |
| Motor → Main   | —    |
---
### :포장: Software Requirements
| Tool               | Version      | Purpose               |
|--------------------|--------------|-----------------------|
| ROS2 Humble        | Required     | rclpy pub/sub         |
| Python 3.10        | Required     | Node runtime          |
| OpenCV (cv2)       | Required     | Camera capture / debug |
| MediaPipe Tasks    | Required     | PoseLandmarker ROI     |
| TFLite Runtime     | Required     | INT8 inference          |
| NumPy              | Required     | Array / ROI ops         |
---
## :파일_폴더: Directory Structure (Top Level)
```bash
SAFRS/
├── Main_Raspberry_Pi/
│   └── main_pi/
│
├── Lidar_Raspberry_Pi/
│   ├── lidar_driver/
│   └── cartographer_mapping/
│
├── Camera_Raspberry_Pi/
│   └── camera_pi/
│       └── camera_client_cluster2/
│           ├── package.xml
│           ├── setup.py
│           ├── setup.cfg
│           ├── resource/
│           │   └── camera_client_cluster2
│           ├── test/
│           │   ├── test_copyright.py
│           │   ├── test_flake8.py
│           │   └── test_pep257.py
│           └── camera_client_cluster2/
│               ├── __init__.py
│               ├── camera_client_node3.py
│               └── model/
│                   ├── monkey_classifier_quant_int8.tflite
│                   └── EfficientDet-Lite1.tflite
│
├── Motor_Raspberry_Pi/
│   └── motor_pi/
│
└── README.md
````

---

## :불: Module Descriptions

### :일: **Main Raspberry Pi — Central Navigation Controller**

Runs the core robotics stack:

* Mode control publishes `/system_mode`
* Receives `/detect`, `/error_xy`, `/end` from Camera Pi
  Launch:

```
ros2 launch project_hybrid_cluster main_pi.launch.py
```

---

### :둘: **LiDAR Raspberry Pi — Mapping + LaserScan Provider**

Provides:

* `/scan` via LiDAR driver
  Driver:

```
ros2 run lidar_driver lidar_driver_node
```

SLAM:

```
ros2 launch cartographer_mapping mapping.launch.py
```

---

### :셋: **Camera Raspberry Pi — Real-Time Vision Node**

Tasks:

* USB camera capture (640x480)
* MediaPipe PoseLandmarker (`pose_landmarker_lite.task`)로 사람 포즈 검출
* 어깨/골반 랜드마크(11,12,23,24)로 ROI 박스 생성
* ROI 중심(cx,cy)와 화면 중심 오차 → `/error_xy` 발행
* ROI를 TFLite INT8 분류기 + HSV 빨강비율 보정으로 ALLY/ENEMY 분류 → `/detect` 발행
* 목표가 dead zone에 들어오면 `/end` 발행

ROS2 Topics:

* Subscribe: `/system_mode` (`system_interfaces/SystemMode`)
* Publish:

  * `/detect` (`system_interfaces/Detect`) : `label`, `cx`, `cy`
  * `/error_xy` (`geometry_msgs/Vector3`) : `x`, `y`, `z` (정상=0.0, 장기 포즈 실패=z=-1.0)
  * `/end` (`system_interfaces/EndSignal`) : `type` = ALLY/ENEMY

Key Logic (camera_client_node3.py):

* Mode에 따라 카메라 자동 ON/OFF

  * ON: `STANDBY`, `TRACK_ALLY`, `TRACK_ENEMY`
  * OFF: 그 외 모드
* 흔들림/순간 누락 보정

  * 포즈 누락 시 `MAX_LOST_FRAMES=6` 동안 마지막 정상 오차(`last_valid_error`) 재발행
  * 그 이상 누락 시 `/error_xy.z = -1.0` 발행
* 분류 보정(HSV red ratio)

  * ROI 내 빨강 비율이 3% 초과면 딥러닝 결과가 ALLY여도 ENEMY로 강제
* dead zone 판정

  * `CENTER_THRESHOLD=20` 픽셀 범위 안이면 "중심 진입"
* STANDBY에서 /end 조건

  * 동일 label 연속 `DETECT_CONSECUTIVE=3` + 중심 진입 + 쿨다운(1초)
* TRACK에서 /end 조건

  * 중심 진입 + 쿨다운(1초)

Run:

```
ros2 run camera_client_cluster2 camera_client_node3
```

---

### :넷: **Motor Raspberry Pi — Hardware Layer**

Responsibilities:

* Subscribe `/error_xy`, execute tracking control
* Use `/end` for trigger decision
  Run as systemd service:

```
sudo systemctl start motor_pi.service
```

---

## :위성_안테나: TF Tree Overview

```bash
map
 └── odom
      └── base_link
           └── laser_frame
```

---

## :직소: Data Pipeline Summary

Perception (Camera Pi)

* `/system_mode` 수신 → 카메라 ON/OFF 및 STANDBY/TRACK 동작 결정
* 프레임 캡처 → PoseLandmarker로 사람 검출
* 랜드마크 기반 ROI 박스 생성 → 중심(cx,cy) 계산
* 중심 오차(error_xy) 발행 → 짐벌이 목표 중심으로 이동
* ROI 분류(TFLite INT8 + HSV red ratio) → `/detect` 발행
  End Condition
* dead zone(±20px) 진입 시 `/end` 발행(모드별 조건 상이)

---

## :로켓: How to Launch the Entire SAFRS System

### **:일: Start Sub-PIs**

Camera Pi

```
ros2 run camera_client_cluster2 camera_client_node3
```

LiDAR Pi

```
ros2 run lidar_driver lidar_driver_node
```

(Optional) SLAM:

```
ros2 launch cartographer_mapping mapping.launch.py
```

Motor Pi

```
sudo systemctl start motor_pi.service
```

---

### **:둘: Start Main Pi**

```
ros2 launch project_hybrid_cluster main_pi.launch.py
```

Includes:

* Mode control (`/system_mode`)
* Receives `/detect`, `/error_xy`, `/end`

---

## :압정: Best Practices

* TRACK 중 흔들림으로 포즈가 잠깐 끊겨도, `last_valid_error` 재발행으로 짐벌이 멈추지 않도록 유지
* `/error_xy.z = -1.0`는 "장기 타겟 로스트" 신호로 처리(모터/메인에서 상태 전환에 활용)
* 디버그 창(`cv2.imshow`)은 헤드리스 환경이면 비활성화 필요(필요 시 코드에서 옵션화)
* 카메라 인덱스가 바뀌면 `_open_camera()`의 idx 후보([0,1,2,4])를 환경에 맞게 조정

---

## :스크롤: License

SAFRS Robotics Platform
License: MIT (pending finalization)
-----------------------------------

## :상반신_그림자: Maintainers

**지윤목장**
SAFRS Robotics Team

```
```
