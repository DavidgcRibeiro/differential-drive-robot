#  Differential Drive Robot — Autonomous Navigation with LiDAR

> Autonomous differential-drive robot with odometry, LiDAR-based obstacle avoidance, color-guided maze navigation, and WebSerial telemetry.

Developed as part of the **Sistemas Embebidos (SE)** curricular unit at the **University of Coimbra (FCTUC/DEEC)**, 2025/2026.

---

##  Overview

A dual-microcontroller autonomous mobile robot built around a STM32L432 (motion control, odometry, FreeRTOS) and an ESP32-S3 (LiDAR, color sensing, Wi-Fi telemetry). The robot navigates an unknown environment using a VL53L7CX 8×8 ToF LiDAR for obstacle avoidance and a TCS34725 color sensor for maze logic — stopping on red, turning 180° on blue.

The full system is accessible via browser at `http://robot.local` with real-time WebSerial telemetry and WASD keyboard control.

---

##  Features

-  **Odometry at 100 Hz** — real-time pose estimation (x, y, θ) using incremental encoders on the STM32
-  **LiDAR Obstacle Avoidance** — VL53L7CX 8×8 grid with 3-zone logic (left / center / right)
-  **Color-Guided Navigation** — TCS34725 detects blue (180° turn) and red (stop)
-  **WebSerial Telemetry** — real-time robot data accessible at `http://robot.local`
-  **WASD Manual Control** — browser-based keyboard control via Wi-Fi
-  **GOTO Navigation Mode** — autonomous navigation to a target (x, y) coordinate
-  **STM32 ↔ ESP32 via SPI** — high-speed inter-processor communication

---

##  Hardware Components

| Component | Description |
|---|---|
| **STM32L432KBU6** | Main microcontroller — odometry, motor control, FreeRTOS |
| **ESP32-S3-WROOM-1** | Wi-Fi module — LiDAR, color sensor, WebSerial, web interface |
| **DRV8874PWR** | Dual DC motor driver |
| **2× DC Motors** | With incremental quadrature encoders (gear ratio 300, 7 PPR) |
| **VL53L7CX** | 8×8 ToF LiDAR — frontal obstacle detection |
| **TCS34725** | RGB color sensor — floor color detection for maze navigation |

---

##  Repository Structure

```
differential-drive-robot/
│
├── STM32/                    # STM32L432 firmware (STM32CubeIDE)
│   └── PeripheralsCheck/
│       ├── Core/
│       │   ├── Src/
│       │   │   ├── main.c        # FreeRTOS tasks: Odometry, Comms, MotorControl
│       │   │   ├── comms.c       # STM32 ↔ ESP32 SPI communication
│       │   │   └── freertos.c
│       │   └── Inc/
│       ├── Drivers/
│       └── Middlewares/          # FreeRTOS, STM32 HAL, USB CDC
│
└── ESP32/                    # ESP32-S3 firmware (PlatformIO)
    └── peripheralsCheck/
        ├── src/
        │   └── main.cpp          # LiDAR, color sensor, WebSerial, web server
        ├── lib/
        │   ├── VL53L7CX/         # ST ToF LiDAR library
        │   └── comms/            # ESP32 ↔ STM32 SPI communication
        └── platformio.ini
```

---

##  System Architecture

```
┌─────────────────────────────────┐         SPI        ┌──────────────────────────────┐
│        STM32L432KBU6            │ ◄────────────────► │       ESP32-S3-WROOM-1       │
│                                 │                     │                              │
│  FreeRTOS Tasks:                │                     │  • VL53L7CX LiDAR (I2C)     │
│  • Odometry @ 100Hz             │                     │  • TCS34725 Color (I2C)      │
│  • Motor Control (PWM)          │                     │  • WebSerial Telemetry       │
│  • Comms (SPI slave)            │                     │  • Web Server (robot.local)  │
│                                 │                     │  • WASD Control              │
│  DRV8874PWR → 2× DC Motors      │                     │  • GOTO Navigation           │
│  with Encoders (TIM1/TIM2)      │                     │                              │
└─────────────────────────────────┘                     └──────────────────────────────┘
```

---

##  Key Implementation Details

- **Odometry** — quadrature encoder reading via STM32 timer input capture (TIM1/TIM2), pose updated at 100 Hz using `vTaskDelayUntil`
- **Obstacle Avoidance** — VL53L7CX 8×8 grid divided into 3 zones; obstacle threshold at 170 mm with minimum 2 valid pixels per zone
- **Color Navigation** — blue detection triggers 180° in-place rotation; red detection triggers full stop
- **GOTO Mode** — autonomous navigation to target (x, y) using angular and linear velocity control
- **Mechanical Parameters** — wheel diameter 45.5 mm, wheelbase 115 mm, encoder resolution 8400 counts/rev (7 PPR × 300 gear × 4 quadrature)

---

##  Development Environment

- **STM32CubeIDE** for STM32 firmware (HAL + FreeRTOS CMSIS v2)
- **PlatformIO** (VS Code) for ESP32 firmware
- **WebSerial** for real-time browser telemetry

---

## 👥 Team

This project was developed by:

- **David Ribeiro** — [@DavidgcRibeiro](https://github.com/DavidgcRibeiro)
- **Tomás Pereira** — [@PesteBuf](https://github.com/PesteBuf)

---

## 🎓 Academic Context

**Course:** Sistemas Embebidos (SE)  
**Institution:** Faculdade de Ciências e Tecnologia da Universidade de Coimbra (FCTUC)  
**Department:** Departamento de Engenharia Eletrotécnica e de Computadores (DEEC)  
**Year:** 2025/2026
