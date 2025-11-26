# GyroTrack
![Banner](https://github.com/zahednia/GyroTrack/blob/main/Photo.png)
**GyroTrack** is an experimental project for tracking the motion of a spinning ball in 3D using:

- an **ESP32** (ESP32-WROOM-32U),
- an **ICM20948** IMU (accelerometer + gyroscope),
- and a **Python visualizer** that compensates for the ball’s rotation.

The key idea:

> Even if the ball is spinning, GyroTrack tries to keep the **motion path (position) in a fixed world frame**, so the *spin* doesn’t break the *trajectory*.

---

## Features

- Reads raw **accelerometer** and **gyroscope** data from ICM20948 via ESP32  
- Serves sensor data as **JSON over Wi-Fi** (`/data` endpoint)
- Python client that:
  - Computes **orientation** from gyroscope (quaternion-based)
  - Rotates accelerometer data into a **fixed world frame** (spin compensation)
  - Integrates linear acceleration → velocity → position
  - Visualizes:
    - **XY plane** (4-direction view, ball path in top view)
    - **Z vs time** (height of the ball over time)

---

## Hardware Setup

### Components

- ESP32 development board (e.g. `esp32dev`, ESP32-WROOM-32U)
- ICM20948 IMU module (I²C)
- Jumper wires
- USB cable for ESP32

### Wiring (I²C)

Typical wiring (adjust pins if your board is different):

| ICM20948 | ESP32       |
|----------|------------|
| VCC      | 3.3V       |
| GND      | GND        |
| SDA      | GPIO 21    |
| SCL      | GPIO 22    |
| AD0      | GND (I2C address 0x68) |

Make sure the ICM20948 module is powered from **3.3V**, not 5V.

---

## ESP32 Firmware

This firmware:

- Initializes the ICM20948,
- Starts a Wi-Fi Access Point: `esp32_gyro` / password `12345678`,
- Hosts a simple HTTP server on port `80`,
- Serves **live sensor data** as JSON at `/data`.

```cpp
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ICM20948_WE.h>

#define I2C_ADDRESS 0x68

ICM20948_WE icm = ICM20948_WE(I2C_ADDRESS);
WebServer server(80);
bool sensor_ok = false;

void handleData() {
    xyzFloat acc = {0, 0, 0};
    xyzFloat gyro = {0, 0, 0};

    if(sensor_ok){
        icm.readSensor();              // Read all IMU data
        icm.getAccRawValues(&acc);     // Raw accelerometer
        icm.getGyrRawValues(&gyro);    // Raw gyroscope
    }

    String json = "{";
    json += ""acc_x":" + String(acc.x) + ",";
    json += ""acc_y":" + String(acc.y) + ",";
    json += ""acc_z":" + String(acc.z) + ",";
    json += ""gyro_x":" + String(gyro.x) + ",";
    json += ""gyro_y":" + String(gyro.y) + ",";
    json += ""gyro_z":" + String(gyro.z);
    json += "}";

    server.send(200, "application/json", json);
}

void setup() {
    Serial.begin(115200);
    Wire.begin(); // Start I²C

    if (icm.init()) {
        Serial.println("ICM20948 initialized!");
        sensor_ok = true;
    } else {
        Serial.println("ERROR: ICM20948 not detected. Sending zeros.");
        sensor_ok = false;
    }

    WiFi.softAP("esp32_gyro", "12345678");
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());

    server.on("/data", handleData);
    server.begin();
}

void loop() {
    server.handleClient();
}
```

### PlatformIO Example `platformio.ini`

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
lib_deps = 
    wollewald/ICM20948_WE@^1.2.6
```

---

## JSON API

### Endpoint

- **URL (when ESP32 is AP):** `http://192.168.4.1/data`
- **Method:** `GET`
- **Response Content-Type:** `application/json`

### Sample JSON

```json
{
  "acc_x": -124.0,
  "acc_y": -260.0,
  "acc_z": 15988.0,
  "gyro_x": 16.0,
  "gyro_y": 1549.0,
  "gyro_z": 180.0
}
```

### Fields

- `acc_x`, `acc_y`, `acc_z`  
  Raw accelerometer readings along the sensor’s X/Y/Z axes.  
  Typically in **LSB** (counts), not directly m/s².  
  For ICM20948:
  - At ±2g range, sensitivity is ~16384 LSB/g.

- `gyro_x`, `gyro_y`, `gyro_z`  
  Raw gyroscope readings (angular rate) around X/Y/Z axes.  
  Also in **LSB**, not directly deg/s.  
  For example, at ±2000 dps, sensitivity is ~16.4 LSB per deg/s.

GyroTrack’s Python side can work with raw values for visualization,  
but for accurate physics you’ll usually want to convert to **g / m/s²**  
and **deg/s / rad/s** based on your IMU configuration.

---

## Python Visualizer

The Python client:

- Connects to the ESP32 access point (`esp32_gyro`),
- Polls the `/data` endpoint at a fixed rate (e.g. every **20 ms → 50 Hz**),
- Computes orientation and rotation-compensated acceleration,
- Integrates to estimate position,
- Draws two live plots:
  1. **XY plane** (top view, 4-direction vector, ball starts at (0,0))
  2. **Height vs time** from the Z axis

### Requirements

- Python 3.9+
- Install dependencies:

```bash
pip install numpy matplotlib pyquaternion requests
```

### Basic Usage

1. Flash the ESP32 with the firmware above.
2. Power the ESP32 + ICM20948.
3. On your PC/laptop:
   - Connect to the Wi-Fi AP:  
     **SSID:** `esp32_gyro`  
     **Password:** `12345678`
4. Confirm that you can open this in a browser:  
   `http://192.168.4.1/data`  
   You should see JSON with `acc_x`, `acc_y`, etc.
5. Run the Python visualizer script (e.g. `gyrotrack.py`) that:
   - Calls `requests.get("http://192.168.4.1/data")`
   - Parses `acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z`
   - Updates orientation and position
   - Plots XY and Z in real time.

---

## How GyroTrack Handles Rotation

A core idea in GyroTrack is **spin compensation**.

### Problem

The IMU is mounted inside a spinning ball:

- The accelerometer measures acceleration in the **sensor frame**.
- When the ball spins, the sensor axes spin too.
- The same real-world acceleration appears with different `(acc_x, acc_y, acc_z)` over time.
- If you directly integrate these as if they were world coordinates,  
  the trajectory becomes nonsense once the ball starts spinning.

### Solution: World Frame Reconstruction

GyroTrack:

1. Reads gyroscope (`gyro_x, gyro_y, gyro_z`) each frame.
2. Integrates angular velocity to maintain a **quaternion orientation**:
   ```python
   orientation = orientation * dq  # dq from gyro * dt
   ```
3. Rotates accelerometer from sensor frame → world frame:
   ```python
   acc_world = orientation.rotate(acc_body)
   ```
4. (Optionally) subtracts gravity from the Z axis:
   ```python
   acc_world[2] -= 9.81
   ```
5. Integrates world-frame acceleration:
   ```python
   vx += acc_world[0] * dt
   vy += acc_world[1] * dt
   vz += acc_world[2] * dt

   x_pos += vx * dt
   y_pos += vy * dt
   z_pos += vz * dt
   ```
6. Uses these world-frame positions for plotting.

Result:

- The ball can spin freely.
- The visualized trajectory (XY + height Z) stays in a **fixed world frame**.
- Spin affects orientation, but **does not corrupt the path**.

---

## Limitations

- Simple gyro integration for orientation → long-term drift possible.
- Double integration of acceleration → position drift over time.
- No advanced fusion filter (Madgwick/Mahony/Kalman) yet.
- No full calibration pipeline (bias/scale corrections).

For short motions (like a thrown ball) and visualization,  
GyroTrack is still very useful as a demo / educational tool.

---

## License

[[(MIT LICENSE)](https://github.com/zahednia/GyroTrack/blob/main/LICENSE)]
