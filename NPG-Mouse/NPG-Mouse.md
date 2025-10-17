# NPG-Mouse: Neuro Playground Lite Mouse Control

This project turns the Neuro Playground Lite (NPG) into a hands-free mouse controller using a headband. It combines EEG/EOG blink detection and head movement sensing for intuitive computer control.

## What It Does
- **Head Movement → Mouse Movement:**
  - The MPU6050 sensor (gyro + accelerometer) is attached to the headband and connected to NPG via the QwiKK port.
  - Moving your head up/down or left/right moves the mouse cursor on your computer.
- **Blink Detection → Mouse Clicks:**
  - NPG reads single-channel EOG data.
  - Double blinks trigger a left mouse click.
  - Triple blinks trigger a right mouse click.

## How It Works
- **Sensors Used:**
  - **MPU6050:** Detects head tilt and orientation for cursor movement.
  - **EEG/EOG Input:** Detects blinks for mouse clicks.
- **Mouse Control:**
  - The code processes head tilt angles and translates them into smooth mouse movements.
  - Sensitivity, deadzone, and acceleration are adjustable for comfort and precision.
- **Blink Detection:**
  - The EOG signal is filtered and analyzed to detect blinks.
  - Timing logic distinguishes between double and triple blinks for different mouse clicks.
- **Calibration:**
  - The headband calibrates itself for neutral position and movement directions using vibration feedback.
- **BLE Connection:**
  - NPG acts as a Bluetooth mouse and keyboard, allowing wireless control.

## Usage
1. Attach the NPG and MPU6050 to a headband.
2. Connect the MPU6050 to NPG via the QwiKK port.
3. Wear the headband and power on NPG.
4. Calibrate by following vibration feedback.
5. Move your head to control the mouse cursor.
6. Blink twice for left click, three times for right click.

---

**Made by Upside Down Labs.**

Open-source, affordable neuroscience for everyone!
