# NPG Lite - Firmware

Neuro PlayGround (NPG) Lite firmware repository.

[Support NPG-Lite project by signing-up with your email on CrowdSupply](https://www.crowdsupply.com/upside-down-labs/neuro-playground-lite)

| Example | Code |
| ------- | ---- |
| Default program to show Visual, Audiotory, and Haptic feedback on NPG| [NPG-Default.ino](NPG-Default/NPG-Default.ino) |
| Calculate FFT & Band Power of single channel EEG and print on Serial | [Serial-FFT.ino](Serial-FFT/Serial-FFT.ino) |
| Bluetooh Low Energy (BLE) server to notify client with real-time NPG data | [BLE-Server.ino](BLE-Server/BLE-Server.ino) |
| BLE client to take notification from server and trigger GPIO | [BLE-Client.ino](BLE-Client/BLE-Client.ino) |
| BLE server to nofity client based on EEG band (beta) power triggers | [BLE-BCI-Server-Toggle.ino](BLE-BCI-Server-Toggle/BLE-BCI-Server-Toggle.ino) |
| InfraRed (IR) reciever code to identify LG AC remote button commands | [IR-LG-Receive.ino](IR-LG-Receive/IR-LG-Receive.ino) |
| IR signal send example code to control LG AC to toggle ON/OFF using user button | [IR-LG-Send.ino](IR-LG-Send/IR-LG-Send.ino) |
| Brain Computer Interface (BCI) to toggle LG AC ON/OFF using EEG band (beta) power | [BCI-IR-Send.ino](BCI-IR-Send/BCI-IR-Send.ino) |
| BLE client that receives notifications from the server and triggers GPIO to control the car | [BLE-BCI-Car.ino](BLE-BCI-Car/BLE-BCI-Car.ino) |
| BCI remote (server) to drive the BLE car using EEG band (beta) power and EMG (envelope) data | [BLE-BCI-Car-Remote.ino](BLE-BCI-Car-Remote/BLE-BCI-Car-Remote.ino) |
| Brain Computer Interface example sketch for Double blink and focus detection. | [BCI-Blink-Serial.ino](BCI-Blink-Serial/BCI-Blink-Serial.ino) |
| Brain Computer Interface to control a menu of options using Double blink and focus detection for ALS patients. | [BCI-Blink-BLE.ino](BCI-Blink-BLE/BCI-Blink-BLE.ino) |
| MPU6050 sketch to stream 3-axis accelerometer data and send 4 keystrokes to play video games on laptop. | [Gyro-Motion-Detection.ino](Gyro-Motion-Detection/Gyro-Motion-Detection.ino) |
| Detects double and triple blinks from EOG signals using high‑pass and notch IIR filters with envelope detection. | [Blinky-Keys-Serial.ino](Blinky-Keys-Serial/Blinky-Keys-Serial.ino) |
| Implements a BLE HID keyboard that sends right‑arrow on double blinks and left‑arrow on triple blinks to control slides in a presentation.| [Blinky-Keys-BLE.ino](Blinky-Keys-BLE/Blinky-Keys-BLE.ino) |
| Implements a BLE gamepad that reads EEG and EMG signals to control games on Windows, using focus and muscle contractions.| [BCI-BLE-Gamepad.ino](BCI-BLE-Gamepad/BCI-BLE-Gamepad.ino) |


