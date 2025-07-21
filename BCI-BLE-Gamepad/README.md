# BCI BLE Gamepad - Control Games with Your Mind

**Control video games using only your brainwaves and muscle signals!**

This project transforms the Neuro Playground Lite into a wireless BLE gamepad that reads EEG (brainwave) and EMG (muscle) signals to control games like GTA V. Experience gaming like never before - no keyboard, no controller, just pure neural control.

## 🧠 How It Works

The system uses three channels of bio-signal acquisition:
- **Channel 1 (EEG)**: Reads brainwaves from your `forehead` to detect focus/concentration
- **Channel 2 (EMG)**: Reads muscle signals from your `left arm` for directional control
- **Channel 3 (EMG)**: Reads muscle signals from your `right arm` for directional control

The firmware processes these signals in real-time using FFT analysis and envelope detection, then transmits control commands via Bluetooth Low Energy to your computer.

## 🎮 Game Controls

- **Focus (Beta waves > 15%)**: Accelerate forward (Button 1/RT)
- **Flex both arms**: Switch control mode (Button 1 ↔ Button 2)
- **Left arm muscle**: Steer left
- **Right arm muscle**: Steer right
- **Focus in reverse mode**: Reverse/brake (Button 2/LT)

## 📋 Requirements

> ⚠️ **Platform Compatibility**: This project currently supports Windows 10/11 only due to XOutput software requirements and specific Bluetooth implementation.

### Hardware
- Neuro Playground Lite (NPG Lite)
- Vibz Playmate
- Gel Electrodes (6 pieces)
- BioAmp Snap Cables
- Nuprep Skin Preparation Gel
- Alcohol Swabs
- USB to Type-C cable

*All components are included in the Neuro Playground Lite kit available from [CrowdSupply](https://www.crowdsupply.com/upside-down-labs/neuro-playground-lite).*

### Software
- Arduino IDE
- ESP32-BLE-Gamepad library
- XOutput software (for controller mapping)
- Windows 10/11 with Bluetooth support

## 🔧 Setup Instructions

### 1. Skin Preparation

**Critical for clean signal acquisition:**

1. Apply Nuprep Skin Preparation Gel to electrode placement areas
2. Clean thoroughly with alcohol swabs
3. Allow skin to dry completely

**Electrode Placement:**
- **EEG**: Forehead (between FP1-FP2) + behind both ears (bony area)
- **EMG**: Palmaris longus muscle on both forearms

### 2. Hardware Connection

1. Connect BioAmp Snap Cables to NPG Lite:
   - **Channel 1**: A0P (positive) + A0N (negative) - EEG
   - **Channel 2**: A1P (positive) + A1N (negative) - Left arm EMG
   - **Channel 3**: A2P (positive) + A2N (negative) - Right arm EMG
   - **Reference**: Connect to reference terminal

2. Attach electrodes:
   - **EEG positive**: Forehead between FP1-FP2
   - **EEG negative**: Behind one ear (bony area)
   - **Reference**: Behind other ear (bony area)
   - **Left EMG**: Positive and negative on left forearm muscle
   - **Right EMG**: Positive and negative on right forearm muscle

### 3. Environment Setup

**Important positioning requirements:**
- Sit 1m away from laptop/PC
- Stay 5m away from AC appliances (microwave, tube lights, fridge)
- Ensure NPG Lite is fully charged
- Disconnect USB-C cable during signal acquisition

### 4. Firmware Installation

1. Download and install Arduino IDE
2. Install ESP32-BLE-Gamepad library:
   - Open Library Manager
   - Search for "ESP32-ble-gamepad"
   - Install the library
3. Upload the firmware:
   - Open `BCI-BLE-Gamepad.ino` in Arduino IDE
   - Select appropriate board and port
   - Click Upload
4. Disconnect USB cable after successful upload

### 5. Bluetooth Connection

1. Turn on NPG Lite using the power switch
2. Enable Bluetooth on your Windows machine
3. Search for and connect to "NPG Gamepad"
4. Verify connection in Windows Game Controllers:
   - Press `Windows + R`
   - Type `joy.cpl` and press Enter
   - Select the controller and click Properties

### 6. Signal Testing

Test your bio-signals before gaming:
- **X-axis**: Should respond when you bend your hands left/right
- **Button 1**: Should activate when you focus intensely
- **Button 2**: Should activate when you flex both arms then focus

### 7. Controller Mapping

1. Download and install XOutput software from [SourceForge](https://sourceforge.net/projects/xoutput/)
2. Open XOutput and click "Add Controller"
3. Click "Edit" to configure mappings:
   - **LX (Left Stick X)**: Configure with left/right arm movements
   - **RT (Right Trigger)**: Configure with focus (Button 1)
   - **LT (Left Trigger)**: Configure with focus after arm flex (Button 2)
4. Save configuration

### 8. Game Time!

1. Launch GTA V (or any game with controller support)
2. Start the controller in XOutput
3. Begin gaming with your mind and muscles!

## 🎯 Technical Details

### Signal Processing
- **Sample Rate**: 500 Hz
- **FFT Size**: 256 samples
- **EEG Bands**: Delta, Theta, Alpha, Beta, Gamma
- **Notch Filter**: 50 Hz noise removal
- **EMG Filter**: 70 Hz high-pass filter
- **Envelope Detection**: 16-sample moving average

### Control Thresholds
- **Focus Detection**: Beta band power > 15% (Button 1) or > 12% (Button 2)
- **EMG Activation**: Envelope > 80 for steering, > 200 for mode switching
- **Smoothing Factor**: 0.73 for stable control

### Battery Management
- Real-time battery monitoring via voltage LUT
- Battery level updates every 10 seconds
- Automatic power management

## 🎮 Supported Games

This controller works with any PC game that supports standard gamepad input:
- Grand Theft Auto V
- Forza Horizon series
- Rocket League
- And many more!

## 🔧 Troubleshooting

**Poor signal quality:**
- Ensure proper skin preparation
- Check electrode placement
- Verify cable connections
- Reduce environmental interference

**Connection issues:**
- Restart NPG Lite
- Re-pair Bluetooth connection
- Check battery level
- Verify firmware upload

**Control sensitivity:**
- Adjust focus thresholds in code
- Recalibrate in XOutput
- Practice consistent muscle activation

## 🤝 Contributing

We welcome contributions! Feel free to:
- Submit bug reports
- Suggest new features
- Improve documentation
- Add support for new games

## 📄 License

This project is licensed under the GNU General Public License v3.0. See the code header for full license details.

## 🏢 About Upside Down Labs

At Upside Down Labs, we create open-source DIY neuroscience hardware and software. Our mission is to make neuroscience affordable and accessible for everyone. By supporting us, you help spread innovation and open science.

**Contact:**
- Website: [upsidedownlabs.tech](https://upsidedownlabs.tech)
- Email: contact@upsidedownlabs.tech

*Experience the future of gaming - control with your mind! 🧠🎮*
