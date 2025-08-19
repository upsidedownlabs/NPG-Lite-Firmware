# Neuro Playground Lite: Single-Channel EEG/EOG/EMG Demo

## Introduction

This program demonstrates how to use a single-channel biosignal acquisition system to detect three types of neural and muscular activity from front cortex EEG recording setup:

- **EEG (Electroencephalography):** Detects focus (beta thresholding from EEG FFT data).
- **EOG (Electrooculography):** Detects double/triple eye blinks (EOG artifact in the EEG signal).
- **EMG (Electromyography):** Detects jaw clench (EMG artifact in the EEG signal).

The output is displayed on the Serial Monitor, showing events such as `Focussed`, `Double blink`, `Triple blink`, and `Jaw Clench`. User will have to put the program into `DEBUG` mode to calibrate the system and adjust the threshold values.

---

## How It Works

1. **Signal Acquisition:**
   - The raw signal is read from the EEG electrode placement where `positive (A0P)` is connected to forhead, `negative (A0N)` is connected behind any ear and `BIAS` is connected behind the other ear.

2. **Filtering:**
   - A notch filter removes powerline noise.
   - Dedicated filters extract EEG, EOG, and EMG features from the signal.

3. **Feature Extraction:**
   - **EEG:**
     - The raw EEG signal is filtered using a 0.5Hz - 45Hz bandpass filter.
     - The filtered signal is then processed using FFT to calculate brain bandpowers.
     - The beta waves (13Hz - 30Hz) bandpower is monitored to detect focus.
   - **EOG:**
     - The raw EOG signal is filtered using a 5Hz - 45Hz bandpass filter.
     - A running average (envelope algorithm) is applied to the filtered EOG signal for smoothing.
     - The EOG envelope is monitored to detect double/triple eye blinks.
   - **EMG:**
     - A running average (envelope algorithm) is applied to the filtered EOG and EMG signals for smoothing.

4. **Event Detection:**
   - Thresholds are set for each feature to detect specific events:
     - **Focus:** When beta bandpower exceeds the set threshold.
     - **Blink:** When EOG envelope crosses defined thresholds.
     - **Jaw Clench:** When EMG envelope exceeds the set threshold.

---

## Debugging & Customization

Since biosignals vary from person to person, the program is designed to be customizable:

### 1. Enable Debug Mode
- Uncomment the line `#define DEBUG`, you can find it under the macros at the top of the code.
- Once the program is compiled and flashed to the NPG Lite it will enable debugging mode.
- The Serial Monitor will now display:
  - Time taken for each calculation
  - Beta power values
  - EOG envelope value
  - EMG envelope

### 2. Signal Analysis & Threshold Tuning

> **Safety Note:**
> - Do not connect your laptop to a charger while testing.
> - Maintain distance from AC appliances to reduce noise.


#### **EEG (Focus Detection):**
- Focus at one point for 5–10 seconds without blinking or moving your eyes.
- On the Serial Monitor, observe the maximum beta power value reached (check the `Max` value for EEG).
- Set `BETA_THRESHOLD` to **50% of the max value noted**.

#### **EOG (Blink Detection):**
- Try blinking and observe the maximum EOG value reached (check the `Max` value for EOG, typically between 30 and 90).
- Set `BlinkLowerThreshold` to **70% of the max value**.
- Set `BlinkUpperThreshold` to **80% of the max value**.

#### **EMG (Jaw Clench Detection):**
- Clench your jaw and observe the maximum EMG value reached (check the `Max` value for EMG, typically between 100 and 300).
- Set `JAW_ON_THRESHOLD` to **80% of the max value**.
- Set `JAW_OFF_THRESHOLD` to **70% of the max value**.

### 3. Real-Time Monitoring
- You can ask a friend to help monitor the values while you test.
- Adjust thresholds as needed for reliable detection.

---

## Summary

This program provides a simple, customizable way to explore EEG, EOG, and EMG detection using a single channel. With real-time feedback and easy threshold adjustment, it is ideal for experimentation and learning in neuroscience playgrounds.

---

**Contact & Credits:**
- Developed by Upside Down Labs
- For support and open-source contributions, visit [Upside Down Labs](https://upsidedownlabs.tech)
