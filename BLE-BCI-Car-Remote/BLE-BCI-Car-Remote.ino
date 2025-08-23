// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

// Copyright (c) 2025 Aman Maheshwari - Aman@upsidedownlabs.tech
// Copyright (c) 2024 - 2025 Krishnanshu Mittal - krishnanshu@upsidedownlabs.tech
// Copyright (c) 2024 - 2025 Deepak Khatri - deepak@upsidedownlabs.tech
// Copyright (c) 2024 - 2025 Upside Down Labs - contact@upsidedownlabs.tech

// At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
// Our mission is to make neuroscience affordable and accessible for everyone.
// By supporting us with your purchase, you help spread innovation and open science.
// Thank you for being part of this journey with us!

/*
 This example is adapted from the client and server code provided by MoThunderz
 Firmware: https://github.com/mo-thunderz/Esp32BlePart2
 YouTube video: https://www.youtube.com/watch?v=s3yoZa6kzus
*/

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Arduino.h>
#include "esp_dsp.h"
#include <vector>
// constants won't change. They're used here to set pin numbers:
const int buttonPin = 9;  // the number of the pushbutton pin
const int ledPin = 7;     // the number of the LED pin

// Variables will change:
int ledState = HIGH;        // the current state of the output pin
uint32_t buttonState;       // the current reading from the input pin
int lastButtonState = LOW;  // the previous reading from the input pin
uint32_t bci_val = 0;       // EEG-based control value (0=stop, 3=forward)
uint32_t emg1_val1 = 0;     // Left EMG control value (0=inactive, 1=left turn)
uint32_t emg2_val2 = 0;     // Right EMG control value (0=inactive, 2=right turn)
uint32_t bootback_val = 4;  // Button toggle value (4=toggle state)

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

// Initialize all pointers
BLEServer *pServer = NULL;                    // Pointer to the server
BLECharacteristic *pCharacteristic_1 = NULL;  // Pointer to Characteristic 1
BLECharacteristic *pCharacteristic_2 = NULL;  // Pointer to Characteristic 2
BLEDescriptor *pDescr_1;                      // Pointer to Descriptor of Characteristic 1
BLE2902 *pBLE2902_1;                          // Pointer to BLE2902 of Characteristic 1
BLE2902 *pBLE2902_2;                          // Pointer to BLE2902 of Characteristic 2

// Some variables to keep track on device connected
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Variable that will continuously be increased and written to the client
uint32_t value = 0;
uint32_t betaThreshold = 2;
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
// UUIDs used in this example:
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_1 "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_2 "1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e"

// Callback function that is called whenever a client is connected or disconnected
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }
};

// ----------------- USER CONFIGURATION -----------------
#define SAMPLE_RATE 512  // samples per second
#define FFT_SIZE 512     // must be a power of two
#define BAUD_RATE 115200
#define INPUT_PIN1 A0  // EEG input pin
#define INPUT_PIN2 A1  // Left hand EMG input pin
#define INPUT_PIN3 A2  // Right hand EMG input pin

// EEG bands (Hz)
#define DELTA_LOW 0.5f
#define DELTA_HIGH 4.0f
#define THETA_LOW 4.0f
#define THETA_HIGH 8.0f
#define ALPHA_LOW 8.0f
#define ALPHA_HIGH 13.0f
#define BETA_LOW 13.0f
#define BETA_HIGH 30.0f
#define GAMMA_LOW 30.0f
#define GAMMA_HIGH 45.0f

#define SMOOTHING_FACTOR 0.63f
#define EPS 1e-7f

// ----------------- BUFFERS & TYPES -----------------
float inputBuffer[FFT_SIZE];
float powerSpectrum[FFT_SIZE / 2];

// For two-real FFT trick
__attribute__((aligned(16))) float y_cf[FFT_SIZE * 2];
float *y1_cf = &y_cf[0];

typedef struct
{
  float delta, theta, alpha, beta, gamma, total;
} BandpowerResults;

BandpowerResults smoothedPowers = { 0, 0, 0, 0, 0, 0 };

// ----------------- NOTCH FILTER CLASSES -----------------
// For 50Hz AC noise removal
// Band-Stop Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 500.0 Hz, frequency: [48.0, 52.0] Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
class NotchFilter {
private:
  struct BiquadState {
    float z1 = 0;
    float z2 = 0;
  };

  BiquadState state1;
  BiquadState state2;

public:
  float process(float input) {
    float output = input;

    // First biquad section
    float x = output - (-1.56858163f * state1.z1) - (0.96424138f * state1.z2);
    output = 0.96508099f * x + (-1.56202714f * state1.z1) + (0.96508099f * state1.z2);
    state1.z2 = state1.z1;
    state1.z1 = x;

    // Second biquad section
    x = output - (-1.61100358f * state2.z1) - (0.96592171f * state2.z2);
    output = 1.00000000f * x + (-1.61854514f * state2.z1) + (1.00000000f * state2.z2);
    state2.z2 = state2.z1;
    state2.z1 = x;

    return output;
  }

  void reset() {
    state1.z1 = state1.z2 = 0;
    state2.z1 = state2.z2 = 0;
  }
};

// ----------------- EMG FILTER CLASSES -----------------
// High-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 500.0 Hz, frequency: 70.0 Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
class EMGHighPassFilter {
private:
  // Filter state for a single channel
  double z1 = 0.0;
  double z2 = 0.0;

public:
  // Process a single sample
  double process(double input) {
    const double x = input - -0.82523238 * z1 - 0.29463653 * z2;
    const double output = 0.52996723 * x + -1.05993445 * z1 + 0.52996723 * z2;

    // Update state
    z2 = z1;
    z1 = x;

    return output;
  }

  // Reset filter state
  void reset() {
    z1 = 0.0;
    z2 = 0.0;
  }
};

// Class to calculate EMG Envelope
class EnvelopeFilter {
private:
  std::vector<double> circularBuffer;
  double sum = 0.0;
  int dataIndex = 0;
  const int bufferSize;

public:
  EnvelopeFilter(int bufferSize)
    : bufferSize(bufferSize) {
    circularBuffer.resize(bufferSize, 0.0);
  }

  double getEnvelope(double absEmg) {
    sum -= circularBuffer[dataIndex];
    sum += absEmg;
    circularBuffer[dataIndex] = absEmg;
    dataIndex = (dataIndex + 1) % bufferSize;
    return (sum / bufferSize);
  }
};

// NEW: Low-pass filter for EEG signals
float EEGFilter(float input) {
  float output = input;
  {
    static float z1 = 0, z2 = 0;
    float x = output - -1.22465158 * z1 - 0.45044543 * z2;
    output = 0.05644846 * x + 0.11289692 * z1 + 0.05644846 * z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}
NotchFilter filters[3];              // Notch filters for all 3 input channels
EMGHighPassFilter emgfilters[2];     // High-pass filters for EMG channels
EnvelopeFilter Envelopefilter1(16);  // Envelope detector for left EMG
EnvelopeFilter Envelopefilter2(16);  // Envelope detector for right EMG

// ----------------- BANDPOWER & SMOOTHING -----------------
BandpowerResults calculateBandpower(float *ps, float binRes, int halfSize) {
  BandpowerResults r = { 0, 0, 0, 0, 0, 0 };
  for (int i = 1; i < halfSize; i++) {
    float freq = i * binRes;
    float p = ps[i];
    r.total += p;
    if (freq >= DELTA_LOW && freq < DELTA_HIGH)
      r.delta += p;
    else if (freq >= THETA_LOW && freq < THETA_HIGH)
      r.theta += p;
    else if (freq >= ALPHA_LOW && freq < ALPHA_HIGH)
      r.alpha += p;
    else if (freq >= BETA_LOW && freq < BETA_HIGH)
      r.beta += p;
    else if (freq >= GAMMA_LOW && freq < GAMMA_HIGH)
      r.gamma += p;
  }
  return r;
}

void smoothBandpower(const BandpowerResults *raw, BandpowerResults *s) {
  s->delta = SMOOTHING_FACTOR * raw->delta + (1 - SMOOTHING_FACTOR) * s->delta;
  s->theta = SMOOTHING_FACTOR * raw->theta + (1 - SMOOTHING_FACTOR) * s->theta;
  s->alpha = SMOOTHING_FACTOR * raw->alpha + (1 - SMOOTHING_FACTOR) * s->alpha;
  s->beta = SMOOTHING_FACTOR * raw->beta + (1 - SMOOTHING_FACTOR) * s->beta;
  s->gamma = SMOOTHING_FACTOR * raw->gamma + (1 - SMOOTHING_FACTOR) * s->gamma;
  s->total = SMOOTHING_FACTOR * raw->total + (1 - SMOOTHING_FACTOR) * s->total;
}

// ----------------- DSP FFT SETUP -----------------
void initFFT() {
  // initialize esp-dsp real-FFT (two-real trick)
  esp_err_t err = dsps_fft2r_init_fc32(NULL, FFT_SIZE);
  if (err != ESP_OK) {
    Serial.println("FFT init failed");
    while (1)
      delay(10);
  }
}

// ----------------- FFT + BANDPOWER + PEAK -----------------
void processFFT() {
  // pack real→complex: real=inputBuffer, imag=0
  for (int i = 0; i < FFT_SIZE; i++) {
    y_cf[2 * i] = inputBuffer[i];
    y_cf[2 * i + 1] = 0.0f;
  }

  // FFT
  dsps_fft2r_fc32(y_cf, FFT_SIZE);
  dsps_bit_rev_fc32(y_cf, FFT_SIZE);
  dsps_cplx2reC_fc32(y_cf, FFT_SIZE);

  // magnitude² spectrum
  int half = FFT_SIZE / 2;
  for (int i = 0; i < half; i++) {
    float re = y1_cf[2 * i];
    float im = y1_cf[2 * i + 1];
    powerSpectrum[i] = re * re + im * im;
  }

  // detect peak bin (skip i=0)
  int maxIdx = 1;
  float maxP = powerSpectrum[1];
  for (int i = 2; i < half; i++) {
    if (powerSpectrum[i] > maxP) {
      maxP = powerSpectrum[i];
      maxIdx = i;
    }
  }
  float binRes = float(SAMPLE_RATE) / FFT_SIZE;
  float peakHz = maxIdx * binRes;

  // bandpower & smoothing
  BandpowerResults raw = calculateBandpower(powerSpectrum, binRes, half);
  smoothBandpower(&raw, &smoothedPowers);
  float T = smoothedPowers.total + EPS;
  Serial.println(((smoothedPowers.beta / T) * 100));  // for debugging purpose only

  // If the power exceeds the threshold (set as 2% of the total power), the threshold value can be adjusted based on your beta parameters.
  if (((smoothedPowers.beta / T) * 100) > betaThreshold) {
    bci_val = 3;
    Serial.println("send 3");
    pCharacteristic_1->setValue(bci_val);  // for forward moving car
    pCharacteristic_1->notify();
    digitalWrite(7, HIGH);  // Visual feedback
  } else {
    bci_val = 0;
    pCharacteristic_1->setValue(bci_val);
    pCharacteristic_1->notify();
    digitalWrite(7, LOW);
  }
}

void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial) {
    delay(1);
  }
  pinMode(INPUT_PIN1, INPUT);
  pinMode(INPUT_PIN2, INPUT);
  pinMode(INPUT_PIN3, INPUT);

  pinMode(7, OUTPUT);

  initFFT();
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);

  // set initial LED state
  digitalWrite(ledPin, ledState);
  // Create the BLE Device
  BLEDevice::init("ESP32");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic_1 = pService->createCharacteristic(
    CHARACTERISTIC_UUID_1,
    BLECharacteristic::PROPERTY_NOTIFY);

  pCharacteristic_2 = pService->createCharacteristic(
    CHARACTERISTIC_UUID_2,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

  // Create a BLE Descriptor
  pDescr_1 = new BLEDescriptor((uint16_t)0x2901);
  pDescr_1->setValue("A very interesting variable");
  pCharacteristic_1->addDescriptor(pDescr_1);

  // Add the BLE2902 Descriptor because we are using "PROPERTY_NOTIFY"
  pBLE2902_1 = new BLE2902();
  pBLE2902_1->setNotifications(true);
  pCharacteristic_1->addDescriptor(pBLE2902_1);

  pBLE2902_2 = new BLE2902();
  pBLE2902_2->setNotifications(true);
  pCharacteristic_2->addDescriptor(pBLE2902_2);

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {
  static uint16_t idx = 0;
  static unsigned long lastMicros = micros();
  unsigned long now = micros(), dt = now - lastMicros;
  lastMicros = now;

  static long timer = 0;
  timer -= dt;
  if (timer <= 0) {
    timer += 1000000L / SAMPLE_RATE;
    int raw1 = analogRead(INPUT_PIN1);
    int raw2 = analogRead(INPUT_PIN2);
    int raw3 = analogRead(INPUT_PIN3);

    float filteeg = EEGFilter(filters[0].process(raw1));
    float filtemg1 = emgfilters[0].process(filters[1].process(raw2));
    float filtemg2 = emgfilters[1].process(filters[2].process(raw3));
    inputBuffer[idx++] = filteeg;

    float env1 = Envelopefilter1.getEnvelope(abs(filtemg1));
    float env2 = Envelopefilter2.getEnvelope(abs(filtemg2));

    // If `env1` exceeds 150, trigger left turn command (send value 1).the threshold value can be adjusted based on your emg parameters.
    if (env1 > 150 && env2 > 150) {
      bootback_val = 4;
      Serial.println("sent 4 - BOTH EMG ACTIVE");  // for debugging purpose only
      pCharacteristic_1->setValue(bootback_val);   // send value 4
      pCharacteristic_1->notify();
    } else if (env1 > 150) {
      emg1_val1 = 2;
      Serial.println("sent 1");                // for debugging purpose only
      pCharacteristic_1->setValue(emg1_val1);  // for left turn car
      pCharacteristic_1->notify();
    }
    // If `env2` exceeds 150, trigger right turn command (send value 2).the threshold value can be adjusted based on your emg parameters.
    else if (env2 > 150) {
      emg2_val2 = 1;
      Serial.println("sent 2");                // for debugging purpose only
      pCharacteristic_1->setValue(emg2_val2);  // for right turn car
      pCharacteristic_1->notify();
    }
  }

  if (idx >= FFT_SIZE) {
    processFFT();
    idx = 0;
  }

  // read the state of the switch into a local variable:
  int reading = digitalRead(buttonPin);

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // if the reading is different from the current stable state
    if (reading != buttonState) {
      buttonState = reading;

      // Button is pressed (LOW because of INPUT_PULLUP)
      if (buttonState == LOW) {
        ledState = HIGH;  // Turn LED on when button pressed
        bootback_val = 4;
        Serial.println("Button PRESSED - sending continuous 4");
      }
      // Button is released
      else {
        ledState = LOW;    // Turn LED off when button released
        bootback_val = 0;  // Optional: send 0 when released
        Serial.println("Button RELEASED - stopping 4");

        if (deviceConnected) {
          pCharacteristic_1->setValue(bootback_val);
          pCharacteristic_1->notify();
        }
      }
    }

    // CONTINUOUSLY send 4 while button is pressed
    if (buttonState == LOW && deviceConnected) {
      pCharacteristic_1->setValue(bootback_val);
      pCharacteristic_1->notify();
      Serial.println("Sending: 4");
      delay(100);  // Add small delay to avoid flooding BLE
    }
  }

  // Set the LED State
  digitalWrite(ledPin, ledState);

  // Save the reading for next time
  lastButtonState = reading;

  // The code below keeps the connection status up-to-date:
  // Disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);                   // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // Connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
  // Save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;
}