// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

// Copyright (c) 2025 Aman Maheshwari - Aman@upsidedownlabs.tech
// Copyright (c) 2025 Krishnanshu Mittal - krishnanshu@upsidedownlabs.tech
// Copyright (c) 2025 Deepak Khatri - deepak@upsidedownlabs.tech
// Copyright (c) 2025 Upside Down Labs - contact@upsidedownlabs.tech

// At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
// Our mission is to make neuroscience affordable and accessible for everyone.
// By supporting us with your purchase, you help spread innovation and open science.
// Thank you for being part of this journey with us!

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "esp_dsp.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// ----------------- USER CONFIGURATION -----------------
#define SAMPLE_RATE       512           // samples per second
#define BAUD_RATE         115200
#define INPUT_PIN         A0
#define PIXEL_PIN         15
#define PIXEL_COUNT       6
#define BLINK_SERVICE_UUID        "6910123a-eb0d-4c35-9a60-bebe1dcb549d"
#define BLINK_CHAR_UUID           "5f4f1107-7fc1-43b2-a540-0aa1a9f1ce78"

BLEServer*           pBleServer     = nullptr;
BLEService*          pBlinkService  = nullptr;
BLECharacteristic*   pBlinkChar     = nullptr;

// Onboard Neopixel at PIXEL_PIN
Adafruit_NeoPixel pixels(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

// ----------------- USER CONFIGURATION -----------------
// Add this after existing defines:
#define ENVELOPE_WINDOW_MS 100  // Smoothing window in milliseconds
#define ENVELOPE_WINDOW_SIZE ((ENVELOPE_WINDOW_MS * SAMPLE_RATE) / 1000)

const unsigned long BLINK_DEBOUNCE_MS   = 250;   // minimal spacing between individual blinks
const unsigned long DOUBLE_BLINK_MS     = 800;   // max time between the two blinks
unsigned long lastBlinkTime     = 0;             // time of most recent blink
unsigned long firstBlinkTime    = 0;             // time of the first blink in a pair
int         blinkCount         = 0;             // how many valid blinks so far (0–2)
bool        menu           = LOW;            // current LED state
const unsigned long TRIPLE_BLINK_MS = 1000;   // max time between all three blinks
unsigned long tripleBlinkStartTime = 0;      // time of the first blink in a triple
int tripleBlinkCount = 0;                    // how many valid blinks in triple sequence

int menuIndex = 0;
const unsigned long MENU_TIMEOUT_MS = 20000;  // 20 seconds
unsigned long menuStartTime = 0;
static bool clientConnected = false;
unsigned long secondBlinkTime   = 0;
unsigned long triple_blink_ms   = 600; 

bool betaEventFired = false;
float BlinkThreshold = 50.0;

// ----------------- CALIBRATION VARIABLES -----------------
enum ProgramState { STATE_CALIBRATING, STATE_RUNNING };
ProgramState programState = STATE_CALIBRATING;
const unsigned long CALIBRATION_DURATION_MS = 10000; // 10 seconds
unsigned long calibrationStartTime = 0;
float maxBetaPct = 0.0;
float betaThreshold = 0.0; // Will be set to 60% of maxBetaPct
const float THRESHOLD_MULTIPLIER = 0.6;
unsigned long lastBlinkUpdate = 0;
unsigned long lastChaseUpdate = 0;
int chasePosition = 0;  // Current position in LED chase

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    clientConnected = true;
    Serial.println("BLE client connected");
  }
  void onDisconnect(BLEServer* pServer) override {
    clientConnected = false;
    Serial.println("BLE client disconnected");
    // restart advertising so another client can connect
    pServer->getAdvertising()->start();
  }
};

// ----------------- BUFFERS & TYPES -----------------
// Add these after existing buffers:
float envelopeBuffer[ENVELOPE_WINDOW_SIZE] = {0};
int envelopeIndex = 0;
float envelopeSum = 0;
float currentEEGEnvelope = 0;

typedef struct {
  float delta, theta, alpha, beta, gamma, total;
} BandpowerResults;

BandpowerResults smoothedPowers = {0};

// High-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 512.0 Hz, frequency: 5.0 Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
float highpass(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - -1.91327599*z1 - 0.91688335*z2;
    output = 0.95753983*x + -1.91507967*z1 + 0.95753983*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

// --- Filter Functions ---
// Band-Stop Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 512.0 Hz, frequency: [48.0, 52.0] Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
float Notch(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - -1.58696045*z1 - 0.96505858*z2;
    output = 0.96588529*x + -1.57986211*z1 + 0.96588529*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.62761184*z1 - 0.96671306*z2;
    output = 1.00000000*x + -1.63566226*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

// Low-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 512.0 Hz, frequency: 45.0 Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
float EEGFilter(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - -1.24200128*z1 - 0.45885207*z2;
    output = 0.05421270*x + 0.10842539*z1 + 0.05421270*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

// ----------------- ENVELOPE FUNCTION -----------------
float updateEEGEnvelope(float sample) {
  float absSample = fabs(sample);  // Rectify EEG signal

  // Update circular buffer and running sum
  envelopeSum -= envelopeBuffer[envelopeIndex];
  envelopeSum += absSample;
  envelopeBuffer[envelopeIndex] = absSample;
  envelopeIndex = (envelopeIndex + 1) % ENVELOPE_WINDOW_SIZE;

  return envelopeSum / ENVELOPE_WINDOW_SIZE;  // Return moving average
}


void showPixels()
{
  for(int i=0; i<PIXEL_COUNT; i++)
  {
    pixels.setPixelColor(i, pixels.Color(5, 5, 5)); // Level 1 to 6
  }
  pixels.show();
}

// ----------------- SETUP & LOOP -----------------
void setup() {
  Serial.begin(BAUD_RATE);
  delay(100);
  pinMode(INPUT_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(300);
  digitalWrite(LED_BUILTIN, LOW);
  pixels.begin();
  pixels.clear();
  pixels.show();

  // Start calibration
  Serial.println("Starting calibration...");
  programState = STATE_CALIBRATING;
  calibrationStartTime = millis();
  lastBlinkUpdate = millis();

  // --- BLE init ---
  BLEDevice::init("ESP32C6_EEG");                    // device name
  pBleServer   = BLEDevice::createServer();
  pBleServer->setCallbacks(new MyServerCallbacks());
  pBlinkService = pBleServer->createService(BLINK_SERVICE_UUID);

  // create a NOTIFY-only characteristic:
  pBlinkChar = pBlinkService->createCharacteristic(
                  BLINK_CHAR_UUID,
                  BLECharacteristic::PROPERTY_NOTIFY
               );
  // ensure clients can subscribe:
  pBlinkChar->addDescriptor(new BLE2902());

  pBlinkService->start();

  // start advertising so a phone/PC can connect:
  BLEAdvertising* pAdvertising = pBleServer->getAdvertising();
  pAdvertising->start();
  Serial.println(">> BLE Advertising started");
}

void loop() {
  static uint16_t idx = 0;
  static unsigned long lastMicros = micros();
  unsigned long now = micros(), dt = now - lastMicros;
  lastMicros = now;

  static long timer = 0;
  timer -= dt;
  if(timer <= 0){
    timer += 1000000L / SAMPLE_RATE;
    int raw = analogRead(INPUT_PIN);
    float filt = EEGFilter(Notch(raw));
    float filtered = highpass(filt);
    currentEEGEnvelope = updateEEGEnvelope(filtered);
    
  }
  // Normal running state processing
  unsigned long nowMs = millis();

// 1) Did we cross threshold and respect per‑blink debounce?
  if (currentEEGEnvelope > BlinkThreshold && (nowMs - lastBlinkTime) >= BLINK_DEBOUNCE_MS) {
    lastBlinkTime = nowMs;    // mark this blink

    // 2) Count it
    if (blinkCount == 0) {
      // first blink of the pair
      firstBlinkTime = nowMs;
      blinkCount = 1;
      // Serial.println("First blink detected");
    }
    else if (blinkCount == 1 && (nowMs - firstBlinkTime) <= DOUBLE_BLINK_MS) {
      // double blink detected 
      secondBlinkTime = nowMs;
      blinkCount = 2;
    }
    else if (blinkCount==2 && (nowMs - secondBlinkTime) <= triple_blink_ms)
    {
          pixels.setPixelColor(menuIndex-1, pixels.Color(0,10,0));
    pixels.show();

    uint8_t cmd[2] = { (uint8_t)'A', (uint8_t)menuIndex };
    if (clientConnected) {
      pBlinkChar->setValue(cmd, 2);
      pBlinkChar->notify();
    }

      Serial.println("Triple blink detected!");
      blinkCount=0;
    }
    else {
      // either too late or extra blink → restart sequence
      firstBlinkTime = nowMs;
      blinkCount = 1;
      
    }
  }

    // if we were in “2 blinks” but no third arrived in time → treat as a real double
    if (blinkCount == 2 && (nowMs - secondBlinkTime) > triple_blink_ms) {
          if(!menu) {
      menu = !menu;
      menuStartTime = millis();      // start our 10s countdown
      Serial.println(0);
      if (clientConnected) {
        uint8_t cmd = 0;
        pBlinkChar->setValue(&cmd, 1);
        pBlinkChar->notify();
      }
      showPixels();
      menuIndex = 0;
      blinkCount = 0;         // reset for next pair
    } else {
      betaEventFired = false;
      menuStartTime = millis();      // restart the 10 second countdown
      if(menuIndex==6) {
        menuIndex=1;
      } else {
        menuIndex++;
      }
      Serial.println(menuIndex);
      uint8_t cmd[2];
      
      cmd[0] = (uint8_t)('S');
      cmd[1] = menuIndex;

      if (clientConnected) {
        pBlinkChar->setValue(cmd, 2); // send exactly 2 bytes
        pBlinkChar->notify();
      }

      showPixels();
      pixels.setPixelColor(menuIndex-1, pixels.Color(0, 0 , 10));
      pixels.show();
      blinkCount = 0;
    }
      Serial.println("Double blink detected");

      blinkCount = 0;
    }

  // 3) Timeout: if we never got the second blink in time, reset
  if (blinkCount == 1 && (nowMs - firstBlinkTime) > DOUBLE_BLINK_MS) {
    blinkCount = 0;
  }


  unsigned long resetTimer = millis();
  if (menu && (resetTimer - menuStartTime > MENU_TIMEOUT_MS)) {
    // 20 seconds elapsed with no double-blink
    menu = false;
    Serial.println("Menu timed out → off");
    // optionally send a notify to tell the client:
    if (clientConnected) {
      uint8_t cmd = 127;         // your “menu off” code
      pBlinkChar->setValue(&cmd, 1);
      pBlinkChar->notify();
    }
    // you might also clear pixels:
    pixels.clear();
    pixels.show();
  }
}
