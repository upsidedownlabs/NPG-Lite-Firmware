/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <https://www.gnu.org/licenses/>.

   BLE connectivity adapted from the ESP32 BLE Server example by Random Nerd Tutorials:
   https://randomnerdtutorials.com/esp32-bluetooth-low-energy-ble-arduino-ide/.

   Copyright (c) 2024 - 2025 Krishnanshu Mittal - karan4g79@gmail.com
   Copyright (c) 2024 - 2025 Deepak Khatri - deepak@upsidedownlabs.tech
   Copyright (c) 2024 - 2025 Upside Down Labs - contact@upsidedownlabs.tech

   At Upside Down Labs, we create open‐source DIY neuroscience hardware and software.
   Our mission is to make neuroscience affordable and accessible for everyone.
   By supporting us with your purchase, you help spread innovation and open science.
   Thank you for being part of this journey with us!
*/

// ----- Existing Includes -----
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <BLEUtils.h>
#include <Adafruit_NeoPixel.h>
#include "esp_timer.h"
#include <sdkconfig.h>
#include "hal/efuse_hal.h"

// ----- Chip-specific Pin Definitions -----
//
// Use the ESP-IDF config macros to detect the chip.
#if defined(CONFIG_IDF_TARGET_ESP32C6)
// Store chip revision number
uint32_t chiprev = efuse_hal_chip_revision();
#define LED_BUILTIN 7
#define PIXEL_PIN 15
#define PIXEL_COUNT 6
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
#define LED_BUILTIN 6
#define PIXEL_PIN 3
#define PIXEL_COUNT 4
#else
#error "Unsupported board: Please target either ESP32-C6 or ESP32-C3 in your Board Manager."
#endif

#define PIXEL_BRIGHTNESS 7                               // Brightness of Neopixel LED
#define NUM_CHANNELS 3                                   // Number of ADC channels
#define SINGLE_SAMPLE_LEN 7                              // Each sample: 1 counter + (3 channels * 2 bytes)
#define BLOCK_COUNT 10                                   // Batch size: 10 samples per notification
#define NEW_PACKET_LEN (BLOCK_COUNT * SINGLE_SAMPLE_LEN) // New packet length (70 bytes)
#define SAMP_RATE 500.0                                  // Sampling rate (500 Hz)

// Onboard Neopixel at PIXEL_PIN
Adafruit_NeoPixel pixels(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

// BLE UUIDs – change if desired.
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define DATA_CHAR_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"    // For ADC data (Notify only)
#define CONTROL_CHAR_UUID "0000ff01-0000-1000-8000-00805f9b34fb" // For commands (Read/Write/Notify)

// ----- Global Variables -----
uint8_t batchBuffer[NEW_PACKET_LEN] = {0}; // Buffer to accumulate BLOCK_COUNT samples
uint8_t samplePacket[SINGLE_SAMPLE_LEN] = {0};
volatile int sampleIndex = 0;      // How many samples accumulated in current batch
volatile bool streaming = false;   // True when "START" command is received
volatile bool bufferReady = false; // Flag set by timer callback

esp_timer_handle_t adcTimer; // Handle for esp_timer
BLECharacteristic *pDataCharacteristic;
BLECharacteristic *pControlCharacteristic;

// Global sample counter (each sample's packet counter)
uint8_t overallCounter = 0;

// ----- BLE Server Callbacks -----
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer) override
  {
    pixels.setPixelColor(0, pixels.Color(0, PIXEL_BRIGHTNESS, 0)); // Green
    pixels.show();
    digitalWrite(LED_BUILTIN, HIGH);
    delay(400);
    digitalWrite(LED_BUILTIN, LOW);
    // Serial.println("BLE client connected");
  }
  void onDisconnect(BLEServer *pServer) override
  {
    pixels.setPixelColor(0, pixels.Color(PIXEL_BRIGHTNESS, 0, 0)); // Red
    pixels.show();
    digitalWrite(LED_BUILTIN, HIGH);
    delay(400);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(400);
    digitalWrite(LED_BUILTIN, LOW);
    // Serial.println("BLE client disconnected");
    streaming = false;
    BLEDevice::startAdvertising();
  }
};

// ----- BLE Control Characteristic Callback -----
// Handles incoming commands ("START", "STOP", "WHORU", "STATUS")
class ControlCallback : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *characteristic) override
  {
    String cmd = characteristic->getValue();
    cmd.trim();
    cmd.toUpperCase();
    if (cmd == "START")
    {
      pixels.setPixelColor(0, pixels.Color(0, 0, PIXEL_BRIGHTNESS)); // Blue
      pixels.show();
      // Reset counters and start streaming
      overallCounter = 0;
      sampleIndex = 0;
      streaming = true;
      // Serial.println("Received START command");
    }
    else if (cmd == "STOP")
    {
      pixels.setPixelColor(0, pixels.Color(0, PIXEL_BRIGHTNESS, 0)); // Green
      pixels.show();
      streaming = false;
      // Serial.println("Received STOP command");
    }
    else if (cmd == "WHORU")
    {
      characteristic->setValue("NPG-LITE");
      characteristic->notify();
      // Serial.println("Received WHORU command");
    }
    else if (cmd == "STATUS")
    {
      characteristic->setValue(streaming ? "RUNNING" : "STOPPED");
      characteristic->notify();
      // Serial.println("Received STATUS command");
    }
    else
    {
      characteristic->setValue("UNKNOWN COMMAND");
      characteristic->notify();
      // Serial.println("Received unknown command");
    }
  }
};

// ----- Timer Callback -----
// This callback is executed every (1e6 / SAMP_RATE) microseconds (i.e. every 2000 µs for 500 Hz)
void IRAM_ATTR adcTimerCallback(void *arg)
{
  if (streaming)
  {
    bufferReady = true;
  }
}

void setup()
{
  // ----- Initialize Neopixel LED -----
  pixels.begin();
  // Set the Neopixel to red (indicating device turned on)
  pixels.setPixelColor(0, pixels.Color(PIXEL_BRIGHTNESS, 0, 0));
  pixels.show();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Setup packet header is done per sample in the loop.
  // Set ADC resolution (12-bit)
  analogReadResolution(12);

  // ----- Initialize BLE -----
  BLEDevice::init("NPG");

  // Retrieve the BLE MAC address
  String bleMAC = BLEDevice::getAddress().toString();

  // Set device name
  String deviceName = "NPG-" + bleMAC;
  esp_ble_gap_set_device_name(deviceName.c_str());

  // Optionally, request a larger MTU:
  BLEDevice::setMTU(111);

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create Data Characteristic (Notify only) for ADC data
  pDataCharacteristic = pService->createCharacteristic(
      DATA_CHAR_UUID,
      BLECharacteristic::PROPERTY_NOTIFY);
  pDataCharacteristic->addDescriptor(new BLE2902());

  // Create Control Characteristic (Read/Write/Notify) for command handling
  pControlCharacteristic = pService->createCharacteristic(
      CONTROL_CHAR_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  pControlCharacteristic->setCallbacks(new ControlCallback());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->start();
  // Serial.println("BLE Advertising started");

  // Create and start periodic timer using esp_timer API
  const esp_timer_create_args_t timerArgs = {
      .callback = &adcTimerCallback,
      .arg = NULL,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "adc_timer"};
  esp_timer_create(&timerArgs, &adcTimer);
  esp_timer_start_periodic(adcTimer, 1000000 / SAMP_RATE);
}

void loop()
{
  // When streaming is enabled and the timer flag is set...
  if (streaming && bufferReady)
  {
    // Create one sample packet (7 bytes)
    memset(samplePacket, 0, SINGLE_SAMPLE_LEN); // Clear buffer before use
    samplePacket[0] = overallCounter;
    overallCounter = (overallCounter + 1) % 256;

    // Read each ADC channel (channels 0, 1, 2) and store as two bytes (big-endian)
    for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++)
    {
      uint16_t adcValue;

#if defined(CONFIG_IDF_TARGET_ESP32C6)
      if (chiprev == 1)
        adcValue = map(analogRead(ch), 0, 3249, 0, 4095); // Scale to 12-bit range
      else
        adcValue = analogRead(ch);
#else
      // Version 0.2 or other chips can use direct reading
      adcValue = analogRead(ch);
#endif

      samplePacket[1 + ch * 2] = highByte(adcValue);
      samplePacket[1 + ch * 2 + 1] = lowByte(adcValue);
    }

    // Append this samplePacket to the batch buffer
    memcpy(&batchBuffer[sampleIndex * SINGLE_SAMPLE_LEN], samplePacket, SINGLE_SAMPLE_LEN);
    sampleIndex++;
    bufferReady = false;

    // Once we've collected BLOCK_COUNT samples, send them as one BLE notification.
    if (sampleIndex >= BLOCK_COUNT)
    {
      pDataCharacteristic->setValue(batchBuffer, NEW_PACKET_LEN);
      pDataCharacteristic->notify();
      sampleIndex = 0;
    }
  }
  yield();
}