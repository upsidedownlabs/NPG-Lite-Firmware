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

   Copyright (c) 2024 - 2025 Deepak Khatri - deepak@upsidedownlabs.tech
   Copyright (c) 2024 - 2025 Mahesh Tupe - tupemahesh91@gmail.com
   Copyright (c) 2024 - 2025 Upside Down Labs - contact@upsidedownlabs.tech

   At Upside Down Labs, we create open‐source DIY neuroscience hardware and software.
   Our mission is to make neuroscience affordable and accessible for everyone.
   By supporting us with your purchase, you help spread innovation and open science.
   Thank you for being part of this journey with us!
*/

#include <WiFi.h>
#include <WiFiManager.h>
#include <WebSocketsServer.h>
#include <Adafruit_NeoPixel.h>
#include <ESPmDNS.h>

#define TRIGGER_PIN 9
#define PIXEL_BRIGHTNESS 7
#define TIMER_FREQ 1000000
#define MOTOR_PIN 7
#define PIXEL_PIN 3
#define LED_PIN 6
// Websockets connection on port 81
WebSocketsServer webSocket = WebSocketsServer(81);
// Onboard neopixel at PIXEL_PIN
Adafruit_NeoPixel pixels(4, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Queue for storing ADC data
static int dataQueueLen = 4000; // Queue length for ADC data
static QueueHandle_t dataQueue; // Queue

hw_timer_t *timer_1 = NULL;
portMUX_TYPE timermux_1 = portMUX_INITIALIZER_UNLOCKED;

int sampling_rate = 500; // change this to change sampling rate in Hz
int FPS = 25;            // change this to change FPS i.e number of packets sent per second

int total_blocks = (int)(sampling_rate / FPS);
int BLOCK_SIZE = 13;
uint8_t *packetBytes = (uint8_t *)calloc(total_blocks * BLOCK_SIZE, sizeof(uint8_t));
bool wm = false; // WiFiManager portal access
bool ws_connected = false;
const char *ssid = "npg-lite-2";
const char *password = "";
bool istrigger = false; // Trigger for user to do action

uint8_t adc_pins[] = {0, 1, 2};

volatile int interruptCounter = 0;
uint8_t *blockbytes = (uint8_t *)calloc(BLOCK_SIZE - 1, sizeof(uint8_t));

void IRAM_ATTR DRDY_ISR()
{
    memset(blockbytes, 0, 12);
    if (ws_connected)
    {
        portENTER_CRITICAL_ISR(&timermux_1);
        for (int i = 0; i < sizeof(adc_pins) / sizeof(uint8_t); i++)
        {
            uint16_t res = analogRead(adc_pins[i]);
            blockbytes[2 * i] = (uint8_t)(res >> 8);
            blockbytes[2 * i + 1] = (uint8_t)(res & 0xFF);
        }
        xQueueSendFromISR(dataQueue, blockbytes, NULL);
        portEXIT_CRITICAL_ISR(&timermux_1);
    }
}

void webSocketEvent(byte num, WStype_t type, uint8_t *payload, size_t length)
{
    ESP_LOGD("ws-event", "some event happned");
    switch (type)
    {
    case WStype_DISCONNECTED:
        ESP_LOGD("ws-event", "Client %d disconnected", num);
        pixels.setPixelColor(0, pixels.Color(PIXEL_BRIGHTNESS, PIXEL_BRIGHTNESS, 0));
        pixels.show();
        ws_connected = false;
        digitalWrite(MOTOR_PIN, HIGH);
        delay(500);
        digitalWrite(MOTOR_PIN, LOW);
        delay(500);
        digitalWrite(MOTOR_PIN, HIGH);
        delay(500);
        digitalWrite(MOTOR_PIN, LOW);
        break;
    case WStype_CONNECTED: // if a client is connected, then type == WStype_CONNECTED
        ESP_LOGD("ws-event", "Client  %d connected", num);
        pixels.setPixelColor(0, pixels.Color(0, PIXEL_BRIGHTNESS, PIXEL_BRIGHTNESS));
        pixels.show();
        ws_connected = true;
        digitalWrite(MOTOR_PIN, HIGH);
        delay(500);
        digitalWrite(MOTOR_PIN, LOW);
        // optionally you can add code here what to do when connected
        break;
    case WStype_TEXT: // if a client has sent data, then type == WStype_TEXT
        break;
    case WStype_BIN:
        ESP_LOGD("ws-event", "BIN data received");
        if (payload[0] == 0x01)
        {
            ESP_LOGD("ws-event", "Trigger high");
            istrigger = true;
            digitalWrite(MOTOR_PIN, HIGH);
            delay(50);
            digitalWrite(MOTOR_PIN, LOW);
        }
        else if (payload[0] == 0x02)
        {
            ESP_LOGD("ws-event", "Trigger low");
            istrigger = false;
            digitalWrite(MOTOR_PIN, HIGH);
            delay(30);
            digitalWrite(MOTOR_PIN, LOW);
            delay(50);
            digitalWrite(MOTOR_PIN, HIGH);
            delay(30);
            digitalWrite(MOTOR_PIN, LOW);
        }
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
    default:
        break;
    }
}
void setup()
{
    Serial.begin(115200);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // Trigger pin to put device in AP mode
    pinMode(TRIGGER_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    pinMode(MOTOR_PIN, OUTPUT);
    WiFi.mode(WIFI_AP_STA);
    dataQueue = xQueueCreate(dataQueueLen, BLOCK_SIZE - 1);

    // Initiate wifimanager
    WiFiManager wifiManager;

    // Set new pixel
    pixels.begin();
    pixels.setPixelColor(0, pixels.Color(PIXEL_BRIGHTNESS, 0, 0)); // RED
    pixels.show();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    for (int i = 0; i < 10; i++)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        if (digitalRead(TRIGGER_PIN) == LOW)
        {
            wm = true;
            pixels.setPixelColor(0, pixels.Color(PIXEL_BRIGHTNESS, 0, PIXEL_BRIGHTNESS));
            pixels.show();
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // Allow to put device into AP mode
    for (int i = 0; i < 10; i++)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        if (digitalRead(TRIGGER_PIN) == LOW)
        {
            wifiManager.resetSettings();
            pixels.setPixelColor(0, pixels.Color(0, 0, PIXEL_BRIGHTNESS)); // BLUE
            pixels.show();
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    pixels.setPixelColor(0, pixels.Color(0, PIXEL_BRIGHTNESS, 0)); // Green
    pixels.show();

    // Try to connect
    if (wm and !wifiManager.autoConnect("CORTX-FM5", "sleepyeeg"))
    {
        Serial.println("failed to connect, we should reset and see if it connects");
        vTaskDelay(1 / portTICK_PERIOD_MS);
        ESP.restart();
    }
    else
    {
        if (!WiFi.softAP(ssid, password))
        {
            ESP_LOGE("APMode", "APModeStatus: %s", "Soft AP creation failed!");
            while (1)
            {
                vTaskDelay(1 / portTICK_PERIOD_MS);
                pixels.setPixelColor(0, pixels.Color(PIXEL_BRIGHTNESS, 0, 0)); // RED
                pixels.show();
                ESP.restart();
            }
        }
        else
        {
            ESP_LOGE("APMode", "APModeStatus: %s", "Soft AP creation success!");
            pixels.setPixelColor(0, pixels.Color(0, PIXEL_BRIGHTNESS, PIXEL_BRIGHTNESS)); // cyan
            pixels.show();
        }
    }

    // Initiate MDNS
    if (!MDNS.begin("multi-emg"))
    {
        Serial.println("Error setting up MDNS responder!");
        while (1)
        {
            vTaskDelay(1 / portTICK_PERIOD_MS);
            pixels.setPixelColor(0, pixels.Color(PIXEL_BRIGHTNESS, 0, 0)); // RED
            pixels.show();
            ESP.restart();
        }
    }
    else
    {
        Serial.println("mDNS responder started");
        pixels.setPixelColor(0, pixels.Color(0, PIXEL_BRIGHTNESS, PIXEL_BRIGHTNESS)); // cyan
        pixels.show();
    }

    // Create weboscket connection
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    MDNS.addService("http", "tcp", 80);

    vTaskDelay(100 / portTICK_PERIOD_MS);

    timer_1 = timerBegin(1000000);
    timerAttachInterrupt(timer_1, &DRDY_ISR);
    timerAlarm(timer_1, (int)(TIMER_FREQ / sampling_rate), true, 0);
    analogReadResolution(12);
    memset(packetBytes, 0, total_blocks * BLOCK_SIZE);
}

uint8_t counter = 0;
void loop()
{
    if (ws_connected)
    {
        uint8_t *packetBytes = (uint8_t *)calloc(total_blocks * BLOCK_SIZE, sizeof(uint8_t)); // Single packet of length PACKET_SIZE in Bytes
        memset(packetBytes, 0, total_blocks * BLOCK_SIZE);
        for (int block = 0; block < total_blocks; block++)
        {
            // Get data from queue
            uint8_t blockBytes[BLOCK_SIZE] = {0};
            blockBytes[0] = counter;
            while (xQueueReceive(dataQueue, (void *)&blockBytes[1], 0) != pdTRUE)
                vTaskDelay(1 / portTICK_PERIOD_MS);
            
            blockBytes[BLOCK_SIZE - 1] = istrigger ? 0xFF : 0x00;
            blockBytes[BLOCK_SIZE - 2] = istrigger ? 0xFF : 0x00;
            unsigned int block_offset = block * BLOCK_SIZE;
            memcpy((uint8_t *)&packetBytes[block_offset], (void *)&blockBytes, BLOCK_SIZE);
            counter = counter < 255 ? ++counter : 0;
        }
        webSocket.sendBIN(0, packetBytes, total_blocks * BLOCK_SIZE);
        free(packetBytes);
    }
    webSocket.loop();
    vTaskDelay(1 / portTICK_PERIOD_MS);
}