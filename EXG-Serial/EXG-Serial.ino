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

// Copyright (c) 2025 Krishnanshu Mittal - krishnanshu@upsidedownlabs.tech
// Copyright (c) 2025 Aman Maheshwari - aman@upsidedownlabs.tech 
// Copyright (c) 2025 Deepak Khatri - deepak@upsidedownlabs.tech
// Copyright (c) 2025 Upside Down Labs - contact@upsidedownlabs.tech

// At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
// Our mission is to make neuroscience affordable and accessible for everyone.
// By supporting us with your purchase, you help spread innovation and open science.
// Thank you for being part of this journey with us!
/*
  FFT routines based on Espressif’s ESP-DSP examples:

    • Initialization (dsps_fft2r_init_fc32) from:
      https://github.com/espressif/esp-dsp/tree/master/examples/basic_math
      (examples/basic_math/main/dsps_math_main.c)

    • Two-real FFT processing
      (dsps_fft2r_fc32, dsps_bit_rev_fc32, dsps_cplx2reC_fc32)
      from: https://github.com/espressif/esp-dsp/tree/master/examples/fft
      (examples/fft/main/dsps_fft_main.c)
*/

// Simple EEG/EOG/EMG demo: detects focus (EEG), blinks (EOG), jaw clench (EMG)

#include <Arduino.h>
#include "esp_dsp.h"

#define DEBUG  // Uncomment this line to enable debugging

// ----------------- USER CONFIGURATION -----------------
#define SAMPLE_RATE       512          // samples per second
#define FFT_SIZE          512          // must be a power of two
#define BAUD_RATE         115200
#define INPUT_PIN         A0
#define LED_PIN 7

// Envelope Configuration
#define ENVELOPE_WINDOW_MS 100  // Smoothing window in milliseconds
#define ENVELOPE_WINDOW_SIZE ((ENVELOPE_WINDOW_MS * SAMPLE_RATE) / 1000)

#define SEGMENT_SEC 1
#define SAMPLES_PER_SEGMENT (SAMPLE_RATE * SEGMENT_SEC)

float eegBuffer[SAMPLES_PER_SEGMENT] = {0};
float eogBuffer[SAMPLES_PER_SEGMENT] = {0};
float emgBuffer[SAMPLES_PER_SEGMENT] = {0};
uint16_t segmentIndex = 0;
unsigned long lastSegmentTimeMs = 0;

float eegAvg = 0, eogAvg = 0, emgAvg = 0;
float eegMin = 0, eegMax = 0;
float eogMin = 0, eogMax = 0;
float emgMin = 0, emgMax = 0;
bool segmentStatsReady = false;


// Focus Detection Configuration
const unsigned long  FOCUS_DEBOUNCE_MS   = 2000; // ignore focus triggers for this many ms after a valid focus detection
const float          BETA_THRESHOLD  = 10.0; // adjust based on calibration: typically 50% of max beta power during focused state
unsigned long        lastFocusTime = 0; // when focus was last accepted (debounce)

// Blink Detection Configuration
const unsigned long BLINK_DEBOUNCE_MS   = 200;   // minimal spacing between individual blinks
const unsigned long DOUBLE_BLINK_MS     = 600;   // max time between the two blinks
unsigned long lastBlinkTime     = 0;             // time of most recent blink
unsigned long firstBlinkTime    = 0;             // time of the first blink in a pair
unsigned long secondBlinkTime   = 0;
unsigned long triple_blink_ms   = 600; 
int         blinkCount         = 0;             // how many valid blinks so far (0–2)
float currentEOGEnvelope = 0;
float BlinkLowerThreshold = 50.0;
float BlinkUpperThreshold = 70.0;

// Jaw Clench Detection Configuration
float currentEMGEnvelope = 0;
// --- Jaw clench debounce config ---
const unsigned long  JAW_DEBOUNCE_MS   = 500;   // ignore new clench triggers for this many ms after a valid clench
const float          JAW_ON_THRESHOLD  = 120.0; // same as your current threshold
const float          JAW_OFF_THRESHOLD = 100.0; // hysteresis: must fall below this to re-arm
const unsigned long JAW_BLOCK_DURATION_MS = 500;  // Block other detections for 500ms after jaw clench
unsigned long lastJawDetectionTime = 0;            // Time when jaw clench was last detected

unsigned long lastJawClenchTime = 0; // last time a clench was accepted
bool jawState = false;              // true = currently considered in a clench


// Envelope Processing Variables
float envelopeBuffer[ENVELOPE_WINDOW_SIZE] = {0};
int envelopeIndex = 0;
float envelopeSum = 0;

// EEG bands (Hz)
#define DELTA_LOW    0.5f
#define DELTA_HIGH   4.0f
#define THETA_LOW    4.0f
#define THETA_HIGH   8.0f
#define ALPHA_LOW    8.0f
#define ALPHA_HIGH   13.0f
#define BETA_LOW     13.0f
#define BETA_HIGH    30.0f
#define GAMMA_LOW    30.0f
#define GAMMA_HIGH   45.0f

#define SMOOTHING_FACTOR 0.63f
#define EPS              1e-7f

// ----------------- BUFFERS & TYPES -----------------
float inputBuffer[FFT_SIZE];
float powerSpectrum[FFT_SIZE/2];
float BetaPower = 0; // last computed beta-band power % (updated in processFFT())

// For two-real FFT trick
__attribute__((aligned(16))) float y_cf[FFT_SIZE * 2];
float *y1_cf = &y_cf[0];

typedef struct {
  float delta, theta, alpha, beta, gamma, total;
} BandpowerResults;

BandpowerResults smoothedPowers = {0};

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

// High-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 512.0 Hz, frequency: 5.0 Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
float EOGFilter(float input)
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

// High-Pass Butterworth IIR digital filter
// Sampling rate: 512.0 Hz, frequency: 70.0 Hz
// Filter is order 2, implemented as second-order sections (biquads)
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
float EMGFilter(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - -0.85080258*z1 - 0.30256882*z2;
    output = 0.53834285*x + -1.07668570*z1 + 0.53834285*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

// updateEnvelope(sample): sliding-window moving-average of absolute(sample)
float updateEnvelope(float sample) 
{
  float absSample = fabs(sample); 

  // Update circular buffer and running sum
  envelopeSum -= envelopeBuffer[envelopeIndex];
  envelopeSum += absSample;
  envelopeBuffer[envelopeIndex] = absSample;
  envelopeIndex = (envelopeIndex + 1) % ENVELOPE_WINDOW_SIZE;

  return envelopeSum / ENVELOPE_WINDOW_SIZE;  // Return moving average
}

// ----------------- BANDPOWER & SMOOTHING -----------------
BandpowerResults calculateBandpower(float *ps, float binRes, int halfSize) {
  BandpowerResults r = {0};
  for(int i=1; i<halfSize; i++){
    float freq = i * binRes;
    float p    = ps[i];
    r.total   += p;
         if(freq>=DELTA_LOW && freq<DELTA_HIGH)  r.delta += p;
    else if(freq>=THETA_LOW && freq<THETA_HIGH)  r.theta += p;
    else if(freq>=ALPHA_LOW && freq<ALPHA_HIGH)  r.alpha += p;
    else if(freq>=BETA_LOW  && freq<BETA_HIGH)   r.beta  += p;
    else if(freq>=GAMMA_LOW && freq<GAMMA_HIGH)  r.gamma += p;
  }
  return r;
}

void smoothBandpower(const BandpowerResults *raw, BandpowerResults *s) {
  s->delta = SMOOTHING_FACTOR*raw->delta + (1-SMOOTHING_FACTOR)*s->delta;
  s->theta = SMOOTHING_FACTOR*raw->theta + (1-SMOOTHING_FACTOR)*s->theta;
  s->alpha = SMOOTHING_FACTOR*raw->alpha + (1-SMOOTHING_FACTOR)*s->alpha;
  s->beta  = SMOOTHING_FACTOR*raw->beta  + (1-SMOOTHING_FACTOR)*s->beta;
  s->gamma = SMOOTHING_FACTOR*raw->gamma + (1-SMOOTHING_FACTOR)*s->gamma;
  s->total = SMOOTHING_FACTOR*raw->total + (1-SMOOTHING_FACTOR)*s->total;
}

// ----------------- DSP FFT SETUP -----------------
void initFFT() {
  // initialize esp-dsp real-FFT (two-real trick)
  esp_err_t err = dsps_fft2r_init_fc32(NULL, FFT_SIZE);
  if(err != ESP_OK){
    Serial.println("FFT init failed");
    while(1) delay(10);
  }
}

// ----------------- FFT + BANDPOWER + PEAK -----------------
void processFFT() {
  // pack real→complex: real=inputBuffer, imag=0
  for(int i=0; i<FFT_SIZE; i++){
    y_cf[2*i]   = inputBuffer[i];
    y_cf[2*i+1] = 0.0f;
  }

  // FFT
  dsps_fft2r_fc32(y_cf, FFT_SIZE);
  dsps_bit_rev_fc32(y_cf, FFT_SIZE);
  dsps_cplx2reC_fc32(y_cf, FFT_SIZE);

  // magnitude² spectrum
  int half = FFT_SIZE/2;
  for(int i=0; i<half; i++){
    float re = y1_cf[2*i];
    float im = y1_cf[2*i+1];
    powerSpectrum[i] = re*re + im*im;
  }

  // detect peak bin (skip i=0)
  int maxIdx = 1;
  float maxP = powerSpectrum[1];
  for(int i=2; i<half; i++){
    if(powerSpectrum[i] > maxP){
      maxP = powerSpectrum[i];
      maxIdx = i;
    }
  }
  float binRes = float(SAMPLE_RATE)/FFT_SIZE;
  float peakHz = maxIdx * binRes;

  // bandpower & smoothing
  BandpowerResults raw = calculateBandpower(powerSpectrum, binRes, half);
  smoothBandpower(&raw, &smoothedPowers);
  BetaPower = (smoothedPowers.beta/ (smoothedPowers.total + EPS))*100;

  // Serial.print("EEG: ");
  // Serial.print(BetaPower);

}

// ----------------- SETUP & LOOP -----------------
void setup() {
  Serial.begin(BAUD_RATE);

  pinMode(INPUT_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(300);
  digitalWrite(LED_PIN, LOW);
  initFFT();

  lastSegmentTimeMs = millis();  // Initialize the timer
}

void loop() {
  digitalWrite(LED_PIN, LOW);

  static uint16_t idx = 0;
  static unsigned long lastMicros = micros();
  unsigned long now = micros(), dt = now - lastMicros;
  lastMicros = now;

  // Declare timing variables
  static unsigned long timeTaken = 0;
  static unsigned long startTime = 0;
  static bool timingActive = false;

  static long timer = 0;
  timer -= dt;
  if(timer <= 0){
    timer += 1000000L / SAMPLE_RATE;

    // Start timing here - capture time before data acquisition
    startTime = micros();
    timingActive = true;

    int raw = analogRead(INPUT_PIN);
    float EEG = EEGFilter(Notch(raw));
    float EOG = EOGFilter(Notch(raw));
    float EMG = EMGFilter(Notch(raw));
    currentEOGEnvelope = updateEnvelope(EOG);
    currentEMGEnvelope = updateEnvelope(EMG);
    inputBuffer[idx++] = EEG;

    if(segmentIndex < SAMPLES_PER_SEGMENT) {
    eegBuffer[segmentIndex] = BetaPower;
    eogBuffer[segmentIndex] = currentEOGEnvelope;
    emgBuffer[segmentIndex] = currentEMGEnvelope;
    segmentIndex++;
    }

    // Calculate elapsed time after all processing is done
    timeTaken = micros() - startTime;
  }

  unsigned long nowMs = millis();

  if ((nowMs - lastSegmentTimeMs) >= (1000UL * SEGMENT_SEC)) {
    // Only process if we have data
    if(segmentIndex > 0) {
        // Compute min/max for last segment
        eegMin = eegBuffer[0];  
        eegMax = eegBuffer[0];    
        eogMin = eogBuffer[0]; 
        eogMax = eogBuffer[0];  
        emgMin = emgBuffer[0];  
        emgMax = emgBuffer[0];  
        float eegSum = 0, eogSum = 0, emgSum = 0;
        for (uint16_t i = 0; i < segmentIndex; i++) {
            float eegVal = eegBuffer[i];
            float eogVal = eogBuffer[i];
            float emgVal = emgBuffer[i];

            if (eegVal < eegMin) eegMin = eegVal;
            if (eegVal > eegMax) eegMax = eegVal;
            eegSum += eegVal;

            if (eogVal < eogMin) eogMin = eogVal;
            if (eogVal > eogMax) eogMax = eogVal;
            eogSum += eogVal;

            if (emgVal < emgMin) emgMin = emgVal;
            if (emgVal > emgMax) emgMax = emgVal;
            emgSum += emgVal;
        }
        eegAvg = eegSum / segmentIndex;
        eogAvg = eogSum / segmentIndex;
        emgAvg = emgSum / segmentIndex;

        // Print ONLY when stats update
        #ifdef DEBUG
        Serial.print("Latency: ");
        Serial.print(timeTaken);
        Serial.print(" μs | EEG: ");
        Serial.print(BetaPower);
        Serial.print(" (Average: "); Serial.print(eegAvg);
        Serial.print(", Min: "); Serial.print(eegMin);
        Serial.print(", Max: "); Serial.print(eegMax); Serial.print(")");
        Serial.print(" | EOG: (Average: "); Serial.print(eogAvg);
        Serial.print(", Min: "); Serial.print(eogMin);
        Serial.print(", Max: "); Serial.print(eogMax); Serial.print(")");
        Serial.print(" | EMG: (Average: "); Serial.print(emgAvg);
        Serial.print(", Min: "); Serial.print(emgMin);
        Serial.print(", Max: "); Serial.print(emgMax); Serial.print(")");
        Serial.println();
        #endif
        segmentStatsReady = true;
    }
    lastSegmentTimeMs = nowMs;
    segmentIndex = 0;
  }

  // Check if we're in the blocking period after jaw clench
  bool jawBlockActive = (nowMs - lastJawDetectionTime) < JAW_BLOCK_DURATION_MS;

  if(!jawBlockActive && BetaPower > BETA_THRESHOLD && (nowMs - lastFocusTime) >= FOCUS_DEBOUNCE_MS)
  {
    lastFocusTime = nowMs;
    Serial.println("Focussed");
    digitalWrite(LED_PIN, HIGH);
  }

  // Debounced jaw-clench detection
  if (!jawState) {
    // not currently clenching — look for rising edge + debounce
    if (currentEMGEnvelope > JAW_ON_THRESHOLD && (nowMs - lastJawClenchTime) >= JAW_DEBOUNCE_MS) {
      jawState = true;
      lastJawClenchTime = nowMs;
      lastJawDetectionTime = nowMs;  // Start blocking period
      Serial.println("Jaw Clench");
    }
  } else {
    // currently in clench state — wait for signal to fall below OFF threshold before re-arming
    if (currentEMGEnvelope < JAW_OFF_THRESHOLD) {
      // mark time when cleared so the next rising edge respects debounce
      lastJawClenchTime = nowMs;
      jawState = false;
    }
  }

  // 1) Did we cross threshold and respect per‑blink debounce?
  if (!jawBlockActive && currentEOGEnvelope > BlinkLowerThreshold && currentEOGEnvelope < BlinkUpperThreshold && (nowMs - lastBlinkTime) >= BLINK_DEBOUNCE_MS) {
    lastBlinkTime = nowMs;    // mark this blink

    // 2) Count it
    if (blinkCount == 0) {
      // first blink of the pair
      firstBlinkTime = nowMs;
      blinkCount = 1;
      // Serial.println("First blink detected");
    }
    else if (blinkCount == 1 && (nowMs - firstBlinkTime) <= DOUBLE_BLINK_MS) {
      // double blink detected - send right arrow key
      secondBlinkTime = nowMs;
      blinkCount = 2;
      // Serial.println("Second blink registered, waiting for triple…");
    }
    else if (blinkCount==2 && (nowMs - secondBlinkTime) <= triple_blink_ms)
    {
      Serial.println("Triple blink detected!");
      blinkCount=0;
    }
    else {
      // either too late or extra blink → restart sequence
      firstBlinkTime = nowMs;
      blinkCount = 1;
      // Serial.println("Blink sequence restarted");
    }
  }

    // if we were in “2 blinks” but no third arrived in time → treat as a real double
    if (blinkCount == 2 && (nowMs - secondBlinkTime) > triple_blink_ms) {
      Serial.println("Double blink detected");
      blinkCount = 0;
    }

  // 3) Timeout: if we never got the second blink in time, reset
  if (blinkCount == 1 && (nowMs - firstBlinkTime) > DOUBLE_BLINK_MS) {
    blinkCount = 0;
  }

  if(idx >= FFT_SIZE){
    processFFT();
    // Calculate total time after FFT processing is complete
    if(timingActive) {
      timeTaken = micros() - startTime;
      timingActive = false;
    }
    idx = 0;
  }
}
