#include <arm_math.h>     // Digital Signal Processing Library
#include <SPI.h>          // Hardware SPI Communication
#include <ILI9341_t3.h>   // Optimized Screen Driver

// --- HARDWARE PIN DEFINITIONS ---
#define TFT_CS    10      // Display Chip Select
#define TFT_DC    9       // Display Data/Command
#define TFT_RST   8       // Display Reset
#define ADC_CNV   2       // AD4001 Convert / Chip Select Pin

// Initialize the display object
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC, TFT_RST);

// Configure high-speed SPI specifically for the AD4001 (20 MHz)
SPISettings ad4001_SPI(20000000, MSBFIRST, SPI_MODE0); 

// --- SYSTEM CONSTANTS ---
const uint16_t FFT_SIZE = 1024;         
const float SAMPLE_RATE = 200000.0;             // 200 kHz exact sampling rate
const float BIN_WIDTH = SAMPLE_RATE / FFT_SIZE; // ~195.3 Hz per frequency bin

// Y-Axis Scaling: 
const float MAGNITUDE_SCALE = 3.9;      

// --- DATA BUFFERS ---
float32_t adcBuffer[FFT_SIZE];          // Raw Voltage Data
float32_t windowBuffer[FFT_SIZE];       // Hanning Window Multipliers
float32_t fftOutput[FFT_SIZE];          // Complex Math Output
float32_t fftMagnitudes[FFT_SIZE / 2];  // Final Amplitude Data

// Anti-flicker memory for the display
int previousBarHeights[325];            

// FFT Hardware Engine Instance
arm_rfft_fast_instance_f32 fft_inst;    

void setup() {
  Serial.begin(115200);

  // 1. HARDWARE WAKE-UP DELAY (Fixes the White Screen issue)
  delay(1000); 
  SPI.begin();

  // 2. CONFIGURE ADC PIN
  pinMode(ADC_CNV, OUTPUT);
  digitalWriteFast(ADC_CNV, HIGH);

  // 3. INITIALIZE FFT MATH ENGINE
  arm_rfft_fast_init_f32(&fft_inst, FFT_SIZE);
  for (int i = 0; i < FFT_SIZE; i++) {
    // Pre-calculate Hanning Windowp
    windowBuffer[i] = 0.5 * (1.0 - cos(2.0 * PI * i / (FFT_SIZE - 1)));
  }

  // 4. INITIALIZE DISPLAY
  tft.begin();
  tft.setRotation(3); // Landscape mode (320x240)
  tft.fillScreen(ILI9341_BLACK);
  
  // Top UI Bar
  tft.fillRect(0, 0, 320, 30, ILI9341_NAVY); 
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(5, 7);
  tft.print("Peak:");

  // X-Axis Labels (2 kHz to 70 kHz)
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_LIGHTGREY);
  tft.setCursor(0, 230);
  tft.print("2kHz");
  tft.setCursor(285, 230);
  tft.print("70kHz");

  // Y-Axis Labels (Voltage Limits)
  tft.setCursor(0, 35);
  tft.print("0.1V -");
  tft.setCursor(0, 132);
  tft.print("0.05V-");

  // Zero out the display memory
  for (int i = 0; i < 320; i++) {
    previousBarHeights[i] = 0;
  }
}

void loop() {
  // ---------------------------------------------------------
  // SUBSYSTEM A: Fetch Real Hardware Data from AD4001
  // ---------------------------------------------------------
  readRealADCData();

  // ---------------------------------------------------------
  // SUBSYSTEM B: Digital Signal Processing (FFT)
  // ---------------------------------------------------------
  
  // Apply Window -> Run FFT -> Calculate Magnitudes
  arm_mult_f32(adcBuffer, windowBuffer, adcBuffer, FFT_SIZE);
  arm_rfft_fast_f32(&fft_inst, adcBuffer, fftOutput, 0);
  arm_cmplx_mag_f32(fftOutput, fftMagnitudes, FFT_SIZE / 2);

  // Digital High-Pass: Wipe out DC offset, 60Hz, and 120Hz fundamental waves
  fftMagnitudes[0] = 0; 
  fftMagnitudes[1] = 0; 
  fftMagnitudes[2] = 0; 

  // --- PEAK DETECTION ALGORITHM ---
  float maxMagnitude = 0.0;
  int peakBinIndex = 0;

  for (int i = 3; i < (FFT_SIZE / 2); i++) {
    if (fftMagnitudes[i] > maxMagnitude) {
      maxMagnitude = fftMagnitudes[i];
      peakBinIndex = i;
    }
  }

  float peakFrequencyHz = peakBinIndex * BIN_WIDTH;

  // ---------------------------------------------------------
  // SUBSYSTEM C: Display Rendering
  // ---------------------------------------------------------
  
  // 1. UPDATE PEAK TEXT INSTANTLY
  tft.fillRect(70, 7, 150, 20, ILI9341_NAVY); // Erase old text cleanly
  tft.setCursor(70, 7);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setTextSize(2);
  tft.print(peakFrequencyHz / 1000.0, 1);     // Convert Hz to kHz with 1 decimal
  tft.print(" kHz");


  // 2. DRAW THE LIVE GRAPH (2kHz to 70kHz mapping)
  int baselineY = 225; // Bottom edge of graph area

  for(int x = 0; x < 320; x++) {
    
    // Map screen pixel (0-319) to frequency target (2000-70000 Hz)
    float targetFreq = map(x, 0, 319, 2000, 70000);
    
    // Find which math bin holds that frequency
    int bin = (int)(targetFreq / BIN_WIDTH);
    if (bin >= (FFT_SIZE / 2)) break; // Safety boundary

    // Calculate physical pixel height
    int barHeight = (int)(fftMagnitudes[bin] * MAGNITUDE_SCALE);
    
    // Cap height at 190 pixels to protect the top UI bar
    if (barHeight > 190) barHeight = 190; 
    if (barHeight < 0) barHeight = 0;

    // --- ANTI-FLICKER ENGINE ---
    // Only redraw the specific pixels that changed
    if (barHeight != previousBarHeights[x]) {
      
      // Draw BLACK line to erase old data
      tft.drawLine(x, baselineY, x, baselineY - previousBarHeights[x], ILI9341_BLACK);
      
      // Draw GREEN line for new data
      tft.drawLine(x, baselineY, x, baselineY - barHeight, ILI9341_GREEN);
      
      // Save state for next frame
      previousBarHeights[x] = barHeight;
    }
  }
}

// ---------------------------------------------------------
// HARDWARE SPI DRIVER FOR AD4001 16-BIT ADC
// ---------------------------------------------------------
void readRealADCData() {
  elapsedMicros timer; // Teensy's ultra-precise internal hardware timer
  
  SPI.beginTransaction(ad4001_SPI);
  
  for (uint16_t i = 0; i < FFT_SIZE; i++) {
    timer = 0; // Reset timer for this specific measurement
    
    // 1. Tell AD4001 to convert voltage to digital
    digitalWriteFast(ADC_CNV, HIGH);
    delayNanoseconds(350); // AD4001 needs ~320ns to process the measurement
    
    // 2. Read the 16-bit answer over SPI
    digitalWriteFast(ADC_CNV, LOW);
    uint16_t rawData = SPI.transfer16(0x00);
    
    // 3. Convert digital math back into Voltage (0 to 3.3V range)
    // Then subtract 1.65V to center the AC wave perfectly at 0 for the FFT
    adcBuffer[i] = ((float)rawData * (3.3 / 65535.0)) - 1.65;
    
    // 4. Wait EXACTLY until 5.0 microseconds have passed 
    // (5 microseconds = 200,000 Sample Rate)
    while (timer < 5) {
      // Loop does nothing, simply waits for perfect timing precision
    }
  }
  
  SPI.endTransaction();
}