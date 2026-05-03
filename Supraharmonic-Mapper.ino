#include <arm_math.h>     // Subsystem B: Math Engine
#include <SPI.h>          // Subsystem C: Hardware Communication
#include <ILI9341_t3.h>   // Subsystem C: Optimized Screen Driver

// --- DISPLAY PINS ---
#define TFT_CS   10
#define TFT_DC   9
#define TFT_RST  8

// Initialize the display object
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC, TFT_RST);

// --- SYSTEM CONSTANTS ---
const uint16_t FFT_SIZE = 1024;         
const float SAMPLE_RATE = 350000.0;     
const float MAGNITUDE_SCALE = 8.0;      // Multiplier to make the graph fit the screen height

// --- DATA BUFFERS ---
float32_t adcBuffer[FFT_SIZE];          
float32_t windowBuffer[FFT_SIZE];       
float32_t fftOutput[FFT_SIZE];          
float32_t fftMagnitudes[FFT_SIZE / 2];  

// Display buffer to eliminate screen flickering
int previousBarHeights[320];            

arm_rfft_fast_instance_f32 fft_inst;    

void setup() {
  Serial.begin(115200);

  // Initialize Subsystem B (FFT)
  arm_rfft_fast_init_f32(&fft_inst, FFT_SIZE);
  for (int i = 0; i < FFT_SIZE; i++) {
    windowBuffer[i] = 0.5 * (1.0 - cos(2.0 * PI * i / (FFT_SIZE - 1)));
  }

  // Initialize Subsystem C (Screen)
  tft.begin();
  tft.setRotation(3); // Set to Landscape mode (320 pixels wide x 240 pixels tall)
  tft.fillScreen(ILI9341_BLACK);

  // Ensure the previous height buffer is zeroed out
  for (int i = 0; i < 320; i++) {
    previousBarHeights[i] = 0;
  }
}

void loop() {
  // ---------------------------------------------------------
  // SUBSYSTEM A: Get the Virtual Data
  // ---------------------------------------------------------
  generateVirtualADCData();

  // ---------------------------------------------------------
  // SUBSYSTEM B: Digital Signal Processing (FFT Engine)
  // ---------------------------------------------------------
  arm_mult_f32(adcBuffer, windowBuffer, adcBuffer, FFT_SIZE);
  arm_rfft_fast_f32(&fft_inst, adcBuffer, fftOutput, 0);
  arm_cmplx_mag_f32(fftOutput, fftMagnitudes, FFT_SIZE / 2);

  fftMagnitudes[0] = 0; 
  fftMagnitudes[1] = 0; 

  // --- PEAK DETECTION ---
  float maxMagnitude = 0.0;
  int peakBinIndex = 0;

  for (int i = 2; i < (FFT_SIZE / 2); i++) {
    if (fftMagnitudes[i] > maxMagnitude) {
      maxMagnitude = fftMagnitudes[i];
      peakBinIndex = i;
    }
  }

  float frequencyResolution = SAMPLE_RATE / FFT_SIZE;
  float peakFrequencyHz = peakBinIndex * frequencyResolution;

  // Print to PC for immediate testing
  Serial.print("Peak Freq: ");
  Serial.print(peakFrequencyHz);
  Serial.println(" Hz");


  // ---------------------------------------------------------
  // SUBSYSTEM C: TFT LCD Screen Rendering
  // ---------------------------------------------------------
  
  // 1. Draw the top UI Bar (Dark Blue background, White text)
  tft.fillRect(0, 0, 320, 30, ILI9341_NAVY); 
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(5, 7);
  tft.print("Peak: ");
  tft.print(peakFrequencyHz / 1000.0, 1); // Convert Hz to kHz with 1 decimal
  tft.print(" kHz");

  // 2. Draw the Live FFT Graph (Green Bars)
  int baselineY = 239; // Bottom edge of the screen

  for(int x = 0; x < 320; x++) {
    int bin = x + 2; // Map screen pixel 'x' directly to FFT bin index (offset by 2 to skip 60Hz)
    
    if (bin >= (FFT_SIZE / 2)) break; // Safety catch

    // Scale the math magnitude to physical screen pixels
    int barHeight = (int)(fftMagnitudes[bin] * MAGNITUDE_SCALE);
    
    // Cap the height so the graph doesn't draw over our top UI text bar!
    if (barHeight > 200) barHeight = 200; 
    if (barHeight < 0) barHeight = 0;

    // --- ANTI-FLICKER RENDERING ENGINE ---
    // Only redraw the screen pixels if the math actually changed
    if (barHeight != previousBarHeights[x]) {
      
      // Draw a BLACK line over the old green bar to erase it instantly
      tft.drawLine(x, baselineY, x, baselineY - previousBarHeights[x], ILI9341_BLACK);
      
      // Draw a GREEN line for the new data
      tft.drawLine(x, baselineY, x, baselineY - barHeight, ILI9341_GREEN);
      
      // Save this new height for the next frame
      previousBarHeights[x] = barHeight;
    }
  }

  // Run at ~20 Frames Per Second (Extremely smooth on the eyes)
  delay(50); 
}


// --- SUBSYSTEM A FUNCTION ---
void generateVirtualADCData() {
  for (uint16_t i = 0; i < FFT_SIZE; i++) {
    float t = (float)i / SAMPLE_RATE;

    float wave_60Hz = 1.0 * sin(2.0 * PI * 60.0 * t);
    float wave_50kHz = 0.1 * sin(2.0 * PI * 50000.0 * t);
    float randomNoise = ((float)random(-100, 100) / 10000.0); 

    adcBuffer[i] = wave_60Hz + wave_50kHz + randomNoise;
  }
}
