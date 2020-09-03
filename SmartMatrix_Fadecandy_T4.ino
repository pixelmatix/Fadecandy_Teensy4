/*
 * This example shows how to display bitmaps that are exported from GIMP and
 * compiled into the sketch and stored in the Teensy's Flash memory
 * See more details here:
 * http://docs.pixelmatix.com/SmartMatrix/library.html#smartmatrix-library-how-to-use-the-library-drawing-raw-bitmaps
 *
 * This example uses only the SmartMatrix Background layer
 */

// uncomment one line to select your MatrixHardware configuration - configuration header needs to be included before <SmartMatrix3.h>
//#include <MatrixHardware_ESP32_V0.h>    // This file contains multiple ESP32 hardware configurations, edit the file to define GPIOPINOUT (or add #define GPIOPINOUT with a hardcoded number before this #include)
//#include <MatrixHardware_KitV1.h>       // SmartMatrix Shield for Teensy 3 V1-V3
//#include <MatrixHardware_KitV4.h>       // SmartLED Shield for Teensy 3 V4
//#include <MatrixHardware_KitV4T4.h>     // Teensy 4 Wired to SmartLED Shield for Teensy 3 V4
#include <MatrixHardware_T4Adapter.h>   // Teensy 4 Adapter attached to SmartLED Shield for Teensy 3 V4
//#include "MatrixHardware_Custom.h"      // Copy an existing MatrixHardware file to your Sketch directory, rename, customize, and you can include it like this
#include <SmartMatrix3.h>

#include "gimpbitmap.h"

// colorwheel.c has been modified to use the gimp32x32bitmap struct
#include "colorwheel.c"

#define COLOR_DEPTH 24                  // known working: 24, 48 - If the sketch uses type `rgb24` directly, COLOR_DEPTH must be 24
const uint16_t kMatrixWidth = 128;        // known working: 32, 64, 96, 128, 256
const uint16_t kMatrixHeight = 64;       // known working: 16, 32, 48, 64, 128
const uint8_t kRefreshDepth = 36;       // known working: 24, 36, 48 (on Teensy 4.x: 3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48)
const uint8_t kDmaBufferRows = 4;       // known working: 2-4, use 2 to save memory, more to keep from dropping frames and automatically lowering refresh rate
const uint8_t kPanelType = SMARTMATRIX_HUB75_64ROW_MOD32SCAN;   // use SMARTMATRIX_HUB75_16ROW_MOD8SCAN for common 16x32 panels
const uint8_t kMatrixOptions = (SMARTMATRIX_OPTIONS_NONE);      // see http://docs.pixelmatix.com/SmartMatrix for options
const uint8_t kBackgroundLayerOptions = (SM_BACKGROUND_OPTIONS_NONE);

SMARTMATRIX_ALLOCATE_BUFFERS(matrix, kMatrixWidth, kMatrixHeight, kRefreshDepth, kDmaBufferRows, kPanelType, kMatrixOptions);
SMARTMATRIX_ALLOCATE_BACKGROUND_LAYER(backgroundLayer, kMatrixWidth, kMatrixHeight, COLOR_DEPTH, kBackgroundLayerOptions);

void drawBitmap(int16_t x, int16_t y, const gimp32x32bitmap* bitmap) {
  for(unsigned int i=0; i < bitmap->height; i++) {
    for(unsigned int j=0; j < bitmap->width; j++) {
      SM_RGB pixel = { bitmap->pixel_data[(i*bitmap->width + j)*3 + 0],
                      bitmap->pixel_data[(i*bitmap->width + j)*3 + 1],
                      bitmap->pixel_data[(i*bitmap->width + j)*3 + 2] };
      if(COLOR_DEPTH == 48) {
          pixel.red = pixel.red << 8;
          pixel.green = pixel.green << 8;
          pixel.blue = pixel.blue << 8;
      }

      backgroundLayer.drawPixel(x + j, y + i, pixel);
    }
  }
}

void setup() {
  matrix.addLayer(&backgroundLayer); 
  matrix.begin();

  matrix.setBrightness(128);

  int x, y;
  backgroundLayer.fillScreen({0,0,0});
  x = (kMatrixWidth / 2) - (colorwheel.width/2);
  y = (kMatrixHeight / 2) - (colorwheel.height/2);
  // can pass &colorwheel in directly as the bitmap source is already gimp32x32bitmap
  drawBitmap(x,y,&colorwheel);
  backgroundLayer.swapBuffers();
}

void loop() {

}
