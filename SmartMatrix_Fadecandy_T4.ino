/*
 * Fadecandy Teensy 4 Device Firmware - Use in combination with fork of Fadecandy Server, and "type": "teensy4" device
 * 
 * Modified Fadecandy Server: https://github.com/pixelmatix/fadecandy
 *
 * SmartMatrix Library: https://github.com/pixelmatix/SmartMatrix/
 * 
 * Compile with USB Type: Dual Serial (one Serial channel is for Fadecandy data, the other for debug)
 * - If you see "error: 'SerialUSB1' was not declared in this scope", make sure you change the USB Type in Teensyduino
 *
 */

// uncomment one line to select your MatrixHardware configuration - configuration header needs to be included before <SmartMatrix.h>
//#include <MatrixHardware_Teensy3_ShieldV4.h>        // SmartLED Shield for Teensy 3 (V4)
#include <MatrixHardware_Teensy4_ShieldV5.h>        // SmartLED Shield for Teensy 4 (V5)
//#include <MatrixHardware_Teensy3_ShieldV1toV3.h>    // SmartMatrix Shield for Teensy 3 V1-V3
//#include <MatrixHardware_Teensy4_ShieldV4Adapter.h> // Teensy 4 Adapter attached to SmartLED Shield for Teensy 3 (V4)
//#include <MatrixHardware_ESP32_V0.h>                // This file contains multiple ESP32 hardware configurations, edit the file to define GPIOPINOUT (or add #define GPIOPINOUT with a hardcoded number before this #include)
//#include "MatrixHardware_Custom.h"                  // Copy an existing MatrixHardware file to your Sketch directory, rename, customize, and you can include it like this
#include <SmartMatrix.h>
#include <FastLED.h>

#include "gimpbitmap.h"

// colorwheel.c has been modified to use the gimp32x32bitmap struct
#include "colorwheel.c"

#define COLOR_DEPTH 48                  // known working: 24, 48 - If the sketch uses type `rgb24` directly, COLOR_DEPTH must be 24
const uint16_t kMatrixWidth = 32;       // Set to the width of your display, must be a multiple of 8
const uint16_t kMatrixHeight = 32;      // Set to the height of your display
const uint8_t kRefreshDepth = 36;       // Tradeoff of color quality vs refresh rate, max brightness, and RAM usage.  36 is typically good, drop down to 24 if you need to.  On Teensy, multiples of 3, up to 48: 3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48.  On ESP32: 24, 36, 48
const uint8_t kDmaBufferRows = 4;       // known working: 2-4, use 2 to save RAM, more to keep from dropping frames and automatically lowering refresh rate.  (This isn't used on ESP32, leave as default)
const uint8_t kPanelType = SM_PANELTYPE_HUB75_32ROW_MOD16SCAN;   // Choose the configuration that matches your panels.  See more details in MatrixCommonHub75.h and the docs: https://github.com/pixelmatix/SmartMatrix/wiki
const uint32_t kMatrixOptions = (SM_HUB75_OPTIONS_NONE);        // see docs for options: https://github.com/pixelmatix/SmartMatrix/wiki
const uint8_t kBackgroundLayerOptions = (SM_BACKGROUND_OPTIONS_NONE);

SMARTMATRIX_ALLOCATE_BUFFERS(matrix, kMatrixWidth, kMatrixHeight, kRefreshDepth, kDmaBufferRows, kPanelType, kMatrixOptions);
SMARTMATRIX_ALLOCATE_BACKGROUND_LAYER(backgroundLayer, kMatrixWidth, kMatrixHeight, COLOR_DEPTH, kBackgroundLayerOptions);

#define COLOR_ORDER_RGB 0
#define COLOR_ORDER_RBG 1
#define COLOR_ORDER_GRB 2
#define COLOR_ORDER_GBR 3
#define COLOR_ORDER_BRG 4
#define COLOR_ORDER_BGR 5

/********************************** SKETCH OPTIONS ****************************************************/

#define COLOR_ORDER COLOR_ORDER_RGB

#if 1
    // Use these settings if you are sending data from Fadecandy that fills the entire matrix
    const int apa102DataBytesPerFrame = (kMatrixWidth * kMatrixHeight) * 4;
    int testPatternWidth = kMatrixWidth;
#else
    //Use these settings and change testPatternWidth/testPatternHeight to match the data you are sending from Fadecandy
    const int testPatternWidth = 64;
    const int testPatternHeight = 64;
    const int apa102DataBytesPerFrame = (testPatternWidth * testPatternHeight) * 4;
#endif

// apa102DataBytesToSkip is how many bytes (pixels * 4) to ignore before drawing to the buffer (unlikely to be useful when non-zero in Fadecandy context)
const int apa102DataBytesToSkip = 0;
//const int apa102DataBytesToSkip = (128 * 64) * 4;

// Fadecandy's Teensy4Device doesn't support sending the color lookup table, so for best results, enable local color correction using SmartMatrix's color correction
const bool colorCorrectionEnabled = true;

// Fadecandy examples typically use serpentineLayout by default (zigzag=true in Processing OPC), so we'll use it by default too
const bool serpentineLayout = true;

const int defaultBrightness = (100*255)/100;        // full (100%) brightness
//const int defaultBrightness = (15*255)/100;       // dim: 15% brightness

/******************************************************************************************************/

#if (COLOR_ORDER == COLOR_ORDER_RGB)
#define COLOR_ORDER_RED     0
#define COLOR_ORDER_GREEN   1
#define COLOR_ORDER_BLUE    2
#endif
#if (COLOR_ORDER == COLOR_ORDER_RBG)
#define COLOR_ORDER_RED     0
#define COLOR_ORDER_BLUE    1
#define COLOR_ORDER_GREEN   2
#endif
#if (COLOR_ORDER == COLOR_ORDER_GRB)
#define COLOR_ORDER_GREEN   0
#define COLOR_ORDER_RED     1
#define COLOR_ORDER_BLUE    2
#endif
#if (COLOR_ORDER == COLOR_ORDER_GBR)
#define COLOR_ORDER_GREEN   0
#define COLOR_ORDER_BLUE    1
#define COLOR_ORDER_RED     2
#endif
#if (COLOR_ORDER == COLOR_ORDER_BRG)
#define COLOR_ORDER_BLUE    0
#define COLOR_ORDER_RED     1
#define COLOR_ORDER_GREEN   2
#endif
#if (COLOR_ORDER == COLOR_ORDER_BGR)
#define COLOR_ORDER_BLUE    0
#define COLOR_ORDER_GREEN   1
#define COLOR_ORDER_RED     2
#endif

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

#define DATA_BUFFER_MAX_BYTES   128

uint8_t dataBuffer[DATA_BUFFER_MAX_BYTES];

SM_RGB *buffer;
SM_RGB color;

uint8_t gbc;
uint8_t largestGbc;
int x, y;
int framesReceived = 0;
uint32_t pixelBuffer32bit[kMatrixWidth*kMatrixHeight*sizeof(SM_RGB)];

// returns apa102Position:
// - return -1 for don't process current byte, currently waiting for Frame Start Message
// - return n>=0 for SOF received, buffer[i] contains n byte within APA102 message (0=first byte of first LED frame)
int getPositionWithinApa102Frame(uint8_t * bufferBytes, unsigned int dataBufferBytesToProcess, unsigned int currentPosition, int & bitOffset, uint8_t & previousBufferPartialByte) {
    static uint32_t lastFourBytes = 0xFFFFFFFFUL;
    static int apa102Position = -1;
    bool continueWithoutShifting = false;
    int previousBitOffset;

    // this function is looking for *the last instance* of 32 zero bits in a row, indicating an APA102 Frame Start, then aligns the data stream with the Frame Start message, and returns the current position within the APA102 message
    bool stillHopeFor32Zeros = false;

    // if we're in the middle of an APA102 frame, keep counting
    if(apa102Position >= 0)
        apa102Position++;

    // if lastFourBytes has this pattern: 0x??000000, there's a chance that we've recently received >=32 continuous zeros
    if((lastFourBytes & 0x00FFFFFFUL) == 0x00000000UL) {
        stillHopeFor32Zeros = true;
    }

    //printf("\r\nlastFourBytes = 0x000000 ");

    // if we have >= 32 continuous zero bits, but no non-zero bits after, we know we received a Frame Start, but can't yet align our data, so stop trying and wait for the next byte
    if (stillHopeFor32Zeros && bufferBytes[currentPosition] == 0x00) {
        //printf("\r\nall zeros, continue ");
        apa102Position = -1;
        stillHopeFor32Zeros = false;
    }

    if(stillHopeFor32Zeros) {    
        // look for 32 continuous zeros, and specifically *the last instance* of 32 continuous zeros
        uint16_t sideBytes = ((lastFourBytes & 0xFF000000) >> 16) | bufferBytes[currentPosition];
        // we know there are 24 continuous zeros already in the middle bytes, look for the last 8 zero bits on either side
        //printf("\r\nsidebytes: %04X ", sideBytes);

        previousBitOffset = bitOffset;
        stillHopeFor32Zeros = false;

        for (int j = 0; j < 8; j++) {
            //printf("\r\nsidebytes << %d= %04X ", j, sideBytes);
            if ((uint8_t)(sideBytes >> 8) == 0x00) {
                bitOffset = previousBitOffset + j;
                bitOffset %= 8;

                // we found a zero, but not necessarly the last zero, keep going until we know the correct value of bitOffset for aligning on the last zero
                stillHopeFor32Zeros = true;
                //printf("\r\nfoundzero: bitOffset %d ", bitOffset);
            }
            sideBytes <<= 1;
        }
    }

    if (stillHopeFor32Zeros) {
        apa102Position = 0;
        // if we have a new bitOffset, data already buffered needs to include the new shift
        if (bitOffset != previousBitOffset) {
            bool shiftLeftToShiftRight = false;
            int numShifts;

            if (bitOffset < previousBitOffset) {
                shiftLeftToShiftRight = true;
                numShifts = 8 - (previousBitOffset - bitOffset);
            } else {
                numShifts = (bitOffset - previousBitOffset);
            }

            // do left shifts
            lastFourBytes <<= numShifts;
            lastFourBytes |= (bufferBytes[currentPosition] <<= numShifts) >> 8;

            for (unsigned int k = currentPosition; k < dataBufferBytesToProcess; k++) {
                uint16_t tempWord = (bufferBytes[k] << 8) | bufferBytes[k + 1];
                tempWord <<= numShifts;
                bufferBytes[k] = (uint8_t)(tempWord >> 8);
            }

            bufferBytes[dataBufferBytesToProcess] <<= numShifts;

            if (shiftLeftToShiftRight) {
                // copy the data from dataBuffer[i+1] to dataBuffer[dataBufferBytesToProcess], as dataBuffer[dataBufferBytesToProcess] is now empty with a gap in data
                for (unsigned int k = currentPosition; k < dataBufferBytesToProcess; k++) {
                    bufferBytes[k + 1] = bufferBytes[k];
                }

                previousBufferPartialByte = bufferBytes[dataBufferBytesToProcess];

                // we shifted all the data right one byte, so bufferBytes[currentPosition] is unusable.  The first byte of first LED frame is in bufferBytes[currentPosition+1].  return without modifying lastFourBytes, and we'll handle bufferBytes[currentPosition+1] on the next pass through
                apa102Position = -1;

                // we can't add bufferBytes[currentPosition] to lastFourBytes as it's garbage data
                continueWithoutShifting = true;
                //printf("++shiftleft++:");
            }

            SerialUSB1.print("new: ");
            SerialUSB1.print(bitOffset);
            SerialUSB1.print(", ");
        }

        //printf("\r\nSOF received, first byte: %02X", dataBuffer[i]);
    }

    if(!continueWithoutShifting) {
        lastFourBytes <<= 8;
        lastFourBytes |= bufferBytes[currentPosition];
    }

    return apa102Position;
}

void setup() {
    SerialUSB1.println("setup");
    matrix.addLayer(&backgroundLayer); 
    matrix.begin();

    matrix.setBrightness(defaultBrightness);
    
    // default refresh rate is 240Hz, but you can tweak it for better results on camera if you want
    //matrix.setRefreshRate(249);

    int x, y;
    backgroundLayer.fillScreen({0,0,0});
    x = (kMatrixWidth / 2) - (colorwheel.width/2);
    y = (kMatrixHeight / 2) - (colorwheel.height/2);
    // can pass &colorwheel in directly as the bitmap source is already gimp32x32bitmap
    drawBitmap(x,y,&colorwheel);
    backgroundLayer.swapBuffers();
}

void loop() {
    static unsigned long lastPrintMs = 0;

    if(matrix.getRefreshRateLoweredFlag()) {
        //printf("REFRESH LOWERED\n");
        SerialUSB1.println("REFRESH LOWERED\n");
    }

    static bool counting = false;
    static bool countingFail = false;
    static bool runOncePerCountingFail;

    if (millis() - lastPrintMs > 1000) {
        lastPrintMs = millis();

        //SerialUSB1.println(framesReceived);
    }

    int dataBufferBytesToProcess = Serial.readBytes((char*)dataBuffer, DATA_BUFFER_MAX_BYTES);

    //printf("got %d\r\n", dataBufferBytesToProcess);
    //SerialUSB1.print("got ");
    //SerialUSB1.println(dataBufferBytesToProcess);

    // process all the data inside dataBuffer
    for (int i = 0; i < dataBufferBytesToProcess; i++) {
        if (!(i % 16)) {
            //printf("\r\n offset: %d | ", bitOffset);
        }
        //printf("%02X ", dataBuffer[i]);

        // we don't need these if the incoming data is always byte aligned
        int bitOffset = 0;
        uint8_t lastPartialByte = 0;

        int apa102Position = getPositionWithinApa102Frame(dataBuffer, dataBufferBytesToProcess, i, bitOffset, lastPartialByte);

        if(apa102Position < 0)
            continue;

        // SOF was just received 
        if(apa102Position == 0) {
            counting = true;
            runOncePerCountingFail = true;
            countingFail = false;
            x = 0;
            y = 0;
            largestGbc = 0;
        }

        // keep track of data inside SOF and EOF markers, make sure it stays in sequence and isn't too long or too short
        if (counting) {
#if 0
            if (apa102Position == 0) {
                printf("\r\n offset: %d | %02X ", bitOffset, dataBuffer[i]);
            }
            if (apa102Position == 1) {
                printf(",%02X ", dataBuffer[i]);
            }
#endif
            if(apa102Position >= apa102DataBytesToSkip) {
                if (apa102Position % 4 == 0) {
                    gbc = dataBuffer[i] & 0x1F;
                    if(gbc > largestGbc)
                        largestGbc = gbc;

                    if ((dataBuffer[i] & 0xE0) != 0xE0) {
                        if (runOncePerCountingFail) {
                            //printf("-%02X:%d:%02X:%02X:%02X:%02X,", dataBuffer[i], apa102Position, dataBuffer[i + 1], dataBuffer[i + 2], dataBuffer[i + 3], dataBuffer[i + 4]);
                            SerialUSB1.print("-");
                            runOncePerCountingFail = false;
                        }
                        countingFail = true;
                        counting = false;
                        continue;
                    }
                } else {
                    uint16_t tempIntensity = ((((uint16_t)dataBuffer[i] * gbc) + (gbc >> 1)) << 8 ) / 0x20;

                    if(COLOR_DEPTH == 24)
                        tempIntensity >>= 8;

                    if (apa102Position % 4 == 1) {
#if (COLOR_ORDER_RED == 0)                            
                        color.red = tempIntensity;
#endif
#if (COLOR_ORDER_GREEN == 0)                            
                        color.green = tempIntensity;
#endif
#if (COLOR_ORDER_BLUE == 0)                            
                        color.blue = tempIntensity;
#endif
                    }
                    if (apa102Position % 4 == 2) {
#if (COLOR_ORDER_RED == 1)                            
                        color.red = tempIntensity;
#endif
#if (COLOR_ORDER_GREEN == 1)                            
                        color.green = tempIntensity;
#endif
#if (COLOR_ORDER_BLUE == 1)                            
                        color.blue = tempIntensity;
#endif
                    }
                    if (apa102Position % 4 == 3) {
#if (COLOR_ORDER_RED == 2)                            
                        color.red = tempIntensity;
#endif
#if (COLOR_ORDER_GREEN == 2)                            
                        color.green = tempIntensity;
#endif
#if (COLOR_ORDER_BLUE == 2)                            
                        color.blue = tempIntensity;
#endif

                        static SM_RGB previousPixel;

#if (COLOR_DEPTH == 48)
                        // store pixels temporarily into pixelBuffer32bit (32-bit aligned access only), storing two pixels at once (2x48 is multiple of 32 bits)
                        if(x%2 == 0) {
                            previousPixel = color;
                        } else {
                            int bigBufferOffset;
                            if(serpentineLayout && y%2) {
                                bigBufferOffset = (sizeof(SM_RGB) * (((testPatternWidth - 1) - x) + (y * kMatrixWidth))) / sizeof(uint32_t);

                                pixelBuffer32bit[bigBufferOffset + 0] = (color.blue << 16) | (color.red << 0);
                                pixelBuffer32bit[bigBufferOffset + 1] = (previousPixel.red << 16) |  (color.green << 0);
                                pixelBuffer32bit[bigBufferOffset + 2] = (previousPixel.green << 16) | (previousPixel.blue << 0);
                            } else {
                                bigBufferOffset = (sizeof(SM_RGB) * ((x-1) + (y * kMatrixWidth))) / sizeof(uint32_t);

                                pixelBuffer32bit[bigBufferOffset + 0] = (previousPixel.blue << 16) | (previousPixel.red << 0);
                                pixelBuffer32bit[bigBufferOffset + 1] = (color.red << 16) |  (previousPixel.green << 0);
                                pixelBuffer32bit[bigBufferOffset + 2] = (color.green << 16) | (color.blue << 0);
                            }
                        }
#else   // COLOR_DEPTH == 24
                        // not yet supported
#endif
                        x++;
                        if (x >= testPatternWidth) {
                            x = 0;
                            y++;
                        }
                    }
                }
            }   

            if (apa102Position == apa102DataBytesPerFrame + apa102DataBytesToSkip - 1) {
                counting = false;
                if (!countingFail) {
                    framesReceived++;

#if 1
                    if (!backgroundLayer.isSwapPending()){
#else
                    while(backgroundLayer.isSwapPending());
                    if(1) {
#endif
#ifdef DEBUG_I2SIN_1_GPIO
                        gpio_set_level(DEBUG_I2SIN_1_GPIO, 1);
#endif
                        // copy pixels from temporary buffer (note: 32-bit aligned) to backgroundLayer
                        buffer = backgroundLayer.backBuffer();
                        memcpy(buffer, pixelBuffer32bit, kMatrixWidth*kMatrixHeight*sizeof(SM_RGB));
#ifdef DEBUG_I2SIN_1_GPIO
                        gpio_set_level(DEBUG_I2SIN_1_GPIO, 0);
                        gpio_set_level(DEBUG_I2SIN_1_GPIO, 1);
#endif
                        // scaling fails if largestGbc==0 (all pixels were black)
                        if(!largestGbc)
                            largestGbc = 0x1F;

                        // figure out how many times we can double background layer brightness without exceeding the max
                        int numShifts=0;
                        while(!(largestGbc / (0x01 << (4-numShifts))))
                            numShifts++;

                        static uint16_t linex = 0;

                        //backgroundLayer.drawLine(linex, 0, linex, kMatrixHeight, {0xffff, 0xffff, 0xffff});
                        linex++;
                        if(linex >= kMatrixWidth)
                          linex = 0;

                        //backgroundLayer.setBrightnessShifts(numShifts);
                        backgroundLayer.swapBuffers(false);
#ifdef DEBUG_I2SIN_1_GPIO
                        gpio_set_level(DEBUG_I2SIN_1_GPIO, 0);
#endif
                        //while(backgroundLayer.isSwapPending());
                    } else {

                    }
                    int fps = matrix.countFPS();
                    if(fps) {
                        SerialUSB1.print("Loops last second:");
                        SerialUSB1.println(fps);
                    }
                }
            }
        }
    }
}
