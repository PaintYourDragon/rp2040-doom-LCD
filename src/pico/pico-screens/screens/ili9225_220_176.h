// DESCRIPTION:
// LED / LCD screen-specific logic

#ifndef __IL9225_220_176__
#define __IL9225_220_176__

#include "shared.h"

#include "pico.h"

#include <stdlib.h>
#include "pico/ili9225.h"

#define MEMORY_WIDTH 320
#define MEMORY_HEIGHT 240

#define LCD_WIDTH 220
#define LCD_HEIGHT 176

#define SCREEN_WIDTH_OFFSET ((LCD_WIDTH - (SCREENWIDTH * 100 / DOWNSAMPLING_FACTOR_OUT_OF_100)) / 2)


void ili9225_220_176_initScreen(void);
void ili9225_220_176_handleFrameStart(uint8_t frame);
void ili9225_220_176_handleScanline(uint16_t *line, int scanline);

#endif
