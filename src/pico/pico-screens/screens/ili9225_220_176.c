#include "ili9225_220_176.h"

void ili9225_220_176_initScreen(void) {
    ili9225_init();

    ili9225_fill(0x0000);
}

void ili9225_220_176_handleFrameStart(uint8_t frame) {
    nearestNeighborHandleFrameStart();
}

void ili9225_220_176_blit(uint16_t *downsampled_line, int scanline) {
    ili9225_set_cursor(SCREEN_WIDTH_OFFSET, (MEMORY_HEIGHT - LCD_HEIGHT) / 2 + (scanline));
    ili9225_write(downsampled_line, DOWNSAMPLED_WIDTH);
}

void ili9225_220_176_handleScanline(uint16_t *line, int scanline) {
    nearestNeighborHandleDownsampling(line, scanline, ili9225_220_176_blit);
}
