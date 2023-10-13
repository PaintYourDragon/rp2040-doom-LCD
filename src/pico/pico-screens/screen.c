#include "screen.h"
#include "hardware/gpio.h"

#if LILYGO_TTGO
#include "screens/lilygo_ttgo.h"
#endif

#if ST7789_240_135
#include "screens/st7789_240_135.h"
#endif

#if ILI9225_220_176
#include "screens/ili9225_220_176.h"
#endif

#if SSD1306_70_40
#include "screens/ssd1306_70_40.hpp"
#endif

#if SSD1306_70_40_i2c
#include "screens/ssd1306_70_40_i2c.h"
#endif

#if ST7735_128_128
#include "screens/st7735_128_128.h"
#endif

#if SSD1306_70_40
static void* ssd1306_70_40;
#endif



void I_initScreen(void) {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
#if ILI9225_220_176
    ili9225_220_176_initScreen();
#elif ST7789_240_135
    st7789_240_135_initScreen();
#elif LILYGO_TTGO
    lilygo_ttgo_initScreen();
#elif SSD1306_70_40
    ssd1306_70_40_initScreen();
#elif SSD1306_70_40_i2c
    ssd1306_70_40_i2c_initScreen();
// TODO get better aspect ratio screen
#elif ST7735_128_128
    st7735_128_128_initScreen();
#endif
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
}

void I_handleFrameStart(uint8_t frame) {
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
#if ILI9225_220_176
    ili9225_220_176_handleFrameStart(frame);
#elif ST7789_240_135
    st7789_240_135_handleFrameStart(frame);
#elif LILYGO_TTGO
    lilygo_ttgo_handleFrameStart(frame);
#elif SSD1306_70_40
    ssd1306_70_40_handleFrameStart(frame);
#elif SSD1306_70_40_i2c
    ssd1306_70_40_i2c_handleFrameStart(frame);
#elif ST7735_128_128
    st7735_128_128_handleFrameStart(frame);
#endif

}

void I_handleScanline(uint16_t *line, int scanline) {
    gpio_put(PICO_DEFAULT_LED_PIN, (scanline >> 7) & 1);
#if ILI9225_220_176
    ili9225_220_176_handleScanline(line, scanline);
#elif ST7789_240_135
    st7789_240_135_handleScanline(line, scanline);
#elif LILYGO_TTGO
    lilygo_ttgo_handleScanline(line, scanline);
#elif SSD1306_70_40
    ssd1306_70_40_handleScanline(line, scanline);
#elif SSD1306_70_40_i2c
    ssd1306_70_40_i2c_handleScanline(line, scanline);
#elif ST7735_128_128
    st7735_128_128_handleScanline(line, scanline);
#endif
}

void I_handleFrameEnd(uint8_t frame) {
#if SSD1306_70_40
    ssd1306_70_40_handleFrameEnd(frame);
#elif SSD1306_70_40_i2c
    ssd1306_70_40_i2c_handleFrameEnd(frame);
#endif
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
}
