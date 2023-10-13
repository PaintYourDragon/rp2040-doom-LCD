#include <string.h>

#include "hardware/gpio.h"

#include "pico/ili9225.h"

#define TFT_RD  8
#define TFT_WR  9
#define TFT_DC  10
#define TFT_RST 11
#define TFT_CS  12

// Much of the TFT-writing code is adapted from GFX_Library_for_Arduino.
// https://github.com/moononournation/Arduino_GFX

#define ILI9225_TFTWIDTH 220
#define ILI9225_TFTHEIGHT 176

#define ILI9225_RST_DELAY 150    ///< delay ms wait for reset finish

#define ILI9225_DRIVER_OUTPUT_CTRL 0x01      // Driver Output Control
#define ILI9225_LCD_AC_DRIVING_CTRL 0x02     // LCD AC Driving Control
#define ILI9225_ENTRY_MODE 0x03              // Entry Mode
#define ILI9225_DISP_CTRL1 0x07              // Display Control 1
#define ILI9225_BLANK_PERIOD_CTRL1 0x08      // Blank Period Control
#define ILI9225_FRAME_CYCLE_CTRL 0x0B        // Frame Cycle Control
#define ILI9225_INTERFACE_CTRL 0x0C          // Interface Control
#define ILI9225_OSC_CTRL 0x0F                // Osc Control
#define ILI9225_POWER_CTRL1 0x10             // Power Control 1
#define ILI9225_POWER_CTRL2 0x11             // Power Control 2
#define ILI9225_POWER_CTRL3 0x12             // Power Control 3
#define ILI9225_POWER_CTRL4 0x13             // Power Control 4
#define ILI9225_POWER_CTRL5 0x14             // Power Control 5
#define ILI9225_VCI_RECYCLING 0x15           // VCI Recycling
#define ILI9225_RAM_ADDR_SET1 0x20           // Horizontal GRAM Address Set
#define ILI9225_RAM_ADDR_SET2 0x21           // Vertical GRAM Address Set
#define ILI9225_GRAM_DATA_REG 0x22           // GRAM Data Register
#define ILI9225_GATE_SCAN_CTRL 0x30          // Gate Scan Control Register
#define ILI9225_VERTICAL_SCROLL_CTRL1 0x31   // Vertical Scroll Control 1 Register
#define ILI9225_VERTICAL_SCROLL_CTRL2 0x32   // Vertical Scroll Control 2 Register
#define ILI9225_VERTICAL_SCROLL_CTRL3 0x33   // Vertical Scroll Control 3 Register
#define ILI9225_PARTIAL_DRIVING_POS1 0x34    // Partial Driving Position 1 Register
#define ILI9225_PARTIAL_DRIVING_POS2 0x35    // Partial Driving Position 2 Register
#define ILI9225_HORIZONTAL_WINDOW_ADDR1 0x36 // Horizontal Address Start Position
#define ILI9225_HORIZONTAL_WINDOW_ADDR2 0x37 // Horizontal Address End Position
#define ILI9225_VERTICAL_WINDOW_ADDR1 0x38   // Vertical Address Start Position
#define ILI9225_VERTICAL_WINDOW_ADDR2 0x39   // Vertical Address End Position
#define ILI9225_GAMMA_CTRL1 0x50             // Gamma Control 1
#define ILI9225_GAMMA_CTRL2 0x51             // Gamma Control 2
#define ILI9225_GAMMA_CTRL3 0x52             // Gamma Control 3
#define ILI9225_GAMMA_CTRL4 0x53             // Gamma Control 4
#define ILI9225_GAMMA_CTRL5 0x54             // Gamma Control 5
#define ILI9225_GAMMA_CTRL6 0x55             // Gamma Control 6
#define ILI9225_GAMMA_CTRL7 0x56             // Gamma Control 7
#define ILI9225_GAMMA_CTRL8 0x57             // Gamma Control 8
#define ILI9225_GAMMA_CTRL9 0x58             // Gamma Control 9
#define ILI9225_GAMMA_CTRL10 0x59            // Gamma Control 10

typedef enum
{
  BEGIN_WRITE,
  WRITE_C8_D16,
  END_WRITE,
  DELAY,
} spi_operation_type_t;

union
{
  uint16_t value;
  struct
  {
    uint8_t lsb;
    uint8_t msb; 
  };
} _data16;

static inline void WRITE(uint8_t d)
{ 
  sio_hw->gpio_clr = (1UL << TFT_WR) | 0xFF;
  sio_hw->gpio_set = d;
  asm("nop");
  sio_hw->gpio_set = 1UL << TFT_WR;
  asm("nop");
}   

/******** low level bit twiddling **********/

static inline void DC_HIGH(void)
{
  sio_hw->gpio_set = 1UL << TFT_DC;
}

static inline void DC_LOW(void)
{
  sio_hw->gpio_clr = 1UL << TFT_DC;
}

static inline void CS_HIGH(void)
{
  sio_hw->gpio_set = 1UL << TFT_CS;
}

static inline void CS_LOW(void)
{
  sio_hw->gpio_clr = 1UL << TFT_CS;
}

static void beginWrite()
{
  DC_HIGH();
  CS_LOW();
}

static void endWrite()
{
  CS_HIGH();
}

static void writeCommand(uint8_t c)
{
  DC_LOW();

  WRITE(c);

  DC_HIGH();
}

static void writeC8D16(uint8_t c, uint16_t d)
{
  DC_LOW();

  WRITE(c);

  DC_HIGH();

  _data16.value = d;
  WRITE(_data16.msb);
  WRITE(_data16.lsb);
}

static void batchOperation(const uint8_t *operations, size_t len)
{
  uint8_t command;
  for (size_t i = 0; i < len; ++i)
  {
    switch (operations[i])
    {
    case BEGIN_WRITE:
      beginWrite();
      break;
    case WRITE_C8_D16:
      command = operations[++i];
      _data16.msb = operations[++i];
      _data16.lsb = operations[++i];
      writeC8D16(command, _data16.value);
      break;
    case END_WRITE:
      endWrite();
      break;
    case DELAY:
      sleep_ms(operations[++i]);
      break;
    }
  }
}

static const uint8_t ili9225_init_operations[] = {
    BEGIN_WRITE,
    WRITE_C8_D16, ILI9225_LCD_AC_DRIVING_CTRL, 0x01, 0x00,
    WRITE_C8_D16, ILI9225_BLANK_PERIOD_CTRL1, 0x08, 0x08, // set BP and FP
    WRITE_C8_D16, ILI9225_FRAME_CYCLE_CTRL, 0x11, 0x00,   // frame cycle
    WRITE_C8_D16, ILI9225_INTERFACE_CTRL, 0x00, 0x00,     // RGB interface setting R0Ch=0x0110 for RGB 18Bit and R0Ch=0111for RGB16Bit
    WRITE_C8_D16, ILI9225_OSC_CTRL, 0x14, 0x01,           // Set frame rate----0801
    WRITE_C8_D16, ILI9225_VCI_RECYCLING, 0x00, 0x00,      // set system interface
    END_WRITE,

    DELAY, 50,

    //*************Power On sequence ****************//
    BEGIN_WRITE,
    WRITE_C8_D16, ILI9225_POWER_CTRL1, 0x08, 0x00, // Set SAP,DSTB,STB----0A00
    WRITE_C8_D16, ILI9225_POWER_CTRL2, 0x1F, 0x3F, // Set APON,PON,AON,VCI1EN,VC----1038
    END_WRITE,

    DELAY, 50,

    BEGIN_WRITE,
    WRITE_C8_D16, ILI9225_POWER_CTRL3, 0x01, 0x21, // Internal reference voltage= Vci;----1121
    WRITE_C8_D16, ILI9225_POWER_CTRL4, 0x00, 0x6F, // Set GVDD----0066
    WRITE_C8_D16, ILI9225_POWER_CTRL5, 0x43, 0x49, // Set VCOMH/VCOML voltage----5F60
    //-------------- Set GRAM area -----------------//
    WRITE_C8_D16, ILI9225_GATE_SCAN_CTRL, 0x00, 0x00,
    WRITE_C8_D16, ILI9225_VERTICAL_SCROLL_CTRL1, 0x00, 0xDB,
    WRITE_C8_D16, ILI9225_VERTICAL_SCROLL_CTRL2, 0x00, 0x00,
    WRITE_C8_D16, ILI9225_VERTICAL_SCROLL_CTRL3, 0x00, 0x00,
    WRITE_C8_D16, ILI9225_PARTIAL_DRIVING_POS1, 0x00, 0xDB,
    WRITE_C8_D16, ILI9225_PARTIAL_DRIVING_POS2, 0x00, 0x00,
    // ----------- Adjust the Gamma Curve ----------//
    WRITE_C8_D16, ILI9225_GAMMA_CTRL1, 0x00, 0x01,  // 0x0400
    WRITE_C8_D16, ILI9225_GAMMA_CTRL2, 0x20, 0x0B,  // 0x060B
    WRITE_C8_D16, ILI9225_GAMMA_CTRL3, 0x00, 0x00,  // 0x0C0A
    WRITE_C8_D16, ILI9225_GAMMA_CTRL4, 0x04, 0x04,  // 0x0105
    WRITE_C8_D16, ILI9225_GAMMA_CTRL5, 0x0C, 0x0C,  // 0x0A0C
    WRITE_C8_D16, ILI9225_GAMMA_CTRL6, 0x00, 0x0C,  // 0x0B06
    WRITE_C8_D16, ILI9225_GAMMA_CTRL7, 0x01, 0x01,  // 0x0004
    WRITE_C8_D16, ILI9225_GAMMA_CTRL8, 0x04, 0x00,  // 0x0501
    WRITE_C8_D16, ILI9225_GAMMA_CTRL9, 0x11, 0x08,  // 0x0E00
    WRITE_C8_D16, ILI9225_GAMMA_CTRL10, 0x05, 0x0C, // 0x000E
    END_WRITE,

    DELAY, 50,

    BEGIN_WRITE,
    WRITE_C8_D16, ILI9225_DISP_CTRL1, 0x10, 0x17,
    END_WRITE};

/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

static void init_output(int pin, int state) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, state);
}

void ili9225_init(void)
{
    init_output(TFT_RD, 1);
    init_output(TFT_WR, 1); // Write strobe high = idle
    init_output(TFT_DC, 1); // Data mode
    init_output(TFT_CS, 1); // Deselect

    // D0-D7 = GPIO0-7
    for (int i=0; i<8; i++) {
        init_output(i, 0);
    }

    init_output(TFT_RST, 1);
    sleep_ms(100);
    gpio_put(TFT_RST, 0);
    sleep_ms(ILI9225_RST_DELAY);
    gpio_put(TFT_RST, 1);
    sleep_ms(ILI9225_RST_DELAY);

    // Rest of init from Arduino lib
    batchOperation(ili9225_init_operations, sizeof(ili9225_init_operations));

    // Set for landscape w/vert flip
    beginWrite();
    writeC8D16(ILI9225_DRIVER_OUTPUT_CTRL, 0x011C);
    writeC8D16(ILI9225_ENTRY_MODE, 0x1038);
    endWrite();
}

void ili9225_set_cursor(uint16_t x, uint16_t y)
{
    beginWrite();
    writeC8D16(ILI9225_VERTICAL_WINDOW_ADDR2, x);
    writeC8D16(ILI9225_VERTICAL_WINDOW_ADDR1, ILI9225_TFTWIDTH);
    writeC8D16(ILI9225_RAM_ADDR_SET2, x);
    writeC8D16(ILI9225_HORIZONTAL_WINDOW_ADDR2, y);
    writeC8D16(ILI9225_HORIZONTAL_WINDOW_ADDR1, ILI9225_TFTWIDTH);
    writeC8D16(ILI9225_RAM_ADDR_SET1, y);
    writeCommand(ILI9225_GRAM_DATA_REG); // Switches to data on completion
    // Write is "held open" (an ili9225_write() call will follow)
}

void ili9225_write(const void* data, size_t count)
{
    // Chip is assumed previously selected
    uint16_t *p = (uint16_t *)data;
    while (count--)
    {
      _data16.value = *p++;
      WRITE(_data16.msb);
      WRITE(_data16.lsb);
    }
    endWrite();
}

void ili9225_fill(uint16_t pixel)
{
    int num_pixels = ILI9225_TFTWIDTH * ILI9225_TFTHEIGHT;
    _data16.value = pixel;

    ili9225_set_cursor(0, 0); // Initiates write
    while (num_pixels--) {
        WRITE(_data16.msb);
        WRITE(_data16.lsb);
    }
    endWrite();
}
