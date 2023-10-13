/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

#ifndef _PICO_ILI9225_H_
#define _PICO_ILI9225_H_

void ili9225_init(void);
void ili9225_set_cursor(uint16_t x, uint16_t y);
void ili9225_write(const void* data, size_t len);
void ili9225_fill(uint16_t pixel);

#endif
