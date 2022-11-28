/* i2s.h
 *
 * Author: Daniel Collins
 * Date:   2022-02-25
 *
 * Copyright (c) 2022 Daniel Collins
 *
 * This file is part of rp2040_i2s_example.
 *
 * rp2040_i2s_example is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License, version 3 as published by the
 * Free Software Foundation.
 *
 * rp2040_i2s_example is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * rp2040_i2s_example. If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include "hardware/pio.h"

#ifndef I2S_H
#define I2S_H

#define AUDIO_BUFFER_FRAMES 48
#define STEREO_BUFFER_SIZE  AUDIO_BUFFER_FRAMES * 2

typedef struct pio_i2s {
    PIO        pio;
    uint8_t    sm_mask;
    uint8_t    sm_dout;
    uint       dma_ch_out_ctrl;
    uint       dma_ch_out_data;
    int32_t*   out_ctrl_blocks[2];
    int32_t    output_buffer[STEREO_BUFFER_SIZE * 2];
} pio_i2s;

void i2s_program_start_synched(PIO pio, void (*dma_handler)(void), pio_i2s* i2s);

#endif // I2S_H
