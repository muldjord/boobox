/* i2s.c
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

#include "i2s.h"
#include <math.h>
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "i2s.pio.h"

static void dma_double_buffer_init(pio_i2s* i2s, void (*dma_handler)(void)) {
    i2s->dma_ch_out_ctrl = dma_claim_unused_channel(true);
    i2s->dma_ch_out_data = dma_claim_unused_channel(true);

    i2s->out_ctrl_blocks[0] = i2s->output_buffer;
    i2s->out_ctrl_blocks[1] = &i2s->output_buffer[STEREO_BUFFER_SIZE];

    dma_channel_config c = dma_channel_get_default_config(i2s->dma_ch_out_ctrl);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_ring(&c, false, 3);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    dma_channel_configure(i2s->dma_ch_out_ctrl, &c, &dma_hw->ch[i2s->dma_ch_out_data].al3_read_addr_trig, i2s->out_ctrl_blocks, 1, false);

    c = dma_channel_get_default_config(i2s->dma_ch_out_data);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_chain_to(&c, i2s->dma_ch_out_ctrl);
    channel_config_set_dreq(&c, pio_get_dreq(i2s->pio, i2s->sm_dout, true));

    dma_channel_configure(i2s->dma_ch_out_data,
                          &c,
                          &i2s->pio->txf[i2s->sm_dout],
                          NULL,
                          STEREO_BUFFER_SIZE,
                          false
    );

    dma_channel_set_irq0_enabled(i2s->dma_ch_out_data, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    dma_channel_start(i2s->dma_ch_out_ctrl);
}

static void i2s_sync_program_init(PIO pio, pio_i2s* i2s) {
    uint offset  = 0;
    i2s->pio     = pio;
    i2s->sm_mask = 0;

    i2s->sm_dout = pio_claim_unused_sm(pio, true);
    i2s->sm_mask |= (1u << i2s->sm_dout);
    offset = pio_add_program(pio0, &i2s_out_master_program);
    // 4th argument is bit depth, 5th dout, 6th bclk pin base (lrclk is bclk pin + 1)
    i2s_out_master_program_init(pio, i2s->sm_dout, offset, 32, 6, 8);
    pio_sm_set_clkdiv(pio, i2s->sm_dout, 89); // Approximately 11KHz audio
}

void i2s_program_start_synched(PIO pio, void (*dma_handler)(void), pio_i2s* i2s) {
  i2s_sync_program_init(pio, i2s);
  dma_double_buffer_init(i2s, dma_handler);
  pio_enable_sm_mask_in_sync(i2s->pio, i2s->sm_mask);
}
