/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/i2s.h>
#include <drivers/uart.h>
#include <stdlib.h>
#include <logging/log.h>
#define AUDIO_SAMPLE_FREQ		(48000)
#define AUDIO_SAMPLES_PER_CH_PER_FRAME	(128)
#define AUDIO_NUM_CHANNELS		(2)
#define AUDIO_SAMPLES_PER_FRAME		\
	(AUDIO_SAMPLES_PER_CH_PER_FRAME * AUDIO_NUM_CHANNELS)
#define AUDIO_SAMPLE_BYTES		(8)
#define AUDIO_SAMPLE_BIT_WIDTH		(24)

#define AUDIO_FRAME_BUF_BYTES		\
	(AUDIO_SAMPLES_PER_FRAME * AUDIO_SAMPLE_BYTES)

#define I2S_PLAY_BUF_COUNT		(2100)


static struct k_mem_slab i2s_mem_slab;
static struct device *host_i2s_dev;
static struct device *uart_dev;
static char audio_buffers[AUDIO_FRAME_BUF_BYTES*I2S_PLAY_BUF_COUNT];
void main(void)
{
	printk("Hello World! %s\n", CONFIG_BOARD);

	struct i2s_config i2s_cfg;
    int ret;
	
    host_i2s_dev = device_get_binding("i2s_rx");
    uart_dev = device_get_binding("uart0");
	k_mem_slab_init(&i2s_mem_slab, audio_buffers, AUDIO_FRAME_BUF_BYTES,
			I2S_PLAY_BUF_COUNT);
	if (!host_i2s_dev) 
    {
		printk("unable to find i2s_rx device");
	    exit(-1);
	}
	/* configure i2s for audio playback */
	i2s_cfg.word_size = AUDIO_SAMPLE_BIT_WIDTH;
	i2s_cfg.channels = AUDIO_NUM_CHANNELS;
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED | I2S_FMT_CLK_NF_NB;
	i2s_cfg.options = I2S_OPT_FRAME_CLK_SLAVE |
		I2S_OPT_BIT_CLK_SLAVE;
	i2s_cfg.frame_clk_freq = AUDIO_SAMPLE_FREQ;
	i2s_cfg.block_size = AUDIO_FRAME_BUF_BYTES;
	i2s_cfg.mem_slab = &i2s_mem_slab;
	/* make the receive interface blocking */
	i2s_cfg.timeout = K_FOREVER;
	ret = i2s_configure(host_i2s_dev, I2S_DIR_RX, &i2s_cfg);
	if (ret != 0)
    {
		printk("dmic_configure failed with %d error", ret);
		exit(-1);
	}

	ret = i2s_trigger(host_i2s_dev, I2S_DIR_RX, I2S_TRIGGER_START);
	if (ret != 0) {
		printk("dmic_trigger failed with %d error", ret);
		exit(-1);
	}
    // wait for buffer
    while(is_i2s_full() == false){
        uart_poll_out(uart_dev,'r');
        k_sleep(10);
    }
    // stop i2s transimsion
    i2s_trigger(host_i2s_dev, I2S_DIR_RX, I2S_TRIGGER_STOP);
    sys_write8(0xf, 0x82006000);
    //send signal to ack python
    uart_poll_out(uart_dev, 0x0);

    void*in_buf;
    size_t size;
    for(int k =0; k< 500; k++)
    {
        // big endian
        ret = i2s_read(host_i2s_dev,&in_buf, &size);
        for(int j =0; j < 256*4; j++)
        {
            uart_poll_out(uart_dev, *(((u8_t*)in_buf)+j));
        }
        k_mem_slab_free(&i2s_mem_slab, &in_buf);
    }
    // sending data done
    sys_write8(0x0, 0x82006000);
    // if my callculated something wrong, fill rest py buffer with 0x0
    while(true)
    {
        uart_poll_out(uart_dev, 0x0);
    }
}
