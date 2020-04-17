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
#define AUDIO_SAMPLE_BYTES		(32)
#define AUDIO_SAMPLE_BIT_WIDTH		(24)

#define AUDIO_FRAME_BUF_BYTES		\
	(AUDIO_SAMPLES_PER_FRAME * AUDIO_SAMPLE_BYTES)

#define I2S_PLAY_BUF_COUNT		(2100)

static struct device *host_i2s_dev;
static struct device *uart_dev;
static struct k_mem_slab i2s_mem_slab;
static char audio_buffers[AUDIO_FRAME_BUF_BYTES*I2S_PLAY_BUF_COUNT];

struct i2s_frame_info
{
	void *mem_block;
	size_t size;
};

void main(void)
{
	printk("Hello World! %s\n", CONFIG_BOARD);

	struct i2s_config i2s_cfg;
    int ret;
    host_i2s_dev = device_get_binding("i2s_rx");
    uart_dev = device_get_binding("uart0");
    // wait a while
    k_sleep(K_SECONDS(6));
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
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_MASK;
	i2s_cfg.options = I2S_OPT_FRAME_CLK_SLAVE;
	i2s_cfg.frame_clk_freq = AUDIO_SAMPLE_FREQ;
	i2s_cfg.block_size = AUDIO_FRAME_BUF_BYTES;
	i2s_cfg.mem_slab = &i2s_mem_slab;
	i2s_cfg.timeout = K_FOREVER;
	ret = i2s_configure(host_i2s_dev, I2S_DIR_RX, &i2s_cfg);

	if (ret != 0)
    {
		printk("i2s_configure failed with %d error", ret);
		exit(-1);
	}
    /* start i2s rx driver */
	ret = i2s_trigger(host_i2s_dev, I2S_DIR_RX, I2S_TRIGGER_START);
	if (ret != 0) {
		printk("i2s_trigger failed with %d error", ret);
		exit(-1);
	}
    /* receive data */
    struct i2s_frame_info received_data[I2S_PLAY_BUF_COUNT];
    for(int i =0; i<I2S_PLAY_BUF_COUNT-600; i++ )
    {
        ret = i2s_read(host_i2s_dev,
                &received_data[i].mem_block,
                &received_data[i].size);
    }
    // stop i2s transimsion
    ret = i2s_trigger(host_i2s_dev, I2S_DIR_RX, I2S_TRIGGER_STOP);
	if (ret != 0) {
		printk("i2s_trigger failed with %d error", ret);
		exit(-1);
	}
    // this part of code is not related to i2s driver
    // it's just for debug purpose, but you can find here how to extract data
    // send signal to ack python
    uart_poll_out(uart_dev, 0x0);
    void *in_buf;
    size_t size;
    for(int i =0; i <I2S_PLAY_BUF_COUNT-600 ; i++)
    {
        in_buf = received_data[i].mem_block;
        size = received_data[i].size;
        for(int j =0; j < size; j++)
        {
            uart_poll_out(uart_dev, *(((u8_t*)in_buf)+j));
        }
        k_mem_slab_free(&i2s_mem_slab, &in_buf);
    }
    // sending data done
    while(true)
    {
        uart_poll_out(uart_dev, 0x0);
    }
}
