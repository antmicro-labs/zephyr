/*
 * Copyright (c) 2020 Antmicro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/i2s.h>
#include <drivers/uart.h>
#include <stdlib.h>
#include <logging/log.h>
#define AUDIO_SAMPLE_FREQ (44100)
#define AUDIO_SAMPLES_PER_CH_PER_FRAME (128)
#define AUDIO_NUM_CHANNELS (2)
#define AUDIO_SAMPLES_PER_FRAME                                                \
	(AUDIO_SAMPLES_PER_CH_PER_FRAME * AUDIO_NUM_CHANNELS)
#define AUDIO_SAMPLE_BYTES (3)
#define AUDIO_SAMPLE_BIT_WIDTH (24)

#define AUDIO_FRAME_BUF_BYTES (AUDIO_SAMPLES_PER_FRAME * AUDIO_SAMPLE_BYTES)

#define I2S_PLAY_BUF_COUNT (2000)

static struct device *host_i2s_rx_dev;
static struct device *host_i2s_tx_dev;
static struct device *uart_dev;
static struct k_mem_slab i2s_rx_mem_slab;
static struct k_mem_slab i2s_tx_mem_slab;
static char audio_buffers[AUDIO_FRAME_BUF_BYTES * I2S_PLAY_BUF_COUNT];
static char user_buffer[AUDIO_FRAME_BUF_BYTES * I2S_PLAY_BUF_COUNT];
static struct i2s_config i2s_rx_cfg;
static struct i2s_config i2s_tx_cfg;
static int ret;

void init()
{
	//configure rx device
	host_i2s_rx_dev = device_get_binding("i2s_rx");
	if (!host_i2s_rx_dev) {
		printk("unable to find i2s_rx device");
		exit(-1);
	}
	k_mem_slab_init(&i2s_rx_mem_slab, audio_buffers, AUDIO_FRAME_BUF_BYTES,
			I2S_PLAY_BUF_COUNT);

	/* configure i2s for audio playback */
	i2s_rx_cfg.word_size = AUDIO_SAMPLE_BIT_WIDTH;
	i2s_rx_cfg.channels = AUDIO_NUM_CHANNELS;
	i2s_rx_cfg.format = I2S_FMT_DATA_FORMAT_MASK;
	i2s_rx_cfg.options = I2S_OPT_FRAME_CLK_SLAVE;
	i2s_rx_cfg.frame_clk_freq = AUDIO_SAMPLE_FREQ;
	i2s_rx_cfg.block_size = AUDIO_FRAME_BUF_BYTES;
	i2s_rx_cfg.mem_slab = &i2s_rx_mem_slab;
	i2s_rx_cfg.timeout = K_FOREVER;
	ret = i2s_configure(host_i2s_rx_dev, I2S_DIR_RX, &i2s_rx_cfg);

	if (ret != 0) {
		printk("i2s_configure failed with %d error", ret);
		exit(-1);
	}

	//configure tx device
	host_i2s_tx_dev = device_get_binding("i2s_tx");
	if (!host_i2s_tx_dev) {
		printk("unable to find i2s_tx device");
		exit(-1);
	}
	k_mem_slab_init(&i2s_tx_mem_slab, audio_buffers, AUDIO_FRAME_BUF_BYTES,
			I2S_PLAY_BUF_COUNT);

	/* configure i2s for audio playback */
	i2s_tx_cfg.word_size = AUDIO_SAMPLE_BIT_WIDTH;
	i2s_tx_cfg.channels = AUDIO_NUM_CHANNELS;
	i2s_tx_cfg.format = I2S_FMT_DATA_FORMAT_MASK;
	i2s_tx_cfg.options = I2S_OPT_FRAME_CLK_SLAVE;
	i2s_tx_cfg.frame_clk_freq = AUDIO_SAMPLE_FREQ;
	i2s_tx_cfg.block_size = AUDIO_FRAME_BUF_BYTES;
	i2s_tx_cfg.mem_slab = &i2s_tx_mem_slab;
	i2s_tx_cfg.timeout = K_FOREVER;
	ret = i2s_configure(host_i2s_tx_dev, I2S_DIR_TX, &i2s_tx_cfg);
	if (ret != 0) {
		printk("i2s_configure failed with %d error", ret);
		exit(-1);
	}
}


void main(void)
{

	k_sleep(K_SECONDS(5));
	init();
	
	/* start i2s rx driver */
	ret = i2s_trigger(host_i2s_rx_dev, I2S_DIR_RX, I2S_TRIGGER_START);
	if (ret != 0) {
		printk("i2s_trigger failed with %d error", ret);
		exit(-1);
	}

	/* start i2s rx driver */
	ret = i2s_trigger(host_i2s_tx_dev, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret != 0) {
		printk("i2s_trigger failed with %d error", ret);
		exit(-1);
	}

	/* receive data */
	void *mem_block;
	size_t size, tot_size = 0;
	while(true){
		for (int i = 0; i < I2S_PLAY_BUF_COUNT; i++) {
			ret = i2s_read(host_i2s_rx_dev, &mem_block, &size);
			memcpy(user_buffer + tot_size, mem_block, size);
			ret = i2s_write(host_i2s_tx_dev, user_buffer + tot_size, size);
			tot_size += size;
			k_mem_slab_free(&i2s_rx_mem_slab, &mem_block);
		}
		tot_size = 0;
	}

	// stop i2s transimsion
	ret = i2s_trigger(host_i2s_tx_dev, I2S_DIR_TX, I2S_TRIGGER_STOP);
	if (ret != 0) {
		printk("i2s_trigger failed with %d error", ret);
		exit(-1);
	}

	// stop i2s transimsion
	ret = i2s_trigger(host_i2s_rx_dev, I2S_DIR_RX, I2S_TRIGGER_STOP);
	if (ret != 0) {
		printk("i2s_trigger failed with %d error", ret);
		exit(-1);
	}
}


