/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/i2s.h>
#include <stdlib.h>
#include <logging/log.h>
#define AUDIO_SAMPLE_FREQ		(48000)
#define AUDIO_SAMPLES_PER_CH_PER_FRAME	(128)
#define AUDIO_NUM_CHANNELS		(2)
#define AUDIO_SAMPLES_PER_FRAME		\
	(AUDIO_SAMPLES_PER_CH_PER_FRAME * AUDIO_NUM_CHANNELS)
#define AUDIO_SAMPLE_BYTES		(4)
#define AUDIO_SAMPLE_BIT_WIDTH		(32)

#define AUDIO_FRAME_BUF_BYTES		\
	(AUDIO_SAMPLES_PER_FRAME * AUDIO_SAMPLE_BYTES)

#define I2S_PLAY_BUF_COUNT		(6)
#define I2S_TX_PRELOAD_BUF_COUNT	(2)


static struct k_mem_slab i2s_rx_mem_slab;
static struct k_mem_slab i2s_tx_mem_slab;
static struct device *i2s_rx_dev;
static struct device *i2s_tx_dev;
static char __aligned(32) audio_buffers_rx[AUDIO_FRAME_BUF_BYTES][I2S_PLAY_BUF_COUNT];
static char __aligned(32) audio_buffers_tx[AUDIO_FRAME_BUF_BYTES][I2S_PLAY_BUF_COUNT];
void main(void)
{
	printk("Hello World! %s\n", CONFIG_BOARD);

	struct i2s_config i2s_cfg_rx;
	struct i2s_config i2s_cfg_tx;

    int ret;
	
    i2s_rx_dev = device_get_binding("i2s_rx");
    i2s_tx_dev = device_get_binding("i2s_tx");

	if (!i2s_rx_dev) 
    {
		printk("unable to find i2s_rx device");
	    exit(-1);
	}

	if (!i2s_tx_dev) 
    {
		printk("unable to find i2s_tx device");
	    exit(-1);
	}

	k_mem_slab_init(&i2s_rx_mem_slab, audio_buffers_rx, AUDIO_FRAME_BUF_BYTES,
			I2S_PLAY_BUF_COUNT);
	k_mem_slab_init(&i2s_tx_mem_slab, audio_buffers_tx, AUDIO_FRAME_BUF_BYTES,
			I2S_PLAY_BUF_COUNT);

	/* configure i2s rx for audio playback */
	i2s_cfg_rx.word_size = AUDIO_SAMPLE_BIT_WIDTH;
	i2s_cfg_rx.channels = AUDIO_NUM_CHANNELS;
	i2s_cfg_rx.format = I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED | I2S_FMT_CLK_NF_NB;
	i2s_cfg_rx.options = I2S_OPT_FRAME_CLK_SLAVE |
		I2S_OPT_BIT_CLK_SLAVE;
	i2s_cfg_rx.frame_clk_freq = AUDIO_SAMPLE_FREQ;
	i2s_cfg_rx.block_size = AUDIO_FRAME_BUF_BYTES;
	i2s_cfg_rx.mem_slab = &i2s_rx_mem_slab;
	/* make the receive interface blocking */
	i2s_cfg_rx.timeout = K_FOREVER;
	ret = i2s_configure(i2s_rx_dev, I2S_DIR_RX, &i2s_cfg_rx);

	if (ret != 0)
    {
		printk("dmic_configure failed with %d error", ret);
		exit(-1);
	}
	/* configure i2s tx for audio playback */
	i2s_cfg_tx.word_size = AUDIO_SAMPLE_BIT_WIDTH;
	i2s_cfg_tx.channels = AUDIO_NUM_CHANNELS;
	i2s_cfg_tx.format = I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED | I2S_FMT_CLK_NF_NB;
	i2s_cfg_tx.options = I2S_OPT_FRAME_CLK_SLAVE |
		I2S_OPT_BIT_CLK_SLAVE;
	i2s_cfg_tx.frame_clk_freq = AUDIO_SAMPLE_FREQ;
	i2s_cfg_tx.block_size = AUDIO_FRAME_BUF_BYTES;
	i2s_cfg_tx.mem_slab = &i2s_tx_mem_slab;
	/* make the receive interface blocking */
	i2s_cfg_tx.timeout = K_FOREVER;
	ret = i2s_configure(i2s_tx_dev, I2S_DIR_TX, &i2s_cfg_tx);

	if (ret != 0)
    {
		printk("dmic_configure failed with %d error", ret);
		exit(-1);
	}


	ret = i2s_trigger(i2s_rx_dev, I2S_DIR_RX, I2S_TRIGGER_START);
	if (ret != 0) {
		printk("dmic_trigger failed with %d error", ret);
		exit(-1);
	}

	ret = i2s_trigger(i2s_tx_dev, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret != 0) {
		printk("dmic_trigger failed with %d error", ret);
		exit(-1);
	}
    sys_write8(0xf, 0x82006000);
    
    void*in_buf;
    size_t size;
    while(true)
    {
        ret = i2s_read(i2s_rx_dev,&in_buf, &size);
        printk("i2s_read stat: %i\n", ret);
        printk("some bytes:%x \n", *((u32_t*)in_buf)+1);

        if (ret != 0) {
            printk("i2s timeout.\n");
            k_sleep(K_SECONDS(2));
            exit(-1);
        }
        k_mem_slab_free(&i2s_rx_mem_slab, &in_buf);
    }

//	ret = i2s_trigger(i2s_rx_dev, I2S_DIR_RX, I2S_TRIGGER_STOP);
//	if (ret != 0) {
//		printk("dmic_trigger failed with %d error", ret);
//		exit(-1);
//	}
}
