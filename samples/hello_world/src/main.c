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
#define AUDIO_SAMPLES_PER_CH_PER_FRAME	(64)
#define AUDIO_NUM_CHANNELS		(2)
#define AUDIO_SAMPLES_PER_FRAME		\
	(AUDIO_SAMPLES_PER_CH_PER_FRAME * AUDIO_NUM_CHANNELS)
#define AUDIO_SAMPLE_BYTES		(4)
#define AUDIO_SAMPLE_BIT_WIDTH		(32)

#define AUDIO_FRAME_BUF_BYTES		\
	(AUDIO_SAMPLES_PER_FRAME * AUDIO_SAMPLE_BYTES)

#define I2S_PLAY_BUF_COUNT		(6)
#define I2S_TX_PRELOAD_BUF_COUNT	(2)

#define BASE_TONE_FREQ_HZ		1046.502 /* Hz */
#define FLOAT_VALUE_OF_2PI		(2 * 3.1415926535897932384626433832795)
#define BASE_TONE_FREQ_RAD		(BASE_TONE_FREQ_HZ * FLOAT_VALUE_OF_2PI)
#define BASE_TONE_PHASE_DELTA		(BASE_TONE_FREQ_RAD / AUDIO_SAMPLE_FREQ)

#define SECONDS_PER_TONE		(1) /* second(s) */
#define TONES_TO_PLAY			\
	{0, 2, 4, 5, 7, 9, 10, 12, 12, 10, 9, 7, 5, 4, 2, 0}

#define AUDIO_FRAMES_PER_SECOND		\
	(AUDIO_SAMPLE_FREQ / AUDIO_SAMPLES_PER_CH_PER_FRAME)
#define AUDIO_FRAMES_PER_TONE_DURATION	\
	(SECONDS_PER_TONE * AUDIO_FRAMES_PER_SECOND)

#define SIGNAL_AMPLITUDE_DBFS		(-36)
#define SIGNAL_AMPLITUDE_BITS		(31 + (SIGNAL_AMPLITUDE_DBFS / 6))
#define SIGNAL_AMPLITUDE_SCALE		(1 << SIGNAL_AMPLITUDE_BITS)

#ifdef AUDIO_PLAY_FROM_HOST
#define APP_MODE_STRING			"host playback"
#else
#define APP_MODE_STRING			"tone playback"
#endif
static struct k_mem_slab i2s_mem_slab;
static struct device *host_i2s_dev;
static char audio_buffers[AUDIO_FRAME_BUF_BYTES][I2S_PLAY_BUF_COUNT];
void main(void)
{
	printk("Hello World! %s\n", CONFIG_BOARD);

	struct i2s_config i2s_cfg;
    int ret;
	
    host_i2s_dev = device_get_binding("i2s_rx");

	k_mem_slab_init(&i2s_mem_slab, audio_buffers, AUDIO_FRAME_BUF_BYTES,
			I2S_PLAY_BUF_COUNT);
	if (!host_i2s_dev) {
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
	if (ret != 0) {
		printk("dmic_configure failed with %d error", ret);
		exit(-1);
	}
	ret = i2s_trigger(host_i2s_dev, I2S_DIR_RX, I2S_TRIGGER_START);
	if (ret != 0) {
		printk("dmic_trigger failed with %d error", ret);
		exit(-1);
	}

    void*in_buf;
    size_t size;
    while(true)
    {
        k_sleep(K_SECONDS(2));
        ret = i2s_read(host_i2s_dev,&in_buf, &size);
     
        if (ret != 0) {
            printk("i2s read failed with %d error", ret);
            exit(-1);
        }
    }
//	ret = i2s_trigger(host_i2s_dev, I2S_DIR_RX, I2S_TRIGGER_STOP);
//	if (ret != 0) {
//		printk("dmic_trigger failed with %d error", ret);
//		exit(-1);
//	}
}
