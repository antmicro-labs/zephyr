
/*
 * Copyright (c) 2018 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <drivers/i2s.h>
#include <soc.h>

#include "i2s_litex.h"
#include <logging/log.h>
LOG_MODULE_REGISTER(i2s_litex);
#define DEV_CFG(dev) \
	((struct mpxxdtyy_config *const)(dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct mpxxdtyy_data *const)(dev)->driver_data)

static int i2s_litex_initialize(struct device *dev)
{
	struct i2s_litex_data *const dev_data =(struct i2s_litex_data*const)dev->driver_data;

	k_sem_init(&dev_data->rx.sem, 0, 1);
	k_sem_init(&dev_data->tx.sem, 0, 1);

	LOG_INF("%s inited", dev->config->name);

	return 0;
}

static int i2s_litex_configure(struct device *dev, enum i2s_dir dir,
			       struct i2s_config *i2s_cfg)
{
//	const struct i2s_litex_cfg *const cfg = DEV_CFG(dev);
//	struct i2s_litex_data *const dev_data = DEV_DATA(dev);
//	struct stream *stream;
//	u32_t bit_clk_freq;
//	int ret;
//
//	if (dir == I2S_DIR_RX) {
//		stream = &dev_data->rx;
//	} else if (dir == I2S_DIR_TX) {
//		stream = &dev_data->tx;
//	} else {
//		LOG_ERR("Either RX or TX direction must be selected");
//		return -EINVAL;
//	}
//
//	if (stream->state != I2S_STATE_NOT_READY &&
//	    stream->state != I2S_STATE_READY) {
//		LOG_ERR("invalid state");
//		return -EINVAL;
//	}
//
//	stream->master = true;
//	if (i2s_cfg->options & I2S_OPT_FRAME_CLK_SLAVE ||
//	    i2s_cfg->options & I2S_OPT_BIT_CLK_SLAVE) {
//		stream->master = false;
//	}
//
//	if (i2s_cfg->frame_clk_freq == 0U) {
//		stream->queue_drop(stream);
//		memset(&stream->cfg, 0, sizeof(struct i2s_config));
//		stream->state = I2S_STATE_NOT_READY;
//		return 0;
//	}
//
//	memcpy(&stream->cfg, i2s_cfg, sizeof(struct i2s_config));
//
//	/* set I2S bitclock */
//	bit_clk_freq = i2s_cfg->frame_clk_freq *
//		       i2s_cfg->word_size * i2s_cfg->channels;
//
//	ret = i2s_litex_set_clock(dev, bit_clk_freq);
//	if (ret < 0) {
//		return ret;
//	}
//
//	/* set I2S Data Format */
//	if (i2s_cfg->word_size == 16U) {
//		LL_I2S_SetDataFormat(cfg->i2s, LL_I2S_DATAFORMAT_16B);
//	} else if (i2s_cfg->word_size == 24U) {
//		LL_I2S_SetDataFormat(cfg->i2s, LL_I2S_DATAFORMAT_24B);
//	} else if (i2s_cfg->word_size == 32U) {
//		LL_I2S_SetDataFormat(cfg->i2s, LL_I2S_DATAFORMAT_32B);
//	} else {
//		LOG_ERR("invalid word size");
//		return -EINVAL;
//	}
//
//	/* set I2S Standard */
//	switch (i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) {
//	case I2S_FMT_DATA_FORMAT_I2S:
//		LL_I2S_SetStandard(cfg->i2s, LL_I2S_STANDARD_PHILIPS);
//		break;
//
//	case I2S_FMT_DATA_FORMAT_PCM_SHORT:
//		LL_I2S_SetStandard(cfg->i2s, LL_I2S_STANDARD_PCM_SHORT);
//		break;
//
//	case I2S_FMT_DATA_FORMAT_PCM_LONG:
//		LL_I2S_SetStandard(cfg->i2s, LL_I2S_STANDARD_PCM_LONG);
//		break;
//
//	case I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:
//		LL_I2S_SetStandard(cfg->i2s, LL_I2S_STANDARD_MSB);
//		break;
//
//	case I2S_FMT_DATA_FORMAT_RIGHT_JUSTIFIED:
//		LL_I2S_SetStandard(cfg->i2s, LL_I2S_STANDARD_LSB);
//		break;
//
//	default:
//		LOG_ERR("Unsupported I2S data format");
//		return -EINVAL;
//	}
//
//	/* set I2S clock polarity */
//	if ((i2s_cfg->format & I2S_FMT_CLK_FORMAT_MASK) == I2S_FMT_BIT_CLK_INV)
//		LL_I2S_SetClockPolarity(cfg->i2s, LL_I2S_POLARITY_HIGH);
//	else
//		LL_I2S_SetClockPolarity(cfg->i2s, LL_I2S_POLARITY_LOW);
//
//	stream->state = I2S_STATE_READY;
	return 0;
}

static int i2s_litex_read(struct device *dev, void **mem_block, size_t *size)
{
//	struct i2s_litex_data *const dev_data = DEV_DATA(dev);
//	int ret;
//
//	if (dev_data->rx.state == I2S_STATE_NOT_READY) {
//		LOG_DBG("invalid state");
//		return -EIO;
//	}
//
//	if (dev_data->rx.state != I2S_STATE_ERROR) {
//		ret = k_sem_take(&dev_data->rx.sem, dev_data->rx.cfg.timeout);
//		if (ret < 0) {
//			return ret;
//		}
//	}
//
//	/* Get data from the beginning of RX queue */
//	ret = queue_get(&dev_data->rx.mem_block_queue, mem_block, size);
//	if (ret < 0) {
//		return -EIO;
//	}
//
	return 0;
}

static int i2s_litex_write(struct device *dev, void *mem_block, size_t size)
{
//	struct i2s_litex_data *const dev_data = DEV_DATA(dev);
//	int ret;
//
//	if (dev_data->tx.state != I2S_STATE_RUNNING &&
//	    dev_data->tx.state != I2S_STATE_READY) {
//		LOG_DBG("invalid state");
//		return -EIO;
//	}
//
//	ret = k_sem_take(&dev_data->tx.sem, dev_data->tx.cfg.timeout);
//	if (ret < 0) {
//		return ret;
//	}
//
//	/* Add data to the end of the TX queue */
//	queue_put(&dev_data->tx.mem_block_queue, mem_block, size);
//
	return 0;
}

static int i2s_litex_trigger(struct device *dev, enum i2s_dir dir,
			     enum i2s_trigger_cmd cmd)
{
//	struct i2s_litex_data *const dev_data = DEV_DATA(dev);
//	struct stream *stream;
//	unsigned int key;
//	int ret;
//
//	if (dir == I2S_DIR_RX) {
//		stream = &dev_data->rx;
//	} else if (dir == I2S_DIR_TX) {
//		stream = &dev_data->tx;
//	} else {
//		LOG_ERR("Either RX or TX direction must be selected");
//		return -EINVAL;
//	}
//
//	switch (cmd) {
//	case I2S_TRIGGER_START:
//		if (stream->state != I2S_STATE_READY) {
//			LOG_ERR("START trigger: invalid state %d",
//				    stream->state);
//			return -EIO;
//		}
//
//		__ASSERT_NO_MSG(stream->mem_block == NULL);
//
//		ret = stream->stream_start(stream, dev);
//		if (ret < 0) {
//			LOG_ERR("START trigger failed %d", ret);
//			return ret;
//		}
//
//		stream->state = I2S_STATE_RUNNING;
//		stream->last_block = false;
//		break;
//
//	case I2S_TRIGGER_STOP:
//		key = irq_lock();
//		if (stream->state != I2S_STATE_RUNNING) {
//			irq_unlock(key);
//			LOG_ERR("STOP trigger: invalid state");
//			return -EIO;
//		}
//		irq_unlock(key);
//		stream->stream_disable(stream, dev);
//		stream->queue_drop(stream);
//		stream->state = I2S_STATE_READY;
//		stream->last_block = true;
//		break;
//
//	case I2S_TRIGGER_DRAIN:
//		key = irq_lock();
//		if (stream->state != I2S_STATE_RUNNING) {
//			irq_unlock(key);
//			LOG_ERR("DRAIN trigger: invalid state");
//			return -EIO;
//		}
//		stream->stream_disable(stream, dev);
//		stream->queue_drop(stream);
//		stream->state = I2S_STATE_READY;
//		irq_unlock(key);
//		break;
//
//	case I2S_TRIGGER_DROP:
//		if (stream->state == I2S_STATE_NOT_READY) {
//			LOG_ERR("DROP trigger: invalid state");
//			return -EIO;
//		}
//		stream->stream_disable(stream, dev);
//		stream->queue_drop(stream);
//		stream->state = I2S_STATE_READY;
//		break;
//
//	case I2S_TRIGGER_PREPARE:
//		if (stream->state != I2S_STATE_ERROR) {
//			LOG_ERR("PREPARE trigger: invalid state");
//			return -EIO;
//		}
//		stream->state = I2S_STATE_READY;
//		stream->queue_drop(stream);
//		break;
//
//	default:
//		LOG_ERR("Unsupported trigger command");
//		return -EINVAL;
//	}
//
	return 0;
}
static const struct i2s_driver_api i2s_litex_driver_api = {
	.configure = i2s_litex_configure,
	.read = i2s_litex_read,
	.write = i2s_litex_write,
	.trigger = i2s_litex_trigger,
};

#define I2S_INIT(n)	\
	static struct i2s_litex_data i2s_litex_data_##n; \
    \
	static struct i2s_litex_cfg i2s_litex_cfg_##n = { \
		.base = DT_INST_##n##_LITEX_I2S_CONTROL_BASE_ADDRESS, \
		.fifo_base = DT_INST_##n##_LITEX_I2S_CONTROL_BASE_ADDRESS, \
	}; \
	DEVICE_AND_API_INIT(i2s_##n, \
			DT_INST_##n##_LITEX_I2S_LABEL, \
			i2s_litex_initialize, \
			&i2s_litex_data_##n, \
			&i2s_litex_cfg_##n, \
			POST_KERNEL, \
			CONFIG_I2S_INIT_PRIORITY, \
			&i2s_litex_driver_api)



I2S_INIT(0);
