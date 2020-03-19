
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
	((struct i2s_litex_cfg *const)(dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct i2s_litex_data*const)(dev)->driver_data)

static int i2s_litex_initialize(struct device *dev)
{
	struct i2s_litex_data *const dev_data =DEV_DATA(dev);

	k_sem_init(&dev_data->rx.sem, 0, 1);
	k_sem_init(&dev_data->tx.sem, 0, 1);

	LOG_INF("%s inited", dev->config->name);

	return 0;
}

static int i2s_litex_configure(struct device *dev, enum i2s_dir dir,
			       struct i2s_config *i2s_cfg)
{

	LOG_INF("i2s conigure function invoked");
	struct i2s_litex_data *const dev_data = DEV_DATA(dev);
    struct stream *stream;

	if (dir == I2S_DIR_RX) {
		stream = &dev_data->rx;
	} else if (dir == I2S_DIR_TX) {
		stream = &dev_data->tx;
	} else {
		LOG_ERR("Either RX or TX direction must be selected");
		return -EINVAL;
	}

	if (stream->state != I2S_STATE_NOT_READY &&
	    stream->state != I2S_STATE_READY) {
		LOG_ERR("invalid state");
		return -EINVAL;
	}
    
	if (i2s_cfg->options & I2S_OPT_FRAME_CLK_MASTER ||
	    i2s_cfg->options & I2S_OPT_BIT_CLK_MASTER ||
        i2s_cfg->options & I2S_OPT_BIT_CLK_GATED) {
		LOG_ERR("invalid operating mode.");
		return -EINVAL;
	}

    if(i2s_cfg->channels != 2)
    {
        LOG_ERR("invalid channel width");
        return -EINVAL;
    }

	if (i2s_cfg->frame_clk_freq == 0U) {
		memset(&stream->cfg, 0, sizeof(struct i2s_config));
		stream->state = I2S_STATE_NOT_READY;
		return 0;
	}

	/* set I2S Data Format */
	if (i2s_cfg->word_size != 32U) {
		LOG_ERR("invalid word size.");
		return -EINVAL;
    }

	/* set I2S Standard */
	if((i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) 
            != I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED) {
		LOG_ERR("unsupported I2S data format");
		return -EINVAL;
	}

    if(i2s_cfg->timeout != K_FOREVER)
    {
        LOG_ERR("driver supports only polling mode");
        return -EINVAL;
    }

	memcpy(&stream->cfg, i2s_cfg, sizeof(struct i2s_config));
	stream->state = I2S_STATE_READY;
    LOG_INF("I2S CONFIGURATION DONE");
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
	struct i2s_litex_data *const dev_data = DEV_DATA(dev);
	const struct i2s_litex_cfg *const cfg = DEV_CFG(dev);
	struct stream *stream;

	if (dir == I2S_DIR_RX) {
		stream = &dev_data->rx;
	} else if (dir == I2S_DIR_TX) {
		stream = &dev_data->tx;
	} else {
		LOG_ERR("Either RX or TX direction must be selected");
		return -EINVAL;
	}

	switch (cmd) {
	case I2S_TRIGGER_START:
		if (stream->state != I2S_STATE_READY) {
			LOG_ERR("START trigger: invalid state %d",
				    stream->state);
			return -EIO;
		}

        LOG_INF("Enabling i2s under %x", cfg->base + I2S_CONTROL_REG_OFFSET);
        litex_write8(I2S_ENABLE | I2S_FIFO_RESET,
                    cfg->base + I2S_CONTROL_REG_OFFSET);
        LOG_INF("Reading i2s %x value should be %x", litex_read8(cfg->base + I2S_CONTROL_REG_OFFSET),I2S_ENABLE | I2S_FIFO_RESET);
		__ASSERT_NO_MSG(stream->mem_block == NULL);

		stream->state = I2S_STATE_RUNNING;
		break;
	case I2S_TRIGGER_STOP:
		if (stream->state != I2S_STATE_RUNNING) {
			LOG_ERR("STOP trigger: invalid state");
			return -EIO;
		}
        LOG_INF("Disabling i2s under %x", cfg->base + I2S_CONTROL_REG_OFFSET);
        litex_write8(I2S_DISABLE | I2S_FIFO_RESET,
                    cfg->base + I2S_CONTROL_REG_OFFSET);
        LOG_INF("Reading i2s %x value should be %x", litex_read8(cfg->base + I2S_CONTROL_REG_OFFSET),I2S_DISABLE | I2S_FIFO_RESET);
//      stream->queue_drop(stream);
		stream->state = I2S_STATE_READY;
		break;

	case I2S_TRIGGER_DRAIN:
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
    case I2S_TRIGGER_DROP:
//		if (stream->state == I2S_STATE_NOT_READY) {
//			LOG_ERR("DROP trigger: invalid state");
//			return -EIO;
//		}
//		stream->stream_disable(stream, dev);
//		stream->queue_drop(stream);
//		stream->state = I2S_STATE_READY;
//		break;
//
	case I2S_TRIGGER_PREPARE:
//		if (stream->state != I2S_STATE_ERROR) {
//			LOG_ERR("PREPARE trigger: invalid state");
//			return -EIO;
//		}
//		stream->state = I2S_STATE_READY;
//		stream->queue_drop(stream);
//		break;
//
	default:
		LOG_ERR("Unsupported trigger command");
		return -EINVAL;
	}

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
