/*
 * Copyright (c) 2020 Antmicro <www.antmicro.com>
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <drivers/i2s.h>
#include <soc.h>
#include <sys/util.h>
#include <assert.h>
#include "i2s_litex.h"
#include <logging/log.h>
LOG_MODULE_REGISTER(i2s_litex);
#define DEV_CFG(dev) ((struct i2s_litex_cfg *const)(dev)->config_info)
#define DEV_DATA(dev) ((struct i2s_litex_data *const)(dev)->driver_data)

#define MODULO_INC(val, max)                                                   \
	{                                                                      \
		val = (++val < max) ? val : 0;                                 \
	}

/**
 * @brief Enable i2s device
 *
 * @param reg base register of device 
 *
 * @return N/A
 */
static void i2s_enable(int reg)
{
	u8_t reg_data = litex_read8(reg + I2S_CONTROL_REG_OFFSET);

	litex_write8(reg_data | I2S_ENABLE, reg + I2S_CONTROL_REG_OFFSET);
}

/**
 * @brief Disable i2s device
 *
 * @param reg base register of device
 *
 * @return N/A
 */
static void i2s_disable(int reg)
{
	u8_t reg_data = litex_read8(reg + I2S_CONTROL_REG_OFFSET);

	litex_write8(reg_data & ~(I2S_ENABLE), reg + I2S_CONTROL_REG_OFFSET);
}

/**
 * @brief Reset i2s fifo 
 *
 * @param reg base register of device 
 *
 * @return N/A
 */
static void i2s_reset_fifo(int reg)
{
	u8_t reg_data = litex_read8(reg + I2S_CONTROL_REG_OFFSET);

	litex_write8(reg_data | I2S_FIFO_RESET, reg + I2S_CONTROL_REG_OFFSET);
}

/**
 * @brief Enable i2s interrupt in event register
 *
 * @param reg base register of device
 * @param irq_type irq type to be enabled one of I2S_EV_READY or I2S_EV_ERROR
 *
 * @return N/A
 */

static void i2s_irq_enable(int reg, int irq_type)
{
	assert(irq_type == I2S_EV_READY || irq_type == I2S_EV_ERROR);

	u8_t reg_data = litex_read8(reg + I2S_EV_ENABLE_REG_OFFSET);

	litex_write8(reg_data | irq_type, reg + I2S_EV_ENABLE_REG_OFFSET);
}

/**
 * @brief Disable i2s interrupt in event register
 *
 * @param reg base register of device
 * @param irq_type irq type to be disabled one of I2S_EV_READY or I2S_EV_ERROR
 *
 * @return N/A
 */
static void i2s_irq_disable(int reg, int irq_type)
{
	assert(irq_type == I2S_EV_READY || irq_type == I2S_EV_ERROR);

	u8_t reg_data = litex_read8(reg + I2S_EV_ENABLE_REG_OFFSET);

	litex_write8(reg_data & ~(irq_type), reg + I2S_EV_ENABLE_REG_OFFSET);
}

/**
 * @brief Clear all pending irqs 
 *
 * @param reg base register of device 
 *
 * @return N/A
 */
static void i2s_clear_pending_irq(int reg)
{
	u8_t reg_data = litex_read8(reg + I2S_EV_PENDING_REG_OFFSET);

	sys_write8(reg_data, reg + I2S_EV_PENDING_REG_OFFSET);
}

/**
 * @brief fast data copy function, 
 * each operation copies 32 bit data chunks
 * This function copies data from fifo into user buffer
 *
 * @param dst memory destination where fifo data will be copied to
 * @param size amount of data to be copied
 * @param sample_width width of signle sample in bits
 *
 * @return N/A
 */
static void i2s_copy_from_fifo(u8_t *dst, size_t size, int sample_width)
{
	u32_t data;
	int chan_size = sample_width / 8;
	int max_off = chan_size - 1;
	for (size_t i = 0; i < size; ++i) {
		data = sys_read32(I2S_RX_FIFO_ADDR +
				  i * CONFIG_I2S_LITEX_FIFO_WORD_SIZE);
		for (int off = max_off; off >= 0; off--) {
			*(dst + i * chan_size + off) = data >> 8 * off;
		}
	}
}

/**
 * @brief fast data copy function, 
 * each operation copies 32 bit data chunks
 * This function copies data from user buffer into fifo
 *
 * @param src memory from which data will be copied to fifo
 * @param size amount of data to be copied in bytes
 * @param sample_width width of signle sample in bits
 *
 * @return N/A
 */
static void i2s_copy_to_fifo(u8_t *src, size_t size, int sample_width)
{
	int chan_size = sample_width / 8;
	int max_off = chan_size - 1;
	u32_t data;
	u8_t *d_ptr = (u8_t *)&data;

	for (size_t i = 0; i < size / chan_size; ++i) {
		for (int off = max_off; off >= 0; off--) {
			*(d_ptr + off) = *(src + i * chan_size + off);
		}
		sys_write32(data, I2S_TX_FIFO_ADDR +
					  i * CONFIG_I2S_LITEX_FIFO_WORD_SIZE);
	}
}

/*
 * Get data from the queue
 */
static int queue_get(struct ring_buf *rb, void **mem_block, size_t *size)
{
	unsigned int key;

	key = irq_lock();

	if (rb->tail == rb->head) {
		/* Ring buffer is empty */
		irq_unlock(key);
		return -ENOMEM;
	}
	*mem_block = rb->buf[rb->tail].mem_block;
	*size = rb->buf[rb->tail].size;
	MODULO_INC(rb->tail, rb->len);

	irq_unlock(key);
	return 0;
}

/*
 * Put data in the queue
 */
static int queue_put(struct ring_buf *rb, void *mem_block, size_t size)
{
	u16_t head_next;
	unsigned int key;

	key = irq_lock();

	head_next = rb->head;
	MODULO_INC(head_next, rb->len);

	if (head_next == rb->tail) {
		/* Ring buffer is full */
		irq_unlock(key);
		return -ENOMEM;
	}

	rb->buf[rb->head].mem_block = mem_block;
	rb->buf[rb->head].size = size;
	rb->head = head_next;

	irq_unlock(key);
	return 0;
}

static int i2s_litex_initialize(struct device *dev)
{
	struct i2s_litex_cfg *cfg = DEV_CFG(dev);
	struct i2s_litex_data *const dev_data = DEV_DATA(dev);
	k_sem_init(&dev_data->rx.sem, 0, CONFIG_I2S_LITEX_RX_BLOCK_COUNT);
	k_sem_init(&dev_data->tx.sem, CONFIG_I2S_LITEX_TX_BLOCK_COUNT - 1,
		   CONFIG_I2S_LITEX_TX_BLOCK_COUNT);

	cfg->irq_config(dev);
	return 0;
}

static int i2s_litex_configure(struct device *dev, enum i2s_dir dir,
			       struct i2s_config *i2s_cfg)
{
	struct i2s_litex_data *const dev_data = DEV_DATA(dev);
	const struct i2s_litex_cfg *const cfg = DEV_CFG(dev);
	struct stream *stream;
	int channels_concatenated;
	if (dir == I2S_DIR_RX) {
		stream = &dev_data->rx;
		channels_concatenated = litex_read8(I2S_RX_STATUS_REG) &
					I2S_RX_STAT_CHANNEL_CONCATENATED_MASK;
	} else if (dir == I2S_DIR_TX) {
		stream = &dev_data->tx;
		channels_concatenated = litex_read8(I2S_TX_STATUS_REG) &
					I2S_TX_STAT_CHANNEL_CONCATENATED_MASK;
	} else {
		LOG_ERR("either RX or TX direction must be selected");
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
		LOG_ERR("invalid operating mode");
		return -EINVAL;
	}

	if (i2s_cfg->channels != 2) {
		LOG_ERR("invalid channels number");
		return -EINVAL;
	}

	int req_buf_s = cfg->fifo_depth * (i2s_cfg->word_size / 8);
	if (i2s_cfg->block_size < req_buf_s) {
		LOG_ERR("not enough space to allocate signle buffer");
		LOG_ERR("fifo requires at least %i bytes", req_buf_s);
		return -EINVAL;
	} else if (i2s_cfg->block_size != req_buf_s) {
		LOG_WRN("the buffer is greater than required, only %i bytes of data are valid",
			req_buf_s);
		i2s_cfg->block_size = req_buf_s;
	}
	/* set I2S Data Format */
	if (i2s_cfg->word_size != 8U && i2s_cfg->word_size != 16U &&
	    i2s_cfg->word_size != 24U && i2s_cfg->word_size != 32U) {
		LOG_ERR("invalid word size");
		return -EINVAL;
	}
	uint8_t format_mask =
		I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED | I2S_FMT_DATA_FORMAT_MASK;
	/* set I2S Standard */
	if ((i2s_cfg->format & format_mask) == 0) {
		LOG_ERR("unsupported I2S data format");
		return -EINVAL;
	}

	if (channels_concatenated) {
		if(i2s_cfg->word_size>16)
		{
			LOG_ERR("can't concatanate channels greater than 16 bit");
			return -EINVAL;
		}
		// if channels are concatenated
		// we can always copy 32 bits
		i2s_cfg->word_size = 32;
	}

	memcpy(&stream->cfg, i2s_cfg, sizeof(struct i2s_config));
	stream->state = I2S_STATE_READY;

	return 0;
}

static int i2s_litex_read(struct device *dev, void **mem_block, size_t *size)
{
	struct i2s_litex_data *const dev_data = DEV_DATA(dev);
	int ret;
	if (dev_data->rx.state == I2S_STATE_NOT_READY) {
		LOG_DBG("invalid state");
		return -ENOMEM;
	}
	// just to implement tiemout
	ret = k_sem_take(&dev_data->rx.sem,
			 SYS_TIMEOUT_MS(dev_data->rx.cfg.timeout));
	if (ret < 0) {
		return ret;
	}
	/* Get data from the beginning of RX queue */
	return queue_get(&dev_data->rx.mem_block_queue, mem_block, size);
}

static int i2s_litex_write(struct device *dev, void *mem_block, size_t size)
{
	struct i2s_litex_data *const dev_data = DEV_DATA(dev);
	int ret;

	if (dev_data->tx.state != I2S_STATE_RUNNING &&
	    dev_data->tx.state != I2S_STATE_READY) {
		LOG_DBG("invalid state");
		return -EIO;
	}
	// just to implement tiemout
	ret = k_sem_take(&dev_data->tx.sem,
			 SYS_TIMEOUT_MS(dev_data->tx.cfg.timeout));
	if (ret < 0) {
		return ret;
	}
	/* Add data to the end of the TX queue */
	return queue_put(&dev_data->tx.mem_block_queue, mem_block, size);
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
		LOG_ERR("either RX or TX direction must be selected");
		return -EINVAL;
	}

	switch (cmd) {
	case I2S_TRIGGER_START:
		if (stream->state != I2S_STATE_READY) {
			LOG_ERR("START trigger: invalid state %d",
				stream->state);
			return -EIO;
		}
		__ASSERT_NO_MSG(stream->mem_block == NULL);
		i2s_reset_fifo(cfg->base);
		// when using tx module
		// write some init data to fifo
		if (dir == I2S_DIR_TX) {
			memset(((void *)I2S_TX_FIFO_ADDR), 0xff,
			       CONFIG_I2S_LITEX_FIFO_MAX_DEPTH *
				       CONFIG_I2S_LITEX_FIFO_WORD_SIZE);
		}
		i2s_enable(cfg->base);
		i2s_irq_enable(cfg->base, I2S_EV_READY);
		stream->state = I2S_STATE_RUNNING;
		break;

	case I2S_TRIGGER_STOP:
		if (stream->state != I2S_STATE_RUNNING) {
			LOG_ERR("STOP trigger: invalid state");
			return -EIO;
		}
		i2s_disable(cfg->base);
		i2s_irq_disable(cfg->base, I2S_EV_READY);
		stream->state = I2S_STATE_READY;
		break;

	default:
		LOG_ERR("unsupported trigger command");
		return -EINVAL;
	}
	return 0;
}

static inline void clear_rx_fifo(const struct i2s_litex_cfg *cfg)
{
	for (int i = 0; i < cfg->fifo_depth; i++) {
		sys_read32(I2S_RX_FIFO_ADDR +
			   i * CONFIG_I2S_LITEX_FIFO_WORD_SIZE);
	}
	i2s_clear_pending_irq(cfg->base);
}

static void i2s_litex_isr_rx(void *arg)
{
	struct device *const dev = (struct device *)arg;
	const struct i2s_litex_cfg *cfg = DEV_CFG(dev);
	struct stream *stream = &DEV_DATA(dev)->rx;
	int ret;

	/* Prepare to receive the next data block */
	ret = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->mem_block,
			       K_NO_WAIT);
	if (ret < 0) {
		clear_rx_fifo(cfg);
		return;
	}
	i2s_copy_from_fifo((u8_t *)stream->mem_block, cfg->fifo_depth,
			   stream->cfg.word_size);
	i2s_clear_pending_irq(cfg->base);

	ret = queue_put(&stream->mem_block_queue, stream->mem_block,
			stream->cfg.block_size);
	if (ret < 0) {
		LOG_WRN("Couldn't copy data from RX fifo to the ring buffer (no space left) - dropping a frame");
		return;
	}

	k_sem_give(&stream->sem);
}

static inline void fill_tx_fifo(const struct i2s_litex_cfg *cfg)
{
	for (int i = 0; i < cfg->fifo_depth; i++) {
		sys_write32(0x0, I2S_TX_FIFO_ADDR +
					 i * CONFIG_I2S_LITEX_FIFO_WORD_SIZE);
	}
	i2s_clear_pending_irq(cfg->base);
}

static void i2s_litex_isr_tx(void *arg)
{
	struct device *const dev = (struct device *)arg;
	const struct i2s_litex_cfg *cfg = DEV_CFG(dev);
	size_t mem_block_size;
	struct stream *stream = &DEV_DATA(dev)->tx;
	int ret;
	ret = queue_get(&stream->mem_block_queue, &stream->mem_block,
			&mem_block_size);
	if (ret < 0) {
		fill_tx_fifo(cfg);
		return;
	}
	k_sem_give(&stream->sem);
	i2s_copy_to_fifo((u8_t *)stream->mem_block, mem_block_size,
			 stream->cfg.word_size);
	i2s_clear_pending_irq(cfg->base);

	k_mem_slab_free(stream->cfg.mem_slab, &stream->mem_block);
}

static const struct i2s_driver_api i2s_litex_driver_api = {
	.configure = i2s_litex_configure,
	.read = i2s_litex_read,
	.write = i2s_litex_write,
	.trigger = i2s_litex_trigger,
};

#define I2S_INIT(dir)                                                          \
                                                                               \
	static struct queue_item rx_ring_buf[CONFIG_I2S_LITEX_RX_BLOCK_COUNT]; \
	static struct queue_item tx_ring_buf[CONFIG_I2S_LITEX_TX_BLOCK_COUNT]; \
                                                                               \
	static struct i2s_litex_data i2s_litex_data_##dir = {                  \
		.dir.mem_block_queue.buf = dir##_ring_buf,                     \
		.dir.mem_block_queue.len =                                     \
			sizeof(dir##_ring_buf) / sizeof(struct queue_item),    \
	};                                                                     \
                                                                               \
	static void i2s_litex_irq_config_func_##dir(struct device *dev);       \
                                                                               \
	static struct i2s_litex_cfg i2s_litex_cfg_##dir = {                    \
		.base = DT_REG_ADDR_BY_NAME(DT_NODELABEL(i2s_##dir), control), \
		.fifo_base =                                                   \
			DT_REG_ADDR_BY_NAME(DT_NODELABEL(i2s_##dir), fifo),    \
		.fifo_depth = DT_PROP(DT_NODELABEL(i2s_##dir), fifo_depth),    \
		.irq_config = i2s_litex_irq_config_func_##dir                  \
	};                                                                     \
	DEVICE_AND_API_INIT(i2s_##dir,                                         \
			    DT_PROP(DT_NODELABEL(i2s_##dir), label),           \
			    i2s_litex_initialize, &i2s_litex_data_##dir,       \
			    &i2s_litex_cfg_##dir, POST_KERNEL,                 \
			    CONFIG_I2S_INIT_PRIORITY, &i2s_litex_driver_api);  \
                                                                               \
	static void i2s_litex_irq_config_func_##dir(struct device *dev)        \
	{                                                                      \
		IRQ_CONNECT(DT_IRQN(DT_NODELABEL(i2s_##dir)),                  \
			    DT_IRQ(DT_NODELABEL(i2s_##dir), priority),         \
			    i2s_litex_isr_##dir, DEVICE_GET(i2s_##dir), 0);    \
		irq_enable(DT_IRQN(DT_NODELABEL(i2s_##dir)));                  \
	}

#if DT_HAS_NODE_STATUS_OKAY(DT_NODELABEL(i2s_rx))
I2S_INIT(rx);
#endif
#if DT_HAS_NODE_STATUS_OKAY(DT_NODELABEL(i2s_tx))
I2S_INIT(tx);
#endif
