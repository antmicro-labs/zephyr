/*
 * Copyright (c) 2020 Antmicro
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <drivers/i2s.h>
#include <soc.h>
#include <sys/util.h>

#include "i2s_litex.h"
#include <logging/log.h>
LOG_MODULE_REGISTER(i2s_litex);
#define DEV_CFG(dev) \
	((struct i2s_litex_cfg *const)(dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct i2s_litex_data*const)(dev)->driver_data)

#define MODULO_INC(val, max) { val = (++val < max) ? val : 0; }
#define CONFIG_I2S_BLOCK_COUNT 5
/**
 * @brief Enable RX device
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
 * @brief Disable RX device
 *
 * @param reg base register of device
 *
 * @return N/A
 */
static void i2s_disable(int reg)
{
	u8_t reg_data = litex_read8(reg + I2S_CONTROL_REG_OFFSET);

	litex_write8(reg_data  & ~(I2S_ENABLE), reg + I2S_CONTROL_REG_OFFSET);
}

/**
 * @brief Enable RX device
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
 * @brief Enable RX interrupt in event register
 *
 * @param reg base register of device
 *
 * @param irq_type irq type to be enabled eg. XX_READY or XX_ERROR
 *
 * @return N/A
 */

static void i2s_irq_enable(int reg, int irq_type)
{
	u8_t reg_data = litex_read8(reg + I2S_EV_ENABLE_REG_OFFSET);

	litex_write8(reg_data | irq_type , reg + I2S_EV_ENABLE_REG_OFFSET);
}

/**
 * @brief Disable RX interrupt in event register
 *
 * @param reg base register of device
 *
 * @param irq_type irq type to be disabled eg. XX_READY or XX_ERROR
 *
 * @return N/A
 */
static void i2s_irq_disable(int reg, int irq_type)
{
	u8_t reg_data = litex_read8(reg + I2S_EV_ENABLE_REG_OFFSET);

	litex_write8(reg_data & ~(irq_type), reg + I2S_EV_ENABLE_REG_OFFSET);
}

/**
 * @briefClear all pending irq 
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
 * @brief Get i2s overflow status 
 *
 * @param reg base register of device
 *
 * @return bool true if overflow occured, false otherwise
 */
static bool i2s_is_overflow(int reg)
{
    u32_t status_reg = litex_read32(reg + I2S_STATUS_REG_OFFSET);
    
    return (status_reg & I2S_STAT_OVERFLOW_MASK);
}

/**
 * @brief Get i2s underflow status 
 *
 * @param reg base register of device
 *
 * @return bool true if underflow occured, false otherwise
 */
static bool i2s_is_underflow(int reg)
{
    u32_t status_reg = litex_read32(reg + I2S_STATUS_REG_OFFSET);
    
    return (status_reg & I2S_STAT_UNDERFLOW_MASK);
}

/**
 * @brief Get i2s dataready status 
 *
 * @param reg base register of device
 *
 * @return bool true if data is ready to read, false otherwise
 */
static bool i2s_rx_is_dataready(int reg)
{
    u32_t status_reg = litex_read32(reg + I2S_STATUS_REG_OFFSET);
    
    return (status_reg & I2S_RX_STAT_DATAREADY_MASK);
}

/**
 * @brief Get i2s empty status 
 *
 * @param reg base register of device
 *
 * @return bool true if fifo empty false otherwise
 */
static bool i2s_rx_is_empty(int reg)
{
    u32_t status_reg = litex_read32(reg + I2S_STATUS_REG_OFFSET);
    
    return (status_reg & I2S_RX_STAT_EMPTY_MASK);
}

/**
 * @brief Get i2s dataready status 
 *
 * @param reg base register of device
 *
 * @return bool true if data is ready to read, false otherwise
 */
static bool i2s_tx_is_full(int reg)
{
    u32_t status_reg = litex_read32(reg + I2S_STATUS_REG_OFFSET);
    
    return (status_reg & I2S_TX_STAT_FULL_MASK);
}

/**
 * @brief Get i2s empty status 
 *
 * @param reg base register of device
 *
 * @return bool true if fifo empty false otherwise
 */
static bool i2s_tx_is_almostfull(int reg)
{
    u32_t status_reg = litex_read32(reg + I2S_STATUS_REG_OFFSET);
    
    return (status_reg & I2S_TX_STAT_ALMOSTFULL_MASK);
}

/**
 * @brief Get i2s wrcount info 
 *
 * @param reg base register of device
 *
 * @return u32_t return amount of data ready to write
 */
static u32_t i2s_get_wrcount(int reg, int off, int mask)
{
    u32_t status_reg = litex_read32(reg + I2S_STATUS_REG_OFFSET);
    
    return (status_reg & mask) >> off;
}

/**
 * @brief Get i2s rdcount info 
 *
 * @param reg base register of device
 *
 * @return u32_t return amount of data ready to read
 */
static u32_t i2s_get_rdcount(int reg, int off, int mask)
{
    u32_t status_reg = litex_read32(reg + I2S_STATUS_REG_OFFSET);
    
    return (status_reg & mask) >> off;
}
/**
 * @brief Return FIFO depth defined by i2s driver 
 *
 * @param dev base register of device
 *
 * @return int fifo_depth
 */
static int i2s_rx_get_fifo_depth(int reg)
{
    u32_t status_reg = litex_read32(reg + I2S_STATUS_REG_OFFSET);
    
    return ((status_reg & I2S_RX_STAT_FIFO_DEPTH_MASK) >> I2S_RX_STAT_FIFO_DEPTH_OFFSET);
}

/**
 * @brief fast data copy function, 
 * each operation copies 32 bit data chunks
 * This function copies data from fifo into user buffer
 *
 * @param dst memory destination where fifo data will be copied to
 * @param size amount of data to be copied
 *
 * @return N/A
 */
static void i2s_copy_from_fifo(u32_t *dst, size_t size)
{
    for(size_t i =0; i < size; ++i)
    {
       *(dst+i) = sys_read32(I2S_RX_FIFO_ADDR + i*FIFO_WORD_SIZE);
    }
}

/**
 * @brief fast data copy function, 
 * each operation copies 32 bit data chunks
 * This function copies data from user buffer into fifo
 *
 * @param src memory from which data will be copied to fifo.
 * @param size amount of data to be copied
 *
 * @return N/A
 */
static void i2s_copy_to_fifo(u32_t *src, size_t size)
{
    for(size_t i =0; i < size; ++i)
    {
        sys_write32(*(src+i),I2S_TX_FIFO_ADDR + i*FIFO_WORD_SIZE);
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

static void debug_registers(int reg)
{
    LOG_INF("Reading i2s CTR 0x%x", litex_read8(reg + I2S_CONTROL_REG_OFFSET));
    LOG_INF("Reading i2s EV_PE 0x%x",litex_read8(reg + I2S_EV_PENDING_REG_OFFSET));
    LOG_INF("Overflow status 0x%x", i2s_is_overflow(reg));
    LOG_INF("Underflow status 0x%x", i2s_is_underflow(reg));
    LOG_INF("Is empty 0x%x", i2s_rx_is_empty(reg));
    LOG_INF("Is ready 0x%x", i2s_rx_is_dataready(reg));
    LOG_INF("Write count 0x%x", i2s_get_wrcount(reg, I2S_RX_STAT_WRCOUNT_OFFSET, I2S_RX_STAT_WRCOUNT_MASK));
    LOG_INF("Read count 0x%x",i2s_get_rdcount(reg, I2S_RX_STAT_RDCOUNT_OFFSET, I2S_RX_STAT_RDCOUNT_MASK) );
    LOG_INF("fifo size 0x%x",i2s_rx_get_fifo_depth(reg));
}

static int i2s_litex_initialize(struct device *dev)
{
	struct i2s_litex_cfg *cfg = DEV_CFG(dev);
	struct i2s_litex_data *const dev_data =DEV_DATA(dev);
    cfg->irq_config(dev);
	k_sem_init(&dev_data->rx.sem, 0, CONFIG_I2S_BLOCK_COUNT);
	k_sem_init(&dev_data->tx.sem, 0, CONFIG_I2S_BLOCK_COUNT);

	LOG_INF("%s inited %x", dev->config->name, cfg->fifo_depth);

	return 0;
}

static int i2s_litex_configure(struct device *dev, enum i2s_dir dir,
			       struct i2s_config *i2s_cfg)
{

	LOG_INF("i2s conigure function invoked");
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

    if(i2s_cfg->block_size < cfg->fifo_depth*FIFO_WORD_SIZE)
    {
        LOG_ERR("not enought space to allocate signle buffer");
        LOG_ERR("Fifo requires at least %i bytes", cfg->fifo_depth*FIFO_WORD_SIZE);
        return -EINVAL;
    }else if(i2s_cfg->block_size != cfg->fifo_depth*FIFO_WORD_SIZE)
    {
        LOG_INF("caution, you buffer is greater than required. Only %i bytes of buffer will be filled by driver", cfg->fifo_depth*FIFO_WORD_SIZE);
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

	memcpy(&stream->cfg, i2s_cfg, sizeof(struct i2s_config));
	stream->state = I2S_STATE_READY;
    LOG_INF("I2S CONFIGURATION DONE");

	return 0;
}

static int i2s_litex_read(struct device *dev, void **mem_block, size_t *size)
{
	struct i2s_litex_data *const dev_data = DEV_DATA(dev);
	//const struct i2s_litex_cfg *const cfg = DEV_CFG(dev);
    int ret;
	if (dev_data->rx.state == I2S_STATE_NOT_READY) {
		LOG_DBG("invalid state");
		return -ENOMEM;
	}

   	if (dev_data->rx.state != I2S_STATE_ERROR) {
   		ret = k_sem_take(&dev_data->rx.sem, dev_data->rx.cfg.timeout);
   		if (ret < 0) {
   			return ret;
   		}
   	}
	/* Get data from the beginning of RX queue */
	ret = queue_get(&dev_data->rx.mem_block_queue, mem_block, size);
	if (ret < 0) {
		return -ENOMEM;
	}
	return 0;
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

	ret = k_sem_take(&dev_data->tx.sem, dev_data->tx.cfg.timeout);
	if (ret < 0) {
		return ret;
	}
	/* Add data to the end of the TX queue */
	queue_put(&dev_data->tx.mem_block_queue, mem_block, size);

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

	switch (cmd) 
    {
	case I2S_TRIGGER_START:
		if (stream->state != I2S_STATE_READY) {
			LOG_ERR("START trigger: invalid state %d",
				    stream->state);
			return -EIO;
		}

		__ASSERT_NO_MSG(stream->mem_block == NULL);
        LOG_INF("Enabling i2s under %x", cfg->base + I2S_CONTROL_REG_OFFSET);
        i2s_reset_fifo(cfg->base);
        // time dependency, rx registers are set to initial value
        // after zephyr start.
        // waiting for a while helps
       // while(litex_read8(cfg->base + I2S_CONTROL_REG_OFFSET) == I2S_FIFO_RESET)
       // {
       //    k_sleep(1);
       // }
        if(dir == I2S_DIR_TX)
        {
            memset(((void*)cfg->fifo_base),0xff,cfg->fifo_depth*FIFO_WORD_SIZE);
        }
        i2s_enable(cfg->base);
        i2s_irq_enable(cfg->base, I2S_EV_READY);     
        i2s_clear_pending_irq(cfg->base);
        stream->state = I2S_STATE_RUNNING;
		break;
	case I2S_TRIGGER_STOP:
		if (stream->state != I2S_STATE_RUNNING) {
			LOG_ERR("STOP trigger: invalid state");
			return -EIO;
		}
        LOG_INF("Disabling i2s under %x", cfg->base + I2S_CONTROL_REG_OFFSET);
        i2s_disable(cfg->base);
        i2s_irq_disable(cfg->base, I2S_EV_READY);       
		stream->state = I2S_STATE_READY;
		break;

	default:
		LOG_ERR("Unsupported trigger command");
		return -EINVAL;
	}

	return 0;
}

static void i2s_litex_isr_rx(void * arg)
{
	struct device *const dev = (struct device *) arg;
	const struct i2s_litex_cfg *cfg = DEV_CFG(dev);
	struct i2s_litex_data *const dev_data = DEV_DATA(dev);
	struct stream *stream = &dev_data->rx;
	int ret;
	/* Prepare to receive the next data block */
	ret = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->mem_block,
			       K_NO_WAIT);
	if (ret < 0) {
		stream->state = I2S_STATE_ERROR;
		return;
	}
    i2s_copy_from_fifo((u32_t*)stream->mem_block, cfg->fifo_depth);
    i2s_clear_pending_irq(cfg->base);
	ret = queue_put(&stream->mem_block_queue, stream->mem_block,
			stream->cfg.block_size);
	
    if (ret < 0) {
		stream->state = I2S_STATE_ERROR;
		return;
	}
	k_sem_give(&stream->sem);
}

static void i2s_litex_isr_tx(void * arg)
{
	struct device *const dev = (struct device *) arg;
	const struct i2s_litex_cfg *cfg = DEV_CFG(dev);
	size_t mem_block_size;
	struct stream *stream = &DEV_DATA(dev)->tx;
	int ret;
    // here should be some code which prevents
    // irq from stalling processor
	ret = queue_get(&stream->mem_block_queue, &stream->mem_block,
			&mem_block_size);
	if (ret < 0) {
        // this is temporary solution
        // maybe tx should be realized in polling mode only?
        memset(((void*)I2S_TX_FIFO_ADDR),0xff,I2S_TX_FIFO_DEPTH*FIFO_WORD_SIZE);
        i2s_clear_pending_irq(cfg->base);
		return;
	}
	k_sem_give(&stream->sem);

    i2s_copy_to_fifo((u32_t*)stream->mem_block, cfg->fifo_depth);
    i2s_clear_pending_irq(cfg->base);
}

static const struct i2s_driver_api i2s_litex_driver_api = {
	.configure = i2s_litex_configure,
	.read = i2s_litex_read,
	.write = i2s_litex_write,
	.trigger = i2s_litex_trigger,
};

#define I2S_INIT(n,dir)	\
									\
struct queue_item rx_##n##_ring_buf[CONFIG_I2S_BLOCK_COUNT+ 1];\
struct queue_item tx_##n##_ring_buf[CONFIG_I2S_BLOCK_COUNT+ 1];\
									\
static struct i2s_litex_data i2s_litex_data_##n = { \
    .dir.mem_block_queue.buf = dir##_##n##_ring_buf,		\
	.dir.mem_block_queue.len = CONFIG_I2S_BLOCK_COUNT+1,	\
    .dir.mem_block_queue.head = 0, \
    .dir.mem_block_queue.tail = 0, \
}; \
    \
static void i2s_litex_irq_config_func_##n(struct device *dev);	\
                            \
static struct i2s_litex_cfg i2s_litex_cfg_##n = { \
    .base = DT_INST_##n##_LITEX_I2S_BASE_ADDRESS_0, \
    .fifo_base = DT_INST_##n##_LITEX_I2S_BASE_ADDRESS_1, \
    .fifo_depth = DT_INST_##n##_LITEX_I2S_FIFO_DEPTH, \
    .irq_config = i2s_litex_irq_config_func_##n \
}; \
DEVICE_AND_API_INIT(i2s_##n, \
        DT_INST_##n##_LITEX_I2S_LABEL, \
        i2s_litex_initialize, \
        &i2s_litex_data_##n, \
        &i2s_litex_cfg_##n, \
        POST_KERNEL, \
        CONFIG_I2S_INIT_PRIORITY, \
        &i2s_litex_driver_api);  \
									\
static void i2s_litex_irq_config_func_##n(struct device *dev)	\
{									\
	IRQ_CONNECT(DT_INST_##n##_LITEX_I2S_IRQ_0, DT_INST_##n##_LITEX_I2S_IRQ_0_PRIORITY,	\
		    i2s_litex_isr_##dir, DEVICE_GET(i2s_##n), 0);	\
	irq_enable(DT_INST_##n##_LITEX_I2S_IRQ_0);	\
    i2s_disable(DT_INST_##n##_LITEX_I2S_BASE_ADDRESS_0);\
}

I2S_INIT(0,rx);

I2S_INIT(1,tx);
