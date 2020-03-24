/*
 * Copyright (c) 2019 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _I2S_LITEI2S__H
#define _I2S_LITEI2S__H

#include <device.h>
#include <drivers/i2s.h>
#include <devicetree.h>
// i2s register offsets 
#define I2S_EV_PENDING_REG_OFFSET 0x4
#define I2S_EV_ENABLE_REG_OFFSET 0x8
#define I2S_CONTROL_REG_OFFSET 0xc
#define I2S_STATUS_REG_OFFSET 0x10
// i2s status register info
#define I2S_STAT_OVERFLOW_OFFSET 0
#define I2S_STAT_OVERFLOW_SIZE 1 
#define I2S_STAT_UNDERFLOW_OFFSET 1
#define I2S_STAT_UNDERFLOW_SIZE 1
#define I2S_STAT_DATAREADY_OFFSET 2
#define I2S_STAT_DATAREADY_SIZE 1
#define I2S_STAT_EMPTY_OFFSET 3
#define I2S_STAT_EMPTY_SIZE 1
#define I2S_STAT_WRCOUNT_OFFSET 4
#define I2S_STAT_WRCOUNT_SIZE 9
#define I2S_STAT_RDCOUNT_OFFSET 13
#define I2S_STAT_RDCOUNT_SIZE 9
#define I2S_STAT_FIFO_DEPTH_OFFSET 22
#define I2S_STAT_FIFO_DEPTH_SIZE 9
// i2s fifo chunk info
#define I2S_FIFO_CHUNK_SIZE (32 /8) 
#define I2S_CHANNEL_SIZE (16 /8)  
#define I2S_MAX_FIFO_DEPTH_SIZE (512/8) 
// 2is control register options
#define I2S_DISABLE 0x0
#define I2S_ENABLE 0x1
#define I2S_EV_ENABLE 0x1
#define I2S_FIFO_RESET 0x2

// i2s rx 
#define I2S_RX_BASE_ADDR DT_INST_0_LITEX_I2S_BASE_ADDRESS_0
#define I2S_RX_EV_STATUS_REG I2S_RX_BASE_ADDR
#define I2S_RX_EV_PENDING_REG (I2S_RX_BASE_ADDR + I2S_EV_PENDING_REG_OFFSET)
#define I2S_RX_EV_ENABLE_REG (I2S_RX_BASE_ADDR + I2S_EV_ENABLE_REG_OFFSET)
#define I2S_RX_CONTROL_REG (I2S_RX_BASE_ADDR + I2S_CONTROL_REG_OFFSET)
#define I2S_RX_STATUS_REG (I2S_RX_BASE_ADDR + I2S_STATUS_REG_OFFSET)

#define I2S_RX_FIFO_ADDR DT_INST_0_LITEX_I2S_BASE_ADDRESS_1
#define I2S_RX_FIFO_DEPTH (DT_INST_0_LITEX_I2S_FIFO_DEPTH / 8) 


// i2s tx 
#define I2S_TX_BASE_ADDR DT_INST_1_LITEX_I2S_BASE_ADDRESS_0
#define I2S_TX_EV_STATUS_REG I2S_TX_BASE_ADDR
#define I2S_TX_EV_PENDING_REG (I2S_TX_BASE_ADDR + I2S_EV_PENDING_REG_OFFSET)
#define I2S_TX_EV_ENABLE_REG (I2S_TX_BASE_ADDR + I2S_EV_ENABLE_REG_OFFSET)
#define I2S_TX_CONTROL_REG (I2S_TX_BASE_ADDR + I2S_CONTROL_REG_OFFSET)
#define I2S_TX_STATUS_REG (I2S_TX_BASE_ADDR + I2S_STATUS_REG_OFFSET)

#define I2S_TX_FIFO_ADDR DT_INST_1_LITEX_I2S_BASE_ADDRESS_1
#define I2S_TX_FIFO_DEPTH (DT_INST_1_LITEX_I2S_FIFO_DEPTH / 8) 


struct stream {
	s32_t state;
	struct k_sem sem;
	struct i2s_config cfg;
};

/* Device run time data */
struct i2s_litex_data {
	struct stream rx;
	struct stream tx;
};

/* Device const configuration */
struct i2s_litex_cfg
{
   u32_t base;
   u32_t fifo_base;
   u16_t fifo_depth;
   void (*irq_config)(struct device *dev);
};

#endif /* _I2S_LITEI2S__H */
