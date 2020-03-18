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

#define I2S_RX_BASE_ADDR DT_INST_0_LITEX_I2S_BASE_ADDRESS_0
#define I2S_RX_EV_STATUS_REG I2S_RX_BASE_ADDR
#define I2S_RX_EV_PENDING_REG (I2S_RX_BASE_ADDR + 0x04)
#define I2S_RX_EV_ENABLE_REG (I2S_RX_BASE_ADDR + 0x08)
#define I2S_RX_CONTROL_REG (I2S_RX_BASE_ADDR + 0x0c)
#define I2S_RX_STATUS_REG (I2S_RX_BASE_ADDR + 0x10)


#define I2S_RX_STAT_OVERFLOW_OFFSET 0
#define I2S_RX_STAT_OVERFLOW_SIZE 1 //bits
#define I2S_RX_STAT_UNDERFLOW_OFFSET 1
#define I2S_RX_STAT_UNDERFLOW_SIZE 1
#define I2S_RX_STAT_DATAREADY_OFFSET 2
#define I2S_RX_STAT_DATAREADY_SIZE 1
#define I2S_RX_STAT_EMPTY_OFFSET 3
#define I2S_RX_STAT_EMPTY_SIZE 1
#define I2S_RX_STAT_WRCOUNT_OFFSET 4
#define I2S_RX_STAT_WRCOUNT_SIZE 9
#define I2S_RX_STAT_RDCOUNT_OFFSET 13
#define I2S_RX_STAT_RDCOUNT_SIZE 9
#define I2S_RX_STAT_FIFO_DEPTH_OFFSET 22
#define I2S_RX_STAT_FIFO_DEPTH_SIZE 9

#define I2S_RX_ENABLE 0x1
// define fifo address for i2s_rx 
#define I2S_RX_FIFO_ADDR DT_INST_0_LITEX_I2S_BASE_ADDRESS_1
//define fifo depth
#define I2S_RX_FIFO_DEPTH (DT_INST_0_LITEX_I2S_FIFO_DEPTH / 32) //bytes

#define I2S_TX_BASE_ADDR DT_INST_1_LITEX_I2S_BASE_ADDRESS_0
#define I2S_TX_EV_STATUS_REG I2S_TX_BASE_ADDR
#define I2S_TX_EV_PENDING_REG (I2S_TX_BASE_ADDR + 0x04)
#define I2S_TX_EV_ENABLE_REG (I2S_TX_BASE_ADDR + 0x08)
#define I2S_TX_CONTROL_REG (I2S_TX_BASE_ADDR + 0x0c)
#define I2S_TX_STATUS_REG (I2S_TX_BASE_ADDR + 0x10)


#define I2S_TX_STAT_OVERFLOW_OFFSET 0
#define I2S_TX_STAT_OVERFLOW_SIZE 1//bits
#define I2S_TX_STAT_UNDERFLOW_OFFSET 1
#define I2S_TX_STAT_UNDERFLOW_SIZE 1
#define I2S_TX_STAT_DATAREADY_OFFSET 2
#define I2S_TX_STAT_DATAREADY_SIZE 1
#define I2S_TX_STAT_EMPTY_OFFSET 3
#define I2S_TX_STAT_EMPTY_SIZE 1
#define I2S_TX_STAT_WRCOUNT_OFFSET 4
#define I2S_TX_STAT_WRCOUNT_SIZE 9
#define I2S_TX_STAT_RDCOUNT_OFFSET 13
#define I2S_TX_STAT_RDCOUNT_SIZE 9
#define I2S_TX_STAT_FIFO_DEPTH_OFFSET 22
#define I2S_TX_STAT_FIFO_DEPTH_SIZE 9

#define I2S_TX_ENABLE 0x1
// define fifo address for i2s_tx
#define I2S_TX_FIFO_ADDR DT_INST_1_LITEX_I2S_BASE_ADDRESS_1
//define fifo depth
#define I2S_TX_FIFO_DEPTH (DT_INST_1_LITEX_I2S_FIFO_DEPTH / 32) //bytes

#define I2S_FIFO_CHUNK_SIZE 4 // bytes
#define I2S_SINGLE_CHANNEL_SIZE 2 // bytes
#define I2S_MAX_FIFO_DEPTH_SIZE (512 /32) 

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
};

#endif /* _I2S_LITEI2S__H */
