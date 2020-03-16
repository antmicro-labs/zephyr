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

#define I2S_BASE_ADDR CSR_I2S_RX_BASE
#define I2S_EV_STATUS_REG I2S_BASE_ADDR
#define I2S_EV_PENDING_REG (I2S_BASE_ADDR + 0x04)
#define I2S_EV_ENABLE_REG (I2S_BASE_ADDR + 0x08)
#define I2S_CONTROL_REG (I2S_BASE_ADDR + 0x0c)
#define I2S_STATUS_REG (I2S_BASE_ADDR + 0x10)

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

#define I2S_ENABLE 0x1

#endif /* _I2S_LITEI2S__H */
