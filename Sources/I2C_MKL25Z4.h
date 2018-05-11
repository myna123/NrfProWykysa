/* -----------------------------------------------------------------------------
 * Copyright (c) 2013-2014 ARM Ltd.
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software. Permission is granted to anyone to use this
 * software for any purpose, including commercial applications, and to alter
 * it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software in
 *    a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *
 * $Date:        31. July 2015
 * $Revision:    V1.00
 *
 * Project:      I2C Driver Definitions for Freescale MKL25Z4
 * -------------------------------------------------------------------------- */

#ifndef __I2C_MKL25Z4_H
#define __I2C_MKL25Z4_H

#include "MKL25Z4.h"

#include "Driver_I2C.h"


/* I2C reset value for RGU */
#define RGU_RESET_I2C0      (1 << 16)       // I2C0 reset
#define RGU_RESET_I2C1      (1 << 17)       // I2C1 reset

/* I2C Driver state flags */
#define I2C_FLAG_INIT       (1 << 0)        // Driver initialized
#define I2C_FLAG_POWER      (1 << 1)        // Driver power on
#define I2C_FLAG_SETUP      (1 << 2)        // Master configured, clock set
#define I2C_FLAG_SLAVE_RX   (1 << 3)        // Slave receive registered


/* I2C Stalled Status flags */
#define I2C_MASTER          (1 << 0)        // Master stalled
#define I2C_SLAVE_TX        (1 << 1)        // Slave stalled on transmit
#define I2C_SLAVE_RX        (1 << 2)        // Slave stalled on receive
#define I2C_SLAVE           (I2C_SLAVE_TX | I2C_SLAVE_RX)

/* I2C Status Miscellaneous states */
#define I2C_STAT_BUSERR      0x00           // I2C Bus error


/* I2C Control Information */
typedef struct {
  ARM_I2C_SignalEvent_t cb_event;           // Event callback
  ARM_I2C_STATUS        status;             // Status flags
  uint8_t               flags;              // Control and state flags
  uint8_t               sla_rw;             // Slave address and RW bit
  bool                  pending;            // Transfer pending (no STOP)
  uint8_t               stalled;            // Stall mode status flags;
  /* jd: stalled means after MasterTransmit a MasterReceive will follow.
  This is used when sending command to slave to which slave sends data in response. After
  MasterTransmit we are "stalled" and wait for call to MasterReceive which then
  sends RE-STAR etc. on the bus.
  */

  /* jd: not used
  uint8_t               con_aa;             // I2C slave CON flag
  */
  int32_t               cnt;                // Master transfer count
  uint8_t              *data;               // Master data to transfer
  uint32_t              num;                // Number of bytes to transfer
  uint8_t              *sdata;              // Slave data to transfer
  uint32_t              snum;               // Number of bytes to transfer
} I2C_CTRL;

/* I2C Resource Configuration */
typedef struct {
  I2C_Type        	   *reg;                // I2C register interface
  IRQn_Type             i2c_ev_irq;         // I2C Event IRQ Number
  // jd: volatile uint32_t    *base_clk_reg;       // Base clock register
  volatile uint32_t    *pclk_cfg_reg;       // Peripheral clock config register
  //jd: const volatile uint32_t *pclk_stat_reg;   // Peripheral clock status register
  //jd: uint32_t              rgu_val;            // Peripheral reset value
  I2C_CTRL             *ctrl;               // Run-Time control information
} const I2C_RESOURCES;


/* Declare the driver instances */
#if (RTE_I2C0)
extern ARM_DRIVER_I2C Driver_I2C0;
#endif

#if (RTE_I2C1)
extern ARM_DRIVER_I2C Driver_I2C1;
#endif


#endif /* __I2C_MKL25Z4_H */
