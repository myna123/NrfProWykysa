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
 * Driver:       Driver_I2C0, Driver_I2C1
 * Configured:   via RTE_Device.h configuration file
 * Project:      I2C Driver for Freescale MKL25Z4
 * Limitations: Only Master mode is supported, slave mode is not implemented!
 * Only 7-bit address is supported.
 * -----------------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting               Value     I2C Interface
 *   ---------------------               -----     -------------
 *   Connect to hardware via Driver_I2C# = 0       use I2C0
 *   Connect to hardware via Driver_I2C# = 1       use I2C1
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 1.01
 *    - ...
 *  Version 1.00
 *    - First version based on Keil driver for NXP LPC18xx V2.01
 *    Only Master mode supported.
 */

/* Notes:
 * Driver uses HAL from Freescale Kinetis SDK for handling device registers.
 * The CMSIS principle of working with I2C:
 * There are 2 scenarios for MasterTransmit:
 * - master only sends data to slave (a command), no data expected from slave
 * - master wants some data from slave and sends command to get these data. Master
 * must first send the command without ending the transaction. Then it sends
 * repeated START and receives data - this is initiated by call to MasterReceive.
 */

#include <string.h>

#include "I2C_MKL25Z4.h"   //#include "I2C_LPC18xx.h"
#include "fsl_i2c_hal.h"	// jd: KSDK HAL functions for handling I2C registers

#include "Driver_I2C.h"
#include "pins_MKL25Z4.h"

#include "RTE_Device.h"

#define ARM_I2C_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,00) /* driver version */



/**
  \fn          uint32_t GetClockFreq (uint32_t clk_src)
  \brief       Return the clock for I2C; in manual it is always bus clock on KL25Z4.
  BUT  in KSDK it is bus clock for I2C0 and system clock (core) for I2C1! see CLOCK_SYS_GetI2cFreq.
  \note  TODO: This could be moved to general file, e.g. clock_mkl25z4.h and use enum for clock src instead of
  I2C_Resources
   We rely on values of bus clock defined in system_MKL24Z4.h file. Hope it will not change.
*/
static uint32_t GetClockFreq (I2C_RESOURCES* i2c)
{
	uint32_t clk = 20971520u;	// preset default clock
#if (CLOCK_SETUP == 0)
	clk = 20971520u;
#elif (CLOCK_SETUP == 1 || CLOCK_SETUP == 4)
	if ( i2c->reg == I2C0 )
		clk = 24000000;	// 24 MHz
	else
		clk = 48000000;
#elif (CLOCK_SETUP == 2)
	if ( i2c->reg == I2C0 )
		clk = 800000;	// 0.8 MHz
	else
		clk = 4000000;
#elif (CLOCK_SETUP == 3)
	if ( i2c->reg == I2C0 )
		clk = 1000000;	// 1 MHz
	else
		clk = 4000000;	// 4 MHz
#else
	#warning CLOCK_SETUP not available for I2C driver. Assuming default bus clock 20.97152MHz.
	// using preset default clock
#endif
	return clk;
}


/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
  ARM_I2C_API_VERSION,
  ARM_I2C_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_I2C_CAPABILITIES DriverCapabilities = {
  0            /* supports 10-bit addressing */
};


#if (RTE_I2C0)
/* I2C0 Control Information */
static I2C_CTRL I2C0_Ctrl = { 0 };

/* I2C0 Resources */
static I2C_RESOURCES I2C0_Resources = {
  I2C0,		/* registers of the I2C module instance for this driver */
  I2C0_IRQn,
  &SIM->SCGC4, // jd: register where clock for i2c is enabled
  &I2C0_Ctrl
};
#endif /* RTE_I2C0 */


#if (RTE_I2C1)
/* I2C1 Control Information */
static I2C_CTRL I2C1_Ctrl = { 0 };

/* I2C1 Resources */
static I2C_RESOURCES I2C1_Resources = {
  I2C1,		/* registers of the I2C module instance for this driver */
  I2C1_IRQn,
  &SIM->SCGC4,
  &I2C1_Ctrl
};
#endif /* RTE_I2C1 */

/**
  \fn          ARM_DRIVER_VERSION I2C_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION I2C_GetVersion (void) {
  return DriverVersion;
}

/**
  \fn          ARM_I2C_CAPABILITIES I2C_GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_I2C_CAPABILITIES
*/
static ARM_I2C_CAPABILITIES I2C_GetCapabilities (void) {
  return DriverCapabilities;
}

/**
  \fn          int32_t I2Cx_Initialize (ARM_I2C_SignalEvent_t cb_event,
                                        I2C_RESOURCES         *i2c)
  \brief       Initialize I2C Interface.
  \param[in]   cb_event  Pointer to \ref ARM_I2C_SignalEvent
  \param[in]   i2c   Pointer to I2C resources
  \return      \ref execution_status
  \note			It enables the clock in SIM for the ports of the pin used by the driver.
*/
static int32_t I2Cx_Initialize (ARM_I2C_SignalEvent_t cb_event, I2C_RESOURCES *i2c) {

  if (i2c->ctrl->flags & I2C_FLAG_POWER) {
    /* Driver initialize not allowed */
    return ARM_DRIVER_ERROR;
  }
  if (i2c->ctrl->flags & I2C_FLAG_INIT) {
    /* Driver already initialized */
    return ARM_DRIVER_OK;
  }
  i2c->ctrl->flags = I2C_FLAG_INIT;

  /* Register driver callback function */
  i2c->ctrl->cb_event = cb_event;

  /* Configure I2C Pins */
  if (i2c->reg == I2C0) {
	  // Enable clock for the pins port
	  PINS_EnablePinPortClock(RTE_I2C0_SCL_PIN);
	  PINS_EnablePinPortClock(RTE_I2C0_SDA_PIN);

	  // Set pin function to I2C
	  PINS_PinConfigure(RTE_I2C0_SCL_PIN, RTE_I2C0_SCL_FUNC);
	  PINS_PinConfigure(RTE_I2C0_SDA_PIN, RTE_I2C0_SDA_FUNC);

  }
  else if (i2c->reg == I2C1) {
	  PINS_EnablePinPortClock(RTE_I2C1_SCL_PIN);
	  PINS_EnablePinPortClock(RTE_I2C1_SDA_PIN);
	  PINS_PinConfigure(RTE_I2C1_SCL_PIN, RTE_I2C1_SCL_FUNC);
	  PINS_PinConfigure(RTE_I2C1_SDA_PIN, RTE_I2C1_SDA_FUNC);
  }

  /* Clear driver status */
  memset (&i2c->ctrl->status, 0, sizeof(ARM_I2C_STATUS));

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I2Cx_Uninitialize (I2C_RESOURCES *i2c)
  \brief       De-initialize I2C Interface.
  \param[in]   i2c   Pointer to I2C resources
  \return      \ref execution_status
*/
static int32_t I2Cx_Uninitialize (I2C_RESOURCES *i2c) {

  if (!(i2c->ctrl->flags & I2C_FLAG_INIT)) {
    /* Driver not initialized */
    return ARM_DRIVER_OK;
  }
  if (i2c->ctrl->flags & I2C_FLAG_POWER) {
    /* Driver needs POWER_OFF first */
    return ARM_DRIVER_ERROR;
  }
  i2c->ctrl->flags = 0;

  /* Unconfigure SCL and SDA pins */
  if (i2c->reg == I2C0) {
 	  PINS_PinConfigure(RTE_I2C0_SCL_PIN, 0);
 	  PINS_PinConfigure(RTE_I2C0_SDA_PIN, 0);
   }
   else if (i2c->reg == I2C1) {
 	  PINS_PinConfigure(RTE_I2C1_SCL_PIN, 0);
 	  PINS_PinConfigure(RTE_I2C1_SDA_PIN, 0);
   }


  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I2Cx_PowerControl (ARM_POWER_STATE state,
                                          I2C_RESOURCES   *i2c)
  \brief       Control I2C Interface Power.
  \param[in]   state  Power state
  \param[in]   i2c    Pointer to I2C resources
  \return      \ref execution_status
*/
static int32_t I2Cx_PowerControl (ARM_POWER_STATE state, I2C_RESOURCES *i2c) {

  if (!(i2c->ctrl->flags & I2C_FLAG_INIT)) {
    /* Driver not initialized */
    return ARM_DRIVER_ERROR;
  }

  switch (state) {
    case ARM_POWER_OFF:
      if (!(i2c->ctrl->flags & I2C_FLAG_POWER)) {
        /* Driver not powered */
        break;
      }
      if (i2c->ctrl->status.busy) {
        /* Transfer operation in progress */
        return ARM_DRIVER_ERROR_BUSY;
      }
      i2c->ctrl->flags = I2C_FLAG_INIT;

      /* Disable I2C interrupts */
      NVIC_DisableIRQ (i2c->i2c_ev_irq);

      I2C_HAL_SetIntCmd(i2c->reg, false);	/* jd: disable interrupts in i2c module*/

      /* Disable I2C Operation */
      /* Use Kinetis SDK HALL: */
      I2C_HAL_Disable(i2c->reg);

      /* Disable I2C peripheral clock */
      if (i2c->reg == I2C0) {
    	  *i2c->pclk_cfg_reg &= ~SIM_SCGC4_I2C0_MASK;
      } else if (i2c->reg == I2C1) {
    	  *i2c->pclk_cfg_reg &= ~SIM_SCGC4_I2C1_MASK;
      }

      break;

    case ARM_POWER_FULL:
      if (i2c->ctrl->flags & I2C_FLAG_POWER) {
        /* Driver already powered */
        break;
      }

      /* Enable I2C peripheral clock */
      if (i2c->reg == I2C0) {
    	  *i2c->pclk_cfg_reg |= SIM_SCGC4_I2C0_MASK;
      } else if (i2c->reg == I2C1) {
    	  *i2c->pclk_cfg_reg |= SIM_SCGC4_I2C1_MASK;
      }
      // jd: TODO: wait for clock running?

      /* Reset I2C peripheral */
      /*jd: not available on KL25
      LPC_RGU->RESET_CTRL1 = i2c->rgu_val;
      while (!(LPC_RGU->RESET_ACTIVE_STATUS1 & i2c->rgu_val));
      */

      /* Enable I2C Operation */
      // KSDK version:
      I2C_HAL_Init(i2c->reg);

      i2c->ctrl->stalled = 0;
      //i2c->ctrl->con_aa  = 0;

      /* Enable I2C interrupts */
      NVIC_ClearPendingIRQ (i2c->i2c_ev_irq);
      NVIC_EnableIRQ (i2c->i2c_ev_irq);

      I2C_HAL_Enable(i2c->reg);
      I2C_HAL_SetIntCmd(i2c->reg, true);	/* jd: enable interrupts in i2c module*/

      i2c->ctrl->flags |= I2C_FLAG_POWER;
      break;

    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I2Cx_MasterTransmit (uint32_t       addr,
                                            const uint8_t *data,
                                            uint32_t       num,
                                            bool           xfer_pending,
                                            I2C_RESOURCES *i2c)
  \brief       Start transmitting data as I2C Master.
  \param[in]   addr          Slave address (7-bit only! 10-bit not supported)
  \param[in]   data          Pointer to buffer with data to transmit to I2C Slave
  \param[in]   num           Number of data bytes to transmit
  \param[in]   xfer_pending  Transfer operation is pending - Stop condition will not be generated
  \param[in]   i2c           Pointer to I2C resources
  \return      \ref execution_status
*/
static int32_t I2Cx_MasterTransmit (uint32_t       addr,
                                    const uint8_t *data,
                                    uint32_t       num,
                                    bool           xfer_pending,
                                    I2C_RESOURCES *i2c) {

	/* Note: MasterTransmit always uses 0 value of r/w bit (= send).
	 Bit r/w - read is 1, write is 0. */

 /* jd: num == 0 is valid request to send just slave address
  with no data (and write bit) */
  if (!data || /*!num ||*/ (addr > 0x7F)) {
    /* Invalid parameters */
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (!(i2c->ctrl->flags & I2C_FLAG_SETUP)) {
    /* Driver not yet configured */
    return ARM_DRIVER_ERROR;
  }

  if (i2c->ctrl->status.busy || (i2c->ctrl->stalled & I2C_SLAVE)) {
    /* Transfer operation in progress, or Slave stalled */
    return ARM_DRIVER_ERROR_BUSY;
  }

  //NVIC_DisableIRQ (i2c->i2c_ev_irq);
  I2C_HAL_SetIntCmd(i2c->reg, false);

  /* Set control variables */
  i2c->ctrl->sla_rw  = addr << 1;	// jd: zde vzdy posilame RW bit = 0 tj. write
  i2c->ctrl->pending = xfer_pending;
  i2c->ctrl->data    = (uint8_t *)data;
  i2c->ctrl->num     = num;
  i2c->ctrl->cnt     = -1;

  /* Update driver status */
  i2c->ctrl->status.busy             = 1;
  i2c->ctrl->status.mode             = 1;		/* 1 = master */
  i2c->ctrl->status.direction        = 0;		/* 0 = transmit, 1 = receive */
  i2c->ctrl->status.arbitration_lost = 0;
  i2c->ctrl->status.bus_error        = 0;
  if (!i2c->ctrl->stalled) {
	  // jd: from KSDK
	  /* Set direction to send for sending of address and data. */
	  I2C_HAL_SetDirMode(i2c->reg, kI2CSend);
	  I2C_HAL_SendStart(i2c->reg);		// also sets I2C to master mode if not already set
	  // Note: 1 byte transmitted to trigger interrupt is not included in  i2c->ctrl->num,
	  // this 1st byte is the slave address
	  i2c->ctrl->cnt = 0;	// this is used as index to data buffer in ISR
	  I2C_HAL_WriteByte(i2c->reg, i2c->ctrl->sla_rw);	/* Send slave address */
	  /* For Kinetis we need to send first byte (slave address) to get an interrupt; it seems LPC just
	   * needs to send start to trigger interrupt. */
  }
  /* Note: if already stalled is handled in error check above */

  //NVIC_EnableIRQ (i2c->i2c_ev_irq);
  //I2C_HAL_ClearInt(i2c->reg);
  I2C_HAL_SetIntCmd(i2c->reg, true);

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I2Cx_MasterReceive (uint32_t       addr,
                                           uint8_t       *data,
                                           uint32_t       num,
                                           bool           xfer_pending,
                                           I2C_RESOURCES *i2c)
  \brief       Start receiving data as I2C Master.
  \param[in]   addr          Slave address (7-bit or 10-bit)
  \param[out]  data          Pointer to buffer for data to receive from I2C Slave
  \param[in]   num           Number of data bytes to receive
  \param[in]   xfer_pending  Transfer operation is pending - Stop condition will not be generated
  \param[in]   i2c           Pointer to I2C resources
  \return      \ref execution_status
*/
static int32_t I2Cx_MasterReceive (uint32_t       addr,
                                   uint8_t       *data,
                                   uint32_t       num,
                                   bool           xfer_pending,
                                   I2C_RESOURCES *i2c) {

  if (!data || !num || (addr > 0x7F)) {
    /* Invalid parameters */ 
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (!(i2c->ctrl->flags & I2C_FLAG_SETUP)) {
    /* Driver not yet configured */
    return ARM_DRIVER_ERROR;
  }

  if (i2c->ctrl->status.busy || (i2c->ctrl->stalled & I2C_SLAVE)) {
    /* Transfer operation in progress, or Slave stalled */
    return ARM_DRIVER_ERROR_BUSY;
  }

  //NVIC_DisableIRQ (i2c->i2c_ev_irq);
  I2C_HAL_SetIntCmd(i2c->reg, false);

  /* Set control variables */
  i2c->ctrl->sla_rw  = (addr << 1) | 0x01;	// Address with Read bit
  i2c->ctrl->pending = xfer_pending;
  i2c->ctrl->data    = data;
  i2c->ctrl->num     = num;
  i2c->ctrl->cnt     = -1;

  /* Update driver status */
  i2c->ctrl->status.busy             = 1;
  i2c->ctrl->status.mode             = 1;
  i2c->ctrl->status.direction        = 0;	/* 0 = transmit, 1 = receive */
  i2c->ctrl->status.arbitration_lost = 0;
  i2c->ctrl->status.bus_error        = 0;

  /* If not stalled, it means we do not generate RE-START, this is simple
     master receive operation: send slave address and slave responds with data */
  /* Note that because we must always generate START or RE-START, the code for
     stalled and not stalled version is the same */

	I2C_HAL_SetDirMode(i2c->reg, kI2CSend); /* sending address */
	I2C_HAL_SendStart(i2c->reg);
	I2C_HAL_WriteByte(i2c->reg, i2c->ctrl->sla_rw); /* Send slave address */
	i2c->ctrl->cnt = 0;	// received 0 bytes of data so far

	//NVIC_EnableIRQ (i2c->i2c_ev_irq);
	I2C_HAL_SetIntCmd(i2c->reg, true);

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I2Cx_SlaveTransmit (const uint8_t *data,
                                           uint32_t       num,
                                           I2C_RESOURCES *i2c)
  \brief       Start transmitting data as I2C Slave.
  \param[in]   data  Pointer to buffer with data to transmit to I2C Master
  \param[in]   num   Number of data bytes to transmit
  \param[in]   i2c   Pointer to I2C resources
  \return      \ref execution_status
*/
static int32_t I2Cx_SlaveTransmit (const uint8_t *data,
                                   uint32_t       num,
                                   I2C_RESOURCES *i2c) {

  if (!data || !num) {
    /* Invalid parameters */
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (i2c->ctrl->status.busy || (i2c->ctrl->stalled & (I2C_MASTER | I2C_SLAVE_RX))) {
    /* Transfer operation in progress, Master stalled or Slave receive stalled */
    return ARM_DRIVER_ERROR_BUSY;
  }

  NVIC_DisableIRQ (i2c->i2c_ev_irq);

  /* Set control variables */
  i2c->ctrl->flags &= ~I2C_FLAG_SLAVE_RX;
  i2c->ctrl->sdata  = (uint8_t *)data;
  i2c->ctrl->snum   = num;
  i2c->ctrl->cnt    = -1;

  /* Update driver status */
  i2c->ctrl->status.general_call = 0;
  i2c->ctrl->status.bus_error    = 0;

  NVIC_EnableIRQ (i2c->i2c_ev_irq);

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I2Cx_SlaveReceive (uint8_t       *data,
                                          uint32_t       num,
                                          I2C_RESOURCES *i2c)
  \brief       Start receiving data as I2C Slave.
  \param[out]  data  Pointer to buffer for data to receive from I2C Master
  \param[in]   num   Number of data bytes to receive
  \param[in]   i2c   Pointer to I2C resources
  \return      \ref execution_status
*/
static int32_t I2Cx_SlaveReceive (uint8_t       *data,
                                  uint32_t       num,
                                  I2C_RESOURCES *i2c) {

  if (!data || !num) {
    /* Invalid parameters */ 
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (i2c->ctrl->status.busy || (i2c->ctrl->stalled & (I2C_MASTER | I2C_SLAVE_TX))) {
    /* Transfer operation in progress, Master stalled or Slave transmit stalled */
    return ARM_DRIVER_ERROR_BUSY;
  }

  NVIC_DisableIRQ (i2c->i2c_ev_irq);

  /* Set control variables */
  i2c->ctrl->flags |= I2C_FLAG_SLAVE_RX;
  i2c->ctrl->sdata  = data;
  i2c->ctrl->snum   = num;
  i2c->ctrl->cnt    = -1;

  /* Update driver status */
  i2c->ctrl->status.general_call = 0;
  i2c->ctrl->status.bus_error    = 0;

  NVIC_EnableIRQ (i2c->i2c_ev_irq);

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I2Cx_GetDataCount (I2C_RESOURCES *i2c)
  \brief       Get transferred data count.
  \return      number of data bytes transferred; -1 when Slave is not addressed by Master
*/
static int32_t I2Cx_GetDataCount (I2C_RESOURCES *i2c) {
  return (i2c->ctrl->cnt);
}

/**
  \fn          int32_t I2Cx_Control (uint32_t       control,
                                     uint32_t       arg,
                                     I2C_RESOURCES *i2c)
  \brief       Control I2C Interface.
  \param[in]   control  operation
  \param[in]   arg      argument of operation (optional)
  \param[in]   i2c      pointer to I2C resources
  \return      \ref execution_status
*/
static int32_t I2Cx_Control (uint32_t control, uint32_t arg, I2C_RESOURCES *i2c) {
  uint32_t val, clk;	// conset;
  //uint8_t mul, icr;

  if (!(i2c->ctrl->flags & I2C_FLAG_POWER)) {
    /* Driver not powered */
    return ARM_DRIVER_ERROR;
  }
  switch (control) {
    case ARM_I2C_OWN_ADDRESS:
      /* Set Own Slave Address */
      val = (arg << 1) & 0xFF;
      if (arg & ARM_I2C_ADDRESS_GC) {
        /* General call enable */
    	I2C_HAL_SetGeneralCallCmd(i2c->reg, true);
      }
      else {
    	  // jd: added else to disable general call
    	  I2C_HAL_SetGeneralCallCmd(i2c->reg, false);
      }

      i2c->reg->A1 = val;		// jd: 7-bit address; same as on LPC18xx


      /* Enable assert acknowledge */
      if ( val )
    	  i2c->reg->C1 &= ~I2C_C1_TXAK_MASK;	// enable ACK
      // KSDK has only I2C_HAL_SendAck which does the same
      break;

    case ARM_I2C_BUS_SPEED:
      /* Set Bus Speed */
      clk = GetClockFreq (i2c);
      switch (arg) {
        case ARM_I2C_BUS_SPEED_STANDARD:
          /* Standard Speed (100kHz) */
          //clk /= 100000;
          I2C_HAL_SetBaudRate(i2c->reg, clk, 100, NULL);
          break;
        case ARM_I2C_BUS_SPEED_FAST:
          /* Fast Speed     (400kHz) */
          //clk /= 400000;
          I2C_HAL_SetBaudRate(i2c->reg, clk, 400, NULL);
          break;
#if 0	/* jd: not supported for now */
        case ARM_I2C_BUS_SPEED_FAST_PLUS:
          /* Fast+ Speed    (  1MHz) */
          if (i2c->reg == LPC_I2C0) {
            clk /= 1000000;
            break;
          }
#endif
        default:
          return ARM_DRIVER_ERROR_UNSUPPORTED;
      }
      /* jd: MUL * ICR = BUS/Baud
       * clk is now BUS/Baud
       * ICR must fit 6 bits -> ICR < 63 (0x3F)
       * Max bus clock is 24 MHz, with min speed 100 kHz this gives:
       * MUL * ICR = 24000000 / 100000 = 240
       * So we need to use MUL "dynamically", it cannot be 1 for all cases.
       * */
#if 0	/* replaced by KSDK HAL I2C_HAL_SetBaudRate */
      mul = 1;
      tmp_clk = clk;
      while ( (tmp_clk > 63) && (mult < 4)  )
      {
    	  tmp_clk = clk;
    	  mul = mul << 1;
    	  tmp_clk = tpm_clk / mul;	// try ICR value with this mult
      }
      icr = (uint8_t)(clk/mul);	// now calculate the real value of ICR
      i2c->reg->F = (((mul-1) < 6) | icr);
#endif


      /* Speed configured, I2C Master active */
      i2c->ctrl->flags |= I2C_FLAG_SETUP;
      break;

    case ARM_I2C_BUS_CLEAR:
      /* Execute Bus clear */
      return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_I2C_ABORT_TRANSFER:
      /* Abort Master/Slave transfer */
      //NVIC_DisableIRQ (i2c->i2c_ev_irq);
      I2C_HAL_SetIntCmd(i2c->reg, false);	/* disable interrupts */

      i2c->ctrl->status.busy = 0;
      i2c->ctrl->stalled = 0;
      i2c->ctrl->snum    = 0;
      /* Master: send STOP to I2C bus           */
      /* Slave:  enter non-addressed Slave mode */
      // jd: writing STO in master mode sends STOP signal to bus,
      // in slave mode it recovers from error condition.
      I2C_HAL_SendStop(i2c->reg);
      I2C_HAL_SetIntCmd(i2c->reg, true);	/* disable interrupts */
      //NVIC_EnableIRQ (i2c->i2c_ev_irq);
      break;

    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }
  return ARM_DRIVER_OK;
}

/**
  \fn          ARM_I2C_STATUS I2Cx_GetStatus (I2C_RESOURCES *i2c)
  \brief       Get I2C status.
  \param[in]   i2c      pointer to I2C resources
  \return      I2C status \ref ARM_I2C_STATUS
*/
static ARM_I2C_STATUS I2Cx_GetStatus (I2C_RESOURCES *i2c) {
  return (i2c->ctrl->status);
}

/**
  \fn          void I2Cx_MasterHandler (I2C_RESOURCES *i2c)
  \brief       I2C Master state event handler.
  \param[in]   i2c  Pointer to I2C resources
  \return      I2C event notification flags. This will be given to user callback if defined.
*/
static uint32_t I2Cx_MasterHandler (I2C_RESOURCES *i2c)
{
  uint32_t event  = 0;

  /* Clear the interrupt flag*/
  I2C_HAL_ClearInt(i2c->reg);

   	/* ERROR: Bus error, timeout? */
  	/*  LPC event: I2C_STAT_BUSERR */
  	if (!I2C_HAL_GetStatusFlag(i2c->reg, kI2CBusBusy)) {
  		/* If no transfer in progress */
  		i2c->ctrl->status.bus_error = 1;
  		i2c->ctrl->status.busy = 0;
  		i2c->ctrl->status.mode = 0;
  		event = ARM_I2C_EVENT_BUS_ERROR |
  		ARM_I2C_EVENT_TRANSFER_DONE |
  		ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
  		return (event);	// nothing else to do
  	}

    /* ERROR: Arbitration lost */
  	if ( I2C_HAL_GetStatusFlag(i2c->reg, kI2CArbitrationLost) )	{
  	      i2c->ctrl->status.arbitration_lost = 1;
  	      i2c->ctrl->status.busy             = 0;
  	      i2c->ctrl->status.mode             = 0;
  	      event = ARM_I2C_EVENT_ARBITRATION_LOST |
  	              ARM_I2C_EVENT_TRANSFER_DONE    |
  	              ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
  	      I2C_HAL_SetIntCmd(i2c->reg, false);
  	      I2C_HAL_SendStop(i2c->reg);
  	      I2C_HAL_ClearArbitrationLost(i2c->reg);
  	      return (event);
  	}

    /* Get current master transfer direction */
    i2c_direction_t direction = I2C_HAL_GetDirMode(i2c->reg);
	if (direction == kI2CSend)
	{ 	/* Sending data */

		/* ERROR: negative assertion (NACK) received.
		 * Only important in transmit mode; in receive mode we generate ACK/NACK. */
		if (I2C_HAL_GetStatusFlag(i2c->reg, kI2CReceivedNak)) {
			/* Record that we got a NAK */
			i2c->ctrl->status.busy = 0;
			i2c->ctrl->status.mode = 0;
			event = ARM_I2C_EVENT_ADDRESS_NACK |
					ARM_I2C_EVENT_TRANSFER_DONE |
					ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
			/* Note: we do not know if it is address or data NACK and there is
			 * no code for data NACK in Drivee_I2C anyway*/
			/* Got a NACK, so we're done with this transfer */
			I2C_HAL_SetIntCmd(i2c->reg, false);
			I2C_HAL_SendStop(i2c->reg);
			return (event);
		}

		if (i2c->ctrl->stalled == I2C_MASTER
				|| (i2c->ctrl->sla_rw & 0x01) ) {
			/* Stalled means MasterTransmit was called earlier with pending = true
			 * and this is 1st call to MasterReceive afterwards.  This is I2C transfer
			 * with repeated start sending.
			 * Another case is "normal" I2C read; MasterTransmit sends slave address with
			 * Write bit and we now need to switch to receive mode to get answer from slave. */

			/* Switch to receive mode*/
			I2C_HAL_SetDirMode(i2c->reg, kI2CReceive);
			i2c->ctrl->status.direction = 1;	/* 0 = transmit, 1 = receive */

			/* Initiate read from slave */
			/* If we want only 1 byte, we need to set NACK before reading */
			if (i2c->ctrl->num == 1 )
				I2C_HAL_SendNak(i2c->reg);
			else
				I2C_HAL_SendAck(i2c->reg);

			/* Dummy read to trigger receive of next byte in interrupt. */
			I2C_HAL_ReadByte(i2c->reg);

			/* Clear stalled flag if it was set */
			i2c->ctrl->stalled = 0;

		}
		else {
			/* Really just sending data; special cases handled above  */

			if (i2c->ctrl->num > 0) {	/* There are bytes to send */
				/* Transmit next byte and update buffer index */
				I2C_HAL_WriteByte(i2c->reg, i2c->ctrl->data[i2c->ctrl->cnt]);
				i2c->ctrl->cnt++;
				i2c->ctrl->num--;
			} else {
				/* No more data to send */
				/* There are 2 cases what can happen after last byte is sent:
					a) send STOP (if MasterTransmit called with pending = false)
					b) do not send STOP and go to receive mode (and send RE-START)
				*/
				if (!i2c->ctrl->pending) {
					/* Finish sending data, send STOP, disable interrupt */
					I2C_HAL_SetIntCmd(i2c->reg, false);
					I2C_HAL_SendStop(i2c->reg);
					i2c->ctrl->status.busy = 0;
					i2c->ctrl->status.mode = 0;
					event = ARM_I2C_EVENT_TRANSFER_DONE;
					return (event);
				} else {
					 /* If will be receiving data, do not send stop */
					/* MasterReceive will be called by the user to start receiving data.
					 * Note that we cannot switch to receive mode here - in MasterReceive
					 * slave address must be sent first, so we must remain in transmit mode. */
					i2c->ctrl->stalled = I2C_MASTER;
					i2c->ctrl->status.busy = 0;

				}	/* else pending*/
			}	/* else no more data */
		}	/* else just sending data */

	}	/* if in transmit mode */
	else
	{ 	/* Receiving data (mater receive) */

		i2c->ctrl->num--;

		switch (i2c->ctrl->num) {
		case 0x0U:
			/* Finish receive data, send STOP, disable interrupt */
			I2C_HAL_SetIntCmd(i2c->reg, false);
			I2C_HAL_SendStop(i2c->reg);
			i2c->ctrl->status.busy = 0;
			event = ARM_I2C_EVENT_TRANSFER_DONE;
			break;
		case 0x1U:
			/* For the byte before last, we need to set NAK */
			I2C_HAL_SendNak(i2c->reg);
			break;
		default:
			I2C_HAL_SendAck(i2c->reg);
			break;
		}

		/* Read recently received byte into buffer and update buffer index */
		i2c->ctrl->data[i2c->ctrl->cnt++] = I2C_HAL_ReadByte(i2c->reg);

		return (event);
	}

	/* never should get here */

}	/* end of MasterHandler*/


/**
  \fn          void I2Cx_SlaveHandler (I2C_RESOURCES *i2c)
  \brief       I2C Slave state event handler.
  \param[in]   i2c  Pointer to I2C resources
  \return      I2C event notification flags
*/
static uint32_t I2Cx_SlaveHandler (I2C_RESOURCES *i2c) {

	/* jd: not implemented */
	uint32_t event  = 0;
	event = ARM_I2C_EVENT_TRANSFER_DONE |
	                 ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
	return (event);
}

#if 0
  uint32_t conset = 0;
  uint32_t event  = 0;

  switch (i2c->reg->STAT & 0xF8) {
    case I2C_STAT_SL_ALOST_GC:
      /* Arbitration lost in General call */
      i2c->ctrl->status.arbitration_lost = 1;
    case I2C_STAT_SL_GCA_A:
      /* General address recvd, ACK returned */
      i2c->ctrl->status.general_call     = 1;
      goto slaw_a;

    case I2C_STAT_SL_ALOST_MW:
      /* Arbitration lost SLA+W */
      i2c->ctrl->status.arbitration_lost = 1;
    case I2C_STAT_SL_SLAW_A:
      /* SLA+W received, ACK returned */
slaw_a:
      /* Stalled Slave receiver also resumes here */
      if (!i2c->ctrl->snum || !(i2c->ctrl->flags & I2C_FLAG_SLAVE_RX)) {
        /* Receive buffer unavailable */
        if (i2c->ctrl->stalled) {
          /* Already stalled, abort transaction to prevent dead-loops */
          event = ARM_I2C_EVENT_TRANSFER_DONE |
                  ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
          conset = I2C_CON_STO | i2c->ctrl->con_aa;
          break;
        }
        /* Stall I2C transaction */
        NVIC_DisableIRQ (i2c->i2c_ev_irq);
        i2c->ctrl->stalled = I2C_SLAVE_RX;
        return (ARM_I2C_EVENT_SLAVE_RECEIVE);
      }
      i2c->ctrl->status.direction = 1;
      i2c->ctrl->status.busy      = 1;
      i2c->ctrl->cnt     = 0;
      i2c->ctrl->stalled = 0;
      conset = I2C_CON_AA;
      break;

    case I2C_STAT_SL_DRGC_A:
      /* Data recvd General call, ACK returned */
    case I2C_STAT_SL_DR_A:
      /* Data received, ACK returned */
      i2c->ctrl->sdata[i2c->ctrl->cnt++] = i2c->reg->DAT;
      i2c->ctrl->snum--;
      if (i2c->ctrl->snum) {
        conset = I2C_CON_AA;
      }
      break;

    case I2C_STAT_SL_ALOST_MR:
      /* Arbitration lost SLA+R */
      i2c->ctrl->status.arbitration_lost = 1;
    case I2C_STAT_SL_SLAR_A:
      /* SLA+R received, ACK returned */
      /* Stalled Slave transmitter also resumes here */
      if (!i2c->ctrl->snum || (i2c->ctrl->flags & I2C_FLAG_SLAVE_RX)) {
        /* Transmit buffer unavailable */
        if (i2c->ctrl->stalled) {
          /* Already stalled, abort transaction to prevent dead-loops */
          event = ARM_I2C_EVENT_TRANSFER_DONE |
                  ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
          conset = I2C_CON_STO | i2c->ctrl->con_aa;
          break;
        }
        NVIC_DisableIRQ (i2c->i2c_ev_irq);
        i2c->ctrl->stalled = I2C_SLAVE_TX;
        return (ARM_I2C_EVENT_SLAVE_TRANSMIT);
      }
      i2c->ctrl->status.direction = 0;
      i2c->ctrl->status.busy      = 1;
      i2c->ctrl->cnt     = 0;
      i2c->ctrl->stalled = 0;
    case I2C_STAT_SL_DT_A:
      /* Data transmitted, ACK received */
      i2c->reg->DAT = i2c->ctrl->sdata[i2c->ctrl->cnt++];
      i2c->ctrl->snum--;
      if (i2c->ctrl->snum) {
        conset = I2C_CON_AA;
      }
      break;

    case I2C_STAT_SL_DT_NA:
      /* Data transmitted, no ACK received */
    case I2C_STAT_SL_LDT_A:
      /* Last data transmitted, ACK received */
    case I2C_STAT_SL_DR_NA:
      /* Data received, no ACK returned */
    case I2C_STAT_SL_DRGC_NA:
      /* Data recvd General call, no ACK returned */
    case I2C_STAT_SL_STOP:
      /* STOP received while addressed */
      i2c->ctrl->status.busy = 0;
      /* Slave operation completed, generate events */
      event = ARM_I2C_EVENT_TRANSFER_DONE;
      if (i2c->ctrl->status.arbitration_lost) {
        event |= ARM_I2C_EVENT_ARBITRATION_LOST;
      }
      if (i2c->ctrl->status.general_call) {
        event |= ARM_I2C_EVENT_GENERAL_CALL;
      }
      if (i2c->ctrl->snum) {
        event |= ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
      }
      conset = i2c->ctrl->con_aa;
      break;
  }
  /* Set/clear control flags */
  i2c->reg->CONSET = conset;
  i2c->reg->CONCLR = conset ^ I2C_CON_FLAGS;

  return (event);
}
#endif

/**
  \fn          void I2Cx_IRQHandler (I2C_RESOURCES *i2c)
  \brief       I2C Event Interrupt handler.
  \param[in]   i2c  Pointer to I2C resources
*/
static void I2Cx_IRQHandler (I2C_RESOURCES *i2c) {
  uint32_t event;

  if (I2C_HAL_IsMaster(i2c->reg) )
	  event = I2Cx_MasterHandler (i2c);
  else
	  event = I2Cx_SlaveHandler (i2c);

  /* Callback event notification */
  if (event && i2c->ctrl->cb_event) {
    i2c->ctrl->cb_event (event);
  }
}

#if (RTE_I2C0)
/* I2C0 Driver wrapper functions */
static int32_t I2C0_Initialize (ARM_I2C_SignalEvent_t cb_event) {
  return (I2Cx_Initialize (cb_event, &I2C0_Resources));
}
static int32_t I2C0_Uninitialize (void) {
  return (I2Cx_Uninitialize (&I2C0_Resources));
}
static int32_t I2C0_PowerControl (ARM_POWER_STATE state) {
  return (I2Cx_PowerControl (state, &I2C0_Resources));
}
static int32_t I2C0_MasterTransmit (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending) {
  return (I2Cx_MasterTransmit (addr, data, num, xfer_pending, &I2C0_Resources));
}
static int32_t I2C0_MasterReceive (uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending) {
  return (I2Cx_MasterReceive (addr, data, num, xfer_pending, &I2C0_Resources));
}
static int32_t I2C0_SlaveTransmit (const uint8_t *data, uint32_t num) {
  return (I2Cx_SlaveTransmit (data, num, &I2C0_Resources));
}
static int32_t I2C0_SlaveReceive (uint8_t *data, uint32_t num) {
  return (I2Cx_SlaveReceive (data, num, &I2C0_Resources));
}
static int32_t I2C0_GetDataCount (void) {
  return (I2Cx_GetDataCount (&I2C0_Resources));
}
static int32_t I2C0_Control (uint32_t control, uint32_t arg) {
  return (I2Cx_Control (control, arg, &I2C0_Resources));
}
static ARM_I2C_STATUS I2C0_GetStatus (void) {
  return (I2Cx_GetStatus (&I2C0_Resources));
}
void I2C0_IRQHandler (void) {
  I2Cx_IRQHandler (&I2C0_Resources);
}

/* I2C0 Driver Control Block */
ARM_DRIVER_I2C Driver_I2C0 = {
  I2C_GetVersion,
  I2C_GetCapabilities,
  I2C0_Initialize,
  I2C0_Uninitialize,
  I2C0_PowerControl,
  I2C0_MasterTransmit,
  I2C0_MasterReceive,
  I2C0_SlaveTransmit,
  I2C0_SlaveReceive,
  I2C0_GetDataCount,
  I2C0_Control,
  I2C0_GetStatus
};
#endif

#if (RTE_I2C1)
/* I2C1 Driver wrapper functions */
static int32_t I2C1_Initialize (ARM_I2C_SignalEvent_t cb_event) {
  return (I2Cx_Initialize (cb_event, &I2C1_Resources));
}
static int32_t I2C1_Uninitialize (void) {
  return (I2Cx_Uninitialize (&I2C1_Resources));
}
static int32_t I2C1_PowerControl (ARM_POWER_STATE state) {
  return (I2Cx_PowerControl (state, &I2C1_Resources));
}
static int32_t I2C1_MasterTransmit (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending) {
  return (I2Cx_MasterTransmit (addr, data, num, xfer_pending, &I2C1_Resources));
}
static int32_t I2C1_MasterReceive (uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending) {
  return (I2Cx_MasterReceive (addr, data, num, xfer_pending, &I2C1_Resources));
}
static int32_t I2C1_SlaveTransmit (const uint8_t *data, uint32_t num) {
  return (I2Cx_SlaveTransmit (data, num, &I2C1_Resources));
}
static int32_t I2C1_SlaveReceive (uint8_t *data, uint32_t num) {
  return (I2Cx_SlaveReceive (data, num, &I2C1_Resources));
}
static int32_t I2C1_GetDataCount (void) {
  return (I2Cx_GetDataCount (&I2C1_Resources));
}
static int32_t I2C1_Control (uint32_t control, uint32_t arg) {
  return (I2Cx_Control (control, arg, &I2C1_Resources));
}
static ARM_I2C_STATUS I2C1_GetStatus (void) {
  return (I2Cx_GetStatus (&I2C1_Resources));
}
void I2C1_IRQHandler (void) {
  I2Cx_IRQHandler (&I2C1_Resources);
}

/* I2C1 Driver Control Block */
ARM_DRIVER_I2C Driver_I2C1 = {
  I2C_GetVersion,
  I2C_GetCapabilities,
  I2C1_Initialize,
  I2C1_Uninitialize,
  I2C1_PowerControl,
  I2C1_MasterTransmit,
  I2C1_MasterReceive,
  I2C1_SlaveTransmit,
  I2C1_SlaveReceive,
  I2C1_GetDataCount,
  I2C1_Control,
  I2C1_GetStatus
};
#endif
