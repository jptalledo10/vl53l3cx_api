/*******************************************************************************
Copyright © 2015, STMicroelectronics International N.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of STMicroelectronics nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/


#ifndef _VL53L0X_PLATFORM_H_
#define _VL53L0X_PLATFORM_H_

#include "vl53lx_def.h"
#include "vl53lx_platform_log.h"
#include "vl53lx_platform_user_data.h"

#ifdef __cplusplus
extern "C" {
#endif

int VL53L0X_i2c_init(char * devPath, int devAddr);
int32_t VL53L0X_i2c_close(void);

/**
 * @file vl53l0_platform.h
 *
 * @brief All end user OS/platform/application porting
 */
 
/**
 * @defgroup VL53L0X_platform_group VL53L0 Platform Functions
 * @brief    VL53L0 Platform Functions
 *  @{
 */



/**
 * @brief   Declare the device Handle as a pointer of the structure @a VL53LX_Dev_t.
 *
 */
typedef VL53LX_Dev_t* VL53LX_DEV;

/**
 * @def PALDevDataGet
 * @brief Get ST private structure @a VL53LX_DevData_t data access
 *
 * @param Dev       Device Handle
 * @param field     ST structure field name
 * It maybe used and as real data "ref" not just as "get" for sub-structure item
 * like PALDevDataGet(FilterData.field)[i] or PALDevDataGet(FilterData.MeasurementIndex)++
 */
#define PALDevDataGet(Dev, field) (Dev->Data.field)

/**
 * @def PALDevDataSet(Dev, field, data)
 * @brief  Set ST private structure @a VL53LX_DevData_t data field
 * @param Dev       Device Handle
 * @param field     ST structure field name
 * @param data      Data to be set
 */
#define PALDevDataSet(Dev, field, data) (Dev->Data.field)=(data)



/**
 * @brief Writes the supplied byte buffer to the device
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[in]   pdata     : pointer to uint8_t (byte) buffer containing the data to be written
 * @param[in]   count     : number of bytes in the supplied byte buffer
 *
 * @return   VL53LX_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_WriteMulti(
		VL53LX_Dev_t *pdev,
		uint16_t      index,
		uint8_t      *pdata,
		uint32_t      count);

/**
 * @brief  Reads the requested number of bytes from the device
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[out]  pdata     : pointer to the uint8_t (byte) buffer to store read data
 * @param[in]   count     : number of bytes to read
 *
 * @return   VL53LX_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_ReadMulti(
		VL53LX_Dev_t *pdev,
		uint16_t      index,
		uint8_t      *pdata,
		uint32_t      count);

/**
 * Write single byte register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      8 bit register data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53LX_Error VL53LX_WrByte(VL53LX_DEV Dev, uint16_t index, uint8_t data);

/**
 * @brief  Reads a single byte from the device
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index
 * @param[out]  pdata     : pointer to uint8_t data value
 *
 * @return   VL53LX_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53LX_Error
 *
 */

VL53LX_Error VL53LX_RdByte(
		VL53LX_Dev_t *pdev,
		uint16_t      index,
		uint8_t      *pdata);


/**
 * @brief  Reads a single word (16-bit unsigned) from the device
 *
 * Manages the big-endian nature of the device (first byte read is the MS byte).
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[out]  pdata     : pointer to uint16_t data value
 *
 * @return   VL53LX_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_RdWord(
		VL53LX_Dev_t *pdev,
		uint16_t      index,
		uint16_t     *pdata);


/**
 * @brief  Writes a single word (16-bit unsigned) to the device
 *
 * Manages the big-endian nature of the device register map
 * (first byte written is the MS byte).
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[in]   data      : uin16_t data value write
 *
 * @return   VL53LX_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_WrWord(
		VL53LX_Dev_t *pdev,
		uint16_t      index,
		uint16_t      data);



/**
 * @brief  Implements a programmable wait in us
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   wait_us   : integer wait in micro seconds
 *
 * @return  VL53LX_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_WaitUs(
		VL53LX_Dev_t *pdev,
		int32_t       wait_us);


/**
 * @brief  Implements a programmable wait in ms
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   wait_ms   : integer wait in milliseconds
 *
 * @return  VL53LX_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_WaitMs(
		VL53LX_Dev_t *pdev,
		int32_t       wait_ms);




/**
 * Initialize PLatform Interface
 * @param   Dev       Device Handle
 * @return  None
 */
void VL53LX_init(VL53LX_DEV Dev);

/**
 * @defgroup VL53LX_registerAccess_group PAL Register Access Functions
 * @brief    PAL Register Access Functions
 *  @{
 */

/**
 * Lock comms interface to serialize all commands to a shared I2C interface for a specific device
 * @param   Dev       Device Handle
 * @return  VL53LX_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53LX_Error
 */
VL53LX_Error VL53LX_LockSequenceAccess(VL53LX_DEV Dev);

/**
 * Unlock comms interface to serialize all commands to a shared I2C interface for a specific device
 * @param   Dev       Device Handle
 * @return  VL53LX_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53LX_Error
 */
VL53LX_Error VL53LX_UnlockSequenceAccess(VL53LX_DEV Dev);



/*
 * @brief Gets current system tick count in [ms]
 *
 * @param[in]   pdev          : pointer to device structure (device handle)
 * @return  time_ms : current time in [ms]
 *
 * @return  VL53LX_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_GetTickCount(
		VL53LX_Dev_t *pdev,
		uint32_t *ptime_ms);




/**
 * @brief Register "wait for value" polling routine
 *
 * Port of the V2WReg Script function  WaitValueMaskEx()
 *
 * @param[in]   pdev          : pointer to device structure (device handle)
 * @param[in]   timeout_ms    : timeout in [ms]
 * @param[in]   index         : uint16_t register index value
 * @param[in]   value         : value to wait for
 * @param[in]   mask          : mask to be applied before comparison with value
 * @param[in]   poll_delay_ms : polling delay been each read transaction in [ms]
 *
 * @return  VL53LX_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_WaitValueMaskEx(
		VL53LX_Dev_t *pdev,
		uint32_t      timeout_ms,
		uint16_t      index,
		uint8_t       value,
		uint8_t       mask,
		uint32_t      poll_delay_ms);




/**
 * Threat safe Update (read/modify/write) single byte register
 *
 * Final_reg = (Initial_reg & and_data) |or_data
 *
 * @param   Dev        Device Handle
 * @param   index      The register index
 * @param   AndData    8 bit and data
 * @param   OrData     8 bit or data
 * @return  VL53LX_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53LX_Error
 */
VL53LX_Error VL53LX_UpdateByte(VL53LX_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData);

/** @} end of VL53LX_registerAccess_group */

    
/**
 * @brief execute delay in all polling API call
 *
 * A typical multi-thread or RTOs implementation is to sleep the task for some 5ms (with 100Hz max rate faster polling is not needed)
 * if nothing specific is need you can define it as an empty/void macro
 * @code
 * #define VL53LX_PollingDelay(...) (void)0
 * @endcode
 * @param Dev       Device Handle
 * @return  VL53LX_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53LX_Error
 */
VL53LX_Error VL53LX_PollingDelay(VL53LX_DEV Dev); /* usually best implemented as a real function */

/** @} end of VL53LX_platform_group */

#ifdef __cplusplus
}
#endif

#endif  /* _VL53LX_PLATFORM_H_ */



