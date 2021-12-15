/*
MIT License

Copyright (c) 2017 John Bryan Moore

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include "vl53lx_platform.h"
#include "vl53lx_api.h"



#define  VL53LX_COMMS_CHUNK_SIZE  56


// calls read_i2c_block_data(address, reg, length)
static int (*i2c_read_func)(uint8_t address, uint16_t reg,
                    uint8_t *list, uint8_t length) = NULL;

// calls write_i2c_block_data(address, reg, list)
static int (*i2c_write_func)(uint8_t address, uint16_t reg,
                    uint8_t *list, uint8_t length) = NULL;

//static pthread_mutex_t i2c_mutex = PTHREAD_MUTEX_INITIALIZER; 

void VL53LX_init(VL53LX_DEV Dev)
{
}

void VL53LX_set_i2c(void *read_func, void *write_func)
{
    i2c_read_func = read_func;
    i2c_write_func = write_func;
}

static int i2c_write(VL53LX_DEV Dev, uint8_t cmd,
                    uint8_t *data, uint8_t len)
{
    int result = VL53LX_ERROR_NONE;

    if (i2c_write_func != NULL)
    {
        if (result == VL53LX_ERROR_NONE)
        {
            if (i2c_write_func(Dev->i2c_slave_address, cmd, data, len) < 0)
            {
                result = VL53LX_ERROR_CONTROL_INTERFACE;
            }
        }

  
    }
    else
    {
        printf("i2c bus write not set.\n");
        result = VL53LX_ERROR_CONTROL_INTERFACE;
    }
    
    return result;
}

static int i2c_read(VL53LX_DEV Dev, uint8_t cmd,
                    uint8_t * data, uint8_t len)
{
    int result = VL53LX_ERROR_NONE;

    if (i2c_read_func != NULL)
    {
        if (result == VL53LX_ERROR_NONE)
        {
            if (i2c_read_func(Dev->i2c_slave_address, cmd, data, len) < 0)
            {
                result =  VL53LX_ERROR_CONTROL_INTERFACE;
            }
        }

      
    }
    else
    {
        printf("i2c bus read not set.\n");
        result =  VL53LX_ERROR_CONTROL_INTERFACE;
    }
    
    return result;
}

VL53LX_Error VL53LX_LockSequenceAccess(VL53LX_DEV Dev)
{
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    return Status;
}

VL53LX_Error VL53LX_UnlockSequenceAccess(VL53LX_DEV Dev)
{
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    return Status;
}

VL53LX_Error VL53LX_WriteMulti(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata,
	uint32_t      count)
{
	return i2c_write(pdev, index, pdata, count);
}


VL53LX_Error VL53LX_ReadMulti(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata,
	uint32_t      count)
{
	 return i2c_read(pdev, index, pdata, count);
}




VL53LX_Error VL53LX_WrByte(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint8_t       VL53LX_p_003)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint8_t  buffer[1];


	buffer[0] = (uint8_t)(VL53LX_p_003);

	status = VL53LX_WriteMulti(pdev, index, buffer, 1);

	return status;
}

VL53LX_Error VL53LX_WrWord(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint16_t      VL53LX_p_003)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint8_t  buffer[2];


	buffer[0] = (uint8_t)(VL53LX_p_003 >> 8);
	buffer[1] = (uint8_t)(VL53LX_p_003 &  0x00FF);

	status = VL53LX_WriteMulti(pdev, index, buffer, VL53LX_BYTES_PER_WORD);

	return status;
}

VL53LX_Error VL53LX_WrDWord(VL53LX_DEV Dev, uint8_t index, uint32_t data)
{
    uint8_t buf[4];
    buf[3] = data>>0&0xFF;
    buf[2] = data>>8&0xFF;
    buf[1] = data>>16&0xFF;
    buf[0] = data>>24&0xFF;
    return i2c_write(Dev, index, buf, 4);
}

VL53LX_Error VL53LX_UpdateByte(VL53LX_DEV Dev, uint8_t index,
                                uint8_t AndData, uint8_t OrData)
{

    int32_t status_int;
    uint8_t data;

    status_int = i2c_read(Dev, index, &data, 1);

    if (status_int != 0)
    {
        return  status_int;
    }

    data = (data & AndData) | OrData;
    return i2c_write(Dev, index, &data, 1);
}

VL53LX_Error VL53LX_RdByte(VL53LX_DEV Dev, uint16_t index, uint8_t *data)
{
    uint8_t tmp = 0;
    int ret = i2c_read(Dev, index, &tmp, 1);
    *data = tmp;
    // printf("%u\n", tmp);
    return ret;
}

VL53LX_Error VL53LX_RdWord(VL53LX_DEV Dev, uint16_t index, uint16_t *data)
{
    uint8_t buf[2];
    int ret = i2c_read(Dev, index, buf, 2);
    uint16_t tmp = 0;
    tmp |= buf[1]<<0;
    tmp |= buf[0]<<8;
    // printf("%u\n", tmp);
    *data = tmp;
    return ret;
}

VL53LX_Error  VL53LX_RdDWord(VL53LX_DEV Dev, uint8_t index, uint32_t *data)
{
    uint8_t buf[4];
    int ret = i2c_read(Dev, index, buf, 4);
    uint32_t tmp = 0;
    tmp |= buf[3]<<0;
    tmp |= buf[2]<<8;
    tmp |= buf[1]<<16;
    tmp |= buf[0]<<24;
    *data = tmp;
    // printf("%zu\n", tmp);
    return ret;
}

VL53LX_Error VL53LX_PollingDelay(VL53LX_DEV Dev)
{
    usleep(5000);
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WaitUs(
		VL53LX_Dev_t *pdev,
		int32_t       wait_us){
    usleep(wait_us);
    return VL53LX_ERROR_NONE;
}


VL53LX_Error VL53LX_WaitMs(
	VL53LX_Dev_t *pdev,
	int32_t       wait_ms)
{
	return VL53LX_WaitUs(pdev, wait_ms * 1000);
}





VL53LX_Error VL53LX_GetTickCount(
  VL53LX_Dev_t* pdev,
  uint32_t* ptime_ms) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  *ptime_ms = (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000LL);
  return VL53LX_ERROR_NONE;
}


VL53LX_Error VL53LX_WaitValueMaskEx(
	VL53LX_Dev_t *pdev,
	uint32_t      timeout_ms,
	uint16_t      index,
	uint8_t       value,
	uint8_t       mask,
	uint32_t      poll_delay_ms)
{


	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint32_t     start_time_ms   = 0;
	uint32_t     current_time_ms = 0;
	uint8_t      byte_value      = 0;
	uint8_t      found           = 0;
#ifdef VL53LX_LOG_ENABLE
	uint32_t     trace_functions = 0;
#endif

	//_LOG_STRING_BUFFER(register_name);

	SUPPRESS_UNUSED_WARNING(poll_delay_ms);

#ifdef VL53LX_LOG_ENABLE

	VL53LX_get_register_name(
			index,
			register_name);


	trace_i2c("WaitValueMaskEx(%5d, %s, 0x%02X, 0x%02X, %5d);\n",
		timeout_ms, register_name, value, mask, poll_delay_ms);
#endif



	VL53LX_GetTickCount(pdev, &start_time_ms);
	pdev->new_data_ready_poll_duration_ms = 0;



#ifdef VL53LX_LOG_ENABLE
	trace_functions = _LOG_GET_TRACE_FUNCTIONS();
#endif
	_LOG_SET_TRACE_FUNCTIONS(VL53LX_TRACE_FUNCTION_NONE);



	while ((status == VL53LX_ERROR_NONE) &&
		   (pdev->new_data_ready_poll_duration_ms < timeout_ms) &&
		   (found == 0))
	{
		status = VL53LX_RdByte(
						pdev,
						index,
						&byte_value);

		if ((byte_value & mask) == value)
		{
			found = 1;
		}




		VL53LX_GetTickCount(pdev, &current_time_ms);
		pdev->new_data_ready_poll_duration_ms = current_time_ms - start_time_ms;
	}


	_LOG_SET_TRACE_FUNCTIONS(trace_functions);

	if (found == 0 && status == VL53LX_ERROR_NONE)
		status = VL53LX_ERROR_TIME_OUT;

	return status;
}
