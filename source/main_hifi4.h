/*
 * main_hifi4.h
 *
 *  Created on: 9 wrz 2023
 *      Author: £ukasz
 */

#ifndef _MAIN_CM33_H_
#define _MAIN_CM33_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fsl_device_registers.h"
#include "fsl_mu.h"
#include "fsl_sema42.h"
#include "arm_math.h"
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define Q31_USED	1

/* Q31 constants */
#define Q31_ONE					0x7fffffff
#define Q31_MIN_ONE				0x80000000
#define Q31_ONE_F32				2147483647.0f
#define Q31_ZERO				0x00000000

//#define APP_MU            		MUA
//#define APP_SEMA42          	SEMA42
//#define SEMA42_GATE 			0U
///* Flag indicates Core Boot Up*/
//#define SEMA42_LOCK_FLAG 		0x02U
//#define SEMA42_UNLOCK_FLAG		0x03U
//#define SEMA42_DSP_LOCK_FLAG	0x04U
///* Channel transmit and receive register */
//#define CHN_MU_REG_NUM 			0U
//#define PROC_NUM				1U
/* Receive/transmit buffer size */
#define BUFFER_SIZE				512

#define IS_IN_RANGE_INCL(arg, low, up)	(((arg) >= (low) && (arg) <= (up)) ? true : false)

/*******************************************************************************
 * Type definitions
 ******************************************************************************/
enum {
	CHANNEL_LEFT,
	CHANNEL_RIGHT,
	CHANNEL_CNT,
};

/*******************************************************************************
 * Exported variables
 ******************************************************************************/
extern float32_t scale_down_factor;
extern float32_t scale_up_factor;

#endif /* _MAIN_CM33_H_ */
