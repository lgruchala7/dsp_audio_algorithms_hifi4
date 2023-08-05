/*
 * Copyright 2018-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <xtensa/config/core.h>
#include <xtensa/xos.h>

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"

#include "pin_mux.h"
#include "dsp_config.h"
#include "board_hifi4.h"
#include "fsl_inputmux.h"
#include "fsl_mu.h"
#include "fsl_sema42.h"

#include <time.h>

#include "dsp_algorithms_hifi4.h"
#include "algorithm_testbench.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
//#define BOARD_XTAL_SYS_CLK_HZ 24000000U /*!< Board xtal_sys frequency in Hz */
//#define BOARD_XTAL32K_CLK_HZ  32768U    /*!< Board xtal32K frequency in Hz */

#define BUFFER_SIZE		160U
#define SAMPLE_RATE		48000U

#define APP_MU      	MUB
#define APP_MU_IRQn 	6
#define PROC_NUM		3U
/* Channel transmit and receive register */
#define CHN_MU_REG_NUM 			0U
#define APP_SEMA42				SEMA42
#define SEMA42_GATE 			0U
#define BOOT_FLAG 				0x01U
#define SEMA42_LOCK_FLAG 		0x02U
#define SEMA42_DSP_LOCK_FLAG 	0x04U
#define SEMA42_DSP_UNLOCK_FLAG 	0x05U
/* How many message is used to test message sending */
#define MSG_LENGTH 		1U

#define ITER_COUNT		100
/*******************************************************************************
 * Type definitions
 ******************************************************************************/
enum {
	SRC_BUFFER_1_RCV,
	SRC_BUFFER_2_RCV,
	DST_BUFFER_1_SEND,
	DST_BUFFER_2_SEND,
	RUN,
	STAGES_MAX,
};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void APP_MU_IRQHandler_0();

/*******************************************************************************
 * Variables
 ******************************************************************************/
extern int NonCacheable_start, NonCacheable_end;
extern int NonCacheable_init_start, NonCacheable_init_end;

uint32_t num_blocks = (BUFFER_SIZE / BLOCK_SIZE / 2);
#ifndef Q31
arm_fir_instance_f32 fir_instance_f32_1;
arm_fir_instance_f32 fir_instance_f32_2;
volatile float32_t * src_shared_buffer_f32_1 = NULL;
volatile float32_t * src_shared_buffer_f32_2 = NULL;
#if (XCHAL_DCACHE_SIZE > 0)
AT_NONCACHEABLE_SECTION_ALIGN(float32_t dst_shared_buffer_f32_1[BUFFER_SIZE], 4) = {0.0f};
AT_NONCACHEABLE_SECTION_ALIGN(float32_t dst_shared_buffer_f32_2[BUFFER_SIZE], 4) = {0.0f};
#else
SDK_ALIGN(float32_t dst_shared_buffer_f32_1[BUFFER_SIZE], 4) = {0.0f};
SDK_ALIGN(float32_t dst_shared_buffer_f32_2[BUFFER_SIZE], 4) = {0.0f};
#endif
#else
arm_fir_instance_q31 fir_instance_q31_1;
arm_fir_instance_q31 fir_instance_q31_2;
volatile q31_t * src_shared_buffer_q31_1 = NULL;
volatile q31_t * src_shared_buffer_q31_2 = NULL;
#if (XCHAL_DCACHE_SIZE > 0)
AT_NONCACHEABLE_SECTION_ALIGN(q31_t dst_shared_buffer_q31_1[BUFFER_SIZE], 4) = {0.0f};
AT_NONCACHEABLE_SECTION_ALIGN(q31_t dst_shared_buffer_q31_2[BUFFER_SIZE], 4) = {0.0f};
#else
SDK_ALIGN(float32_t dst_shared_buffer_q31_1[BUFFER_SIZE], 4) = {0.0f};
SDK_ALIGN(float32_t dst_shared_buffer_q31_2[BUFFER_SIZE], 4) = {0.0f};
#endif
#endif

volatile static int program_stage = SRC_BUFFER_1_RCV;
static bool is_intA = false;

/*******************************************************************************
 * Code
 ******************************************************************************/
static void XOS_Init(void)
{
    xos_set_clock_freq(XOS_CLOCK_FREQ);
    xos_start_system_timer(-1, 0);

    /* DSP interrupt only can be enable after XOS is started. */
	xos_register_interrupt_handler(APP_MU_IRQn, APP_MU_IRQHandler_0, NULL);
	xos_interrupt_enable(APP_MU_IRQn);
}

void APP_MU_IRQHandler_0(void)
{
    uint32_t flag = MU_GetStatusFlags(APP_MU);

    if ((flag & kMU_Rx0FullFlag) == kMU_Rx0FullFlag)
    {
    	MU_ClearStatusFlags(APP_MU, kMU_Rx0FullFlag);
    	switch (program_stage)
    	{
    		case SRC_BUFFER_1_RCV:
    		{
				#ifndef Q31
				src_shared_buffer_f32_1 = (float32_t *)MU_ReceiveMsgNonBlocking(APP_MU, CHN_MU_REG_NUM);
				#else
				src_shared_buffer_q31_1 = (q31_t *)MU_ReceiveMsgNonBlocking(APP_MU, CHN_MU_REG_NUM);
				#endif
				program_stage = SRC_BUFFER_2_RCV;
				break;
    		}
    		case SRC_BUFFER_2_RCV:
    		{
				MU_DisableInterrupts(APP_MU, kMU_Rx0FullInterruptEnable);
				#ifndef Q31
				src_shared_buffer_f32_2 = (float32_t *)MU_ReceiveMsgNonBlocking(APP_MU, CHN_MU_REG_NUM);
				#else
				src_shared_buffer_q31_2 = (q31_t *)MU_ReceiveMsgNonBlocking(APP_MU, CHN_MU_REG_NUM);
				#endif
				MU_EnableInterrupts(APP_MU, kMU_Tx0EmptyInterruptEnable);
				program_stage = DST_BUFFER_1_SEND;
				break;
    		}
    		default:
    		{
    			PRINTF("Program flow error - unexpected interrupt\r\nExiting with code -3\r\n");
    			exit(-3);
    			break;
    		}
    	}
    }
    else if ((flag & kMU_Tx0EmptyFlag) == kMU_Tx0EmptyFlag)
    {
    	MU_ClearStatusFlags(APP_MU, kMU_Tx0EmptyFlag);
    	switch (program_stage)
    	{
    		case DST_BUFFER_1_SEND:
    		{
    			#ifndef Q31
    			MU_SendMsgNonBlocking(APP_MU, CHN_MU_REG_NUM, (uint32_t)dst_shared_buffer_f32_1);
    			#else
    			MU_SendMsgNonBlocking(APP_MU, CHN_MU_REG_NUM, (uint32_t)dst_shared_buffer_q31_1);
    			#endif
    			program_stage = DST_BUFFER_2_SEND;
    			break;
    		}
    		case DST_BUFFER_2_SEND:
    		{
    			MU_DisableInterrupts(APP_MU, kMU_Tx0EmptyInterruptEnable);
    			#ifndef Q31
    			MU_SendMsgNonBlocking(APP_MU, CHN_MU_REG_NUM, (uint32_t)dst_shared_buffer_f32_2);
    			#else
    			MU_SendMsgNonBlocking(APP_MU, CHN_MU_REG_NUM, (uint32_t)dst_shared_buffer_q31_2);
    			#endif
    			program_stage = RUN;
    			break;
    		}
    		default:
    		{
    			PRINTF("Program flow error - unexpected interrupt\r\nExiting with code -4\r\n");
    			exit(-4);
    			break;
    		}
    	}
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
    /* Disable DSP cache for noncacheable sections. */
    xthal_set_region_attribute((uint32_t *)&NonCacheable_start,
                               (uint32_t)&NonCacheable_end - (uint32_t)&NonCacheable_start, XCHAL_CA_BYPASS, 0);
    xthal_set_region_attribute((uint32_t *)&NonCacheable_init_start,
                               (uint32_t)&NonCacheable_init_end - (uint32_t)&NonCacheable_init_start, XCHAL_CA_BYPASS, 0);

    xos_start_main("main", 7, 0);
    INPUTMUX_Init(INPUTMUX);
    BOARD_InitBootPins();
    BOARD_InitDebugConsole();
    XOS_Init();

    check_coefficients();
    calculate_coefficients();

    /* MUB init */
    MU_Init(APP_MU);

    SEMA42_Init(APP_SEMA42);

	#ifndef Q31
    arm_fir_init_f32(&fir_instance_f32_1, FIR_COEFF_COUNT, (float32_t *)&fir_filter_coeff_f32[0], &fir_state_f32_1[0], BLOCK_SIZE);
    arm_fir_init_f32(&fir_instance_f32_2, FIR_COEFF_COUNT, (float32_t *)&fir_filter_coeff_f32[0], &fir_state_f32_2[0], BLOCK_SIZE);
	#else
    arm_float_to_q31(fir_filter_coeff_f32, fir_filter_coeff_q31, FIR_COEFF_COUNT);
    arm_fir_init_q31(&fir_instance_q31_1, FIR_COEFF_COUNT, (q31_t *)&fir_filter_coeff_q31[0], &fir_state_q31_1[0], BLOCK_SIZE);
    arm_fir_init_q31(&fir_instance_q31_2, FIR_COEFF_COUNT, (q31_t *)&fir_filter_coeff_q31[0], &fir_state_q31_2[0], BLOCK_SIZE);
	#endif

    /* Send flag to CM33 core to indicate DSP Core has startup */
    MU_SetFlags(APP_MU, BOOT_FLAG);

    /* Enable transmit and receive interrupt */
    MU_EnableInterrupts(APP_MU, (kMU_Rx0FullInterruptEnable));

    while (1)
    {

		while (SEMA42_LOCK_FLAG != MU_GetFlags(APP_MU))
		{
		}

    	SEMA42_Lock(APP_SEMA42, SEMA42_GATE, PROC_NUM);
    	MU_SetFlags(APP_MU, SEMA42_DSP_LOCK_FLAG);

		for (uint32_t j = 0; j < num_blocks; ++j)
		{
			#ifndef Q31
			if (!is_intA)
			{
				arm_fir_f32(&fir_instance_f32_1, (float32_t *)&src_shared_buffer_f32_1[0] + (j * BLOCK_SIZE), &dst_shared_buffer_f32_1[0] + (j * BLOCK_SIZE), BLOCK_SIZE);
				arm_fir_f32(&fir_instance_f32_2, (float32_t *)&src_shared_buffer_f32_1[BUFFER_SIZE/2] + (j * BLOCK_SIZE), &dst_shared_buffer_f32_1[BUFFER_SIZE/2] + (j * BLOCK_SIZE), BLOCK_SIZE);
			}
			else
			{
				arm_fir_f32(&fir_instance_f32_1, (float32_t *)&src_shared_buffer_f32_2[0] + (j * BLOCK_SIZE), &dst_shared_buffer_f32_2[0] + (j * BLOCK_SIZE), BLOCK_SIZE);
				arm_fir_f32(&fir_instance_f32_2, (float32_t *)&src_shared_buffer_f32_2[BUFFER_SIZE/2] + (j * BLOCK_SIZE), &dst_shared_buffer_f32_2[BUFFER_SIZE/2] + (j * BLOCK_SIZE), BLOCK_SIZE);
			}
			#else
			if (is_intA)
			{
				arm_fir_q31(&fir_instance_q31_1, (q31_t *)&src_shared_buffer_q31_1 + (j * BLOCK_SIZE), &dst_shared_buffer_q31_1 + (j * BLOCK_SIZE), BLOCK_SIZE);
				arm_fir_q31(&fir_instance_q31_2, (q31_t *)&src_shared_buffer_q31_1 + (j * BLOCK_SIZE), &dst_shared_buffer_q31_1 + (j * BLOCK_SIZE), BLOCK_SIZE);
			}
			else
			{
				arm_fir_q31(&fir_instance_q31_1, (q31_t *)&src_shared_buffer_q31_2[0] + (j * BLOCK_SIZE), &dst_shared_buffer_q31_2[0] + (j * BLOCK_SIZE), BLOCK_SIZE);
				arm_fir_q31(&fir_instance_q31_2, (q31_t *)&src_shared_buffer_q31_2[BUFFER_SIZE/2] + (j * BLOCK_SIZE), &dst_shared_buffer_q31_2[BUFFER_SIZE/2] + (j * BLOCK_SIZE), BLOCK_SIZE);
			}
			#endif
		}

		MU_SetFlags(APP_MU, SEMA42_DSP_UNLOCK_FLAG);
		SEMA42_Unlock(APP_SEMA42, SEMA42_GATE);

		is_intA = (is_intA == true ? false : true);
    }
}
