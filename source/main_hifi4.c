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

#include "main_hifi4.h"
#include "drc_algorithms.h"
#include "drc_algorithms_cfg.h"
#include "algorithm_testbench.h"
#include "filters_cfg.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SAMPLE_RATE				48000U

#define APP_MU      			MUB
#define APP_MU_IRQn 			6U
#define PROC_NUM				3U
/* Channel transmit and receive register */
#define CHN_MU_REG_NUM 			0U
#define APP_SEMA42				SEMA42
#define SEMA42_GATE 			0U
#define BOOT_FLAG 				0x01U
#define SEMA42_LOCK_FLAG 		0x02U
#define SEMA42_UNLOCK_FLAG		0x03U
#define SEMA42_DSP_LOCK_FLAG 	0x04U
#define SEMA42_DSP_UNLOCK_FLAG 	0x05U

/*******************************************************************************
 * Type definitions
 ******************************************************************************/
enum {
	SRC_BUFFER_1_SEND,
	SRC_BUFFER_2_SEND,
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

#if (XCHAL_DCACHE_SIZE > 0)
AT_NONCACHEABLE_SECTION_ALIGN(volatile int32_t src_buffer_32_1[BUFFER_SIZE], 4) = {0.0f};
AT_NONCACHEABLE_SECTION_ALIGN(volatile int32_t src_buffer_32_2[BUFFER_SIZE], 4) = {0.0f};
AT_NONCACHEABLE_SECTION_ALIGN(volatile int32_t dst_buffer_32_1[BUFFER_SIZE], 4) = {0.0f};
AT_NONCACHEABLE_SECTION_ALIGN(volatile int32_t dst_buffer_32_2[BUFFER_SIZE], 4) = {0.0f};
#else
SDK_ALIGN(volatile int32_t src_buffer_32_1[BUFFER_SIZE], 4) = {0.0f};
SDK_ALIGN(volatile int32_t src_buffer_32_2[BUFFER_SIZE], 4) = {0.0f};
SDK_ALIGN(volatile int32_t dst_buffer_32_1[BUFFER_SIZE], 4) = {0.0f};
SDK_ALIGN(volatile int32_t dst_buffer_32_2[BUFFER_SIZE], 4) = {0.0f};
#endif

#if !Q31_USED
#if (XCHAL_DCACHE_SIZE > 0)
AT_NONCACHEABLE_SECTION_ALIGN(volatile float32_t src_buffer_f32_1[BUFFER_SIZE], 4) = {0.0f};
AT_NONCACHEABLE_SECTION_ALIGN(volatile float32_t src_buffer_f32_2[BUFFER_SIZE], 4) = {0.0f};
AT_NONCACHEABLE_SECTION_ALIGN(volatile float32_t dst_buffer_f32_1[BUFFER_SIZE], 4) = {0.0f};
AT_NONCACHEABLE_SECTION_ALIGN(volatile float32_t dst_buffer_f32_2[BUFFER_SIZE], 4) = {0.0f};
#else
SDK_ALIGN(volatile float32_t src_buffer_f32_1[BUFFER_SIZE], 4) = {0.0f};
SDK_ALIGN(volatile float32_t src_buffer_f32_2[BUFFER_SIZE], 4) = {0.0f};
SDK_ALIGN(volatile float32_t dst_buffer_f32_1[BUFFER_SIZE], 4) = {0.0f};
SDK_ALIGN(volatile float32_t dst_buffer_f32_2[BUFFER_SIZE], 4) = {0.0f};
#endif
#else
#if (XCHAL_DCACHE_SIZE > 0)
AT_NONCACHEABLE_SECTION_ALIGN(volatile q31_t src_buffer_q31_1[BUFFER_SIZE], 4) = {Q31_ZERO};
AT_NONCACHEABLE_SECTION_ALIGN(volatile q31_t src_buffer_q31_2[BUFFER_SIZE], 4) = {Q31_ZERO};
AT_NONCACHEABLE_SECTION_ALIGN(volatile q31_t dst_buffer_q31_1[BUFFER_SIZE], 4) = {Q31_ZERO};
AT_NONCACHEABLE_SECTION_ALIGN(volatile q31_t dst_buffer_q31_2[BUFFER_SIZE], 4) = {Q31_ZERO};
#else
SDK_ALIGN(volatile q31_t src_buffer_q31_1[BUFFER_SIZE], 4) = {Q31_ZERO};
SDK_ALIGN(volatile q31_t src_buffer_q31_2[BUFFER_SIZE], 4) = {Q31_ZERO};
SDK_ALIGN(volatile q31_t dst_buffer_q31_1[BUFFER_SIZE], 4) = {Q31_ZERO};
SDK_ALIGN(volatile q31_t dst_buffer_q31_2[BUFFER_SIZE], 4) = {Q31_ZERO};
#endif /* XCHAL_DCACHE_SIZE > 0 */
#endif /* Q31_USED */

volatile static int program_stage = SRC_BUFFER_1_SEND;

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

    if ((flag & kMU_Tx0EmptyFlag) == kMU_Tx0EmptyFlag)
    {
    	MU_ClearStatusFlags(APP_MU, kMU_Tx0EmptyFlag);
    	switch (program_stage)
    	{
			case SRC_BUFFER_1_SEND:
			{
				MU_SendMsgNonBlocking(APP_MU, CHN_MU_REG_NUM, (uint32_t)src_buffer_32_1);
				program_stage = SRC_BUFFER_2_SEND;
				break;
			}
			case SRC_BUFFER_2_SEND:
			{
				MU_SendMsgNonBlocking(APP_MU, CHN_MU_REG_NUM, (uint32_t)src_buffer_32_2);
				program_stage = DST_BUFFER_1_SEND;
				break;
			}
    		case DST_BUFFER_1_SEND:
    		{
    			MU_SendMsgNonBlocking(APP_MU, CHN_MU_REG_NUM, (uint32_t)dst_buffer_32_1);
    			program_stage = DST_BUFFER_2_SEND;
    			break;
    		}
    		case DST_BUFFER_2_SEND:
    		{
    			MU_DisableInterrupts(APP_MU, kMU_Tx0EmptyInterruptEnable);
    			MU_SendMsgNonBlocking(APP_MU, CHN_MU_REG_NUM, (uint32_t)dst_buffer_32_2);
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
	bool is_intA = false;

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

    calculate_coefficients();

    /* MUB init */
    MU_Init(APP_MU);

    SEMA42_Init(APP_SEMA42);

    init_fir_filter();
    init_iir_df1_filter();
	#if !Q31_USED
    init_iir_df2T_filter();
    #endif /* Q31_USED */

    /* Send flag to CM33 core to indicate DSP Core has startup */
    MU_SetFlags(APP_MU, BOOT_FLAG);

    /* Enable transmit and receive interrupt */
    MU_EnableInterrupts(APP_MU, (kMU_Tx0EmptyInterruptEnable));

    while (1)
    {

    	MU_SetFlags(APP_MU, SEMA42_DSP_LOCK_FLAG);

		while (SEMA42_UNLOCK_FLAG != MU_GetFlags(APP_MU))
		{
		}

    	SEMA42_Lock(APP_SEMA42, SEMA42_GATE, PROC_NUM);

		if (!is_intA)
		{
			#if Q31_USED
			for (int i = 0, j = 0, k = (BUFFER_SIZE/2); i < BUFFER_SIZE; i += 2, ++j, ++k)
			{
				src_buffer_q31_1[j] = (q31_t)src_buffer_32_1[i];
				src_buffer_q31_1[k] = (q31_t)src_buffer_32_1[i+1];
			}
			#else
			for (int i = 0, j = 0, k = (BUFFER_SIZE/2); i < BUFFER_SIZE; i += 2, ++j, ++k)
			{
				src_buffer_f32_1[j] = (float32_t)src_buffer_32_1[i];
				src_buffer_f32_1[k] = (float32_t)src_buffer_32_1[i+1];
			}
			#endif

			#if Q31_USED
//					fir_process_batch((q31_t *)src_buffer_q31_1, (q31_t *)dst_buffer_q31_1);
//					iir_df1_process_batch((q31_t *)src_buffer_q31_1, (q31_t *)dst_buffer_q31_1);
			drc_full_stereo_balanced((q31_t *)src_buffer_q31_1, (q31_t *)dst_buffer_q31_1);
			#else
//					fir_process_batch((float32_t *)src_buffer_f32_1, (float32_t *)dst_buffer_f32_1);
//					iir_df1_process_batch((float32_t *)src_buffer_f32_1, (float32_t *)dst_buffer_f32_1);
//					iir_df2T_process_batch((float32_t *)src_buffer_f32_1, (float32_t *)dst_buffer_f32_1);
//			drc_full_stereo_balanced((float32_t *)src_buffer_f32_1, (float32_t *)dst_buffer_f32_1);
			#endif

			#if Q31_USED
			for (int i = 0, j = 0, k = (BUFFER_SIZE/2); i < BUFFER_SIZE; i += 2, ++j, ++k)
			{
				dst_buffer_32_1[i] = (int32_t)dst_buffer_q31_1[j];
				dst_buffer_32_1[i+1] = (int32_t)dst_buffer_q31_1[k];
			}
			#else
			for (int i = 0, j = 0, k = (BUFFER_SIZE/2); i < BUFFER_SIZE; i += 2, ++j, ++k)
			{
				dst_buffer_32_1[i] = (int32_t)dst_buffer_f32_1[j];
				dst_buffer_32_1[i+1] = (int32_t)dst_buffer_f32_1[k];
			}
			#endif
		}
		else
		{
			#if Q31_USED
			for (int i = 0, j = 0, k = (BUFFER_SIZE/2); i < BUFFER_SIZE; i += 2, ++j, ++k)
			{
				src_buffer_q31_2[j] = (q31_t)src_buffer_32_2[i];
				src_buffer_q31_2[k] = (q31_t)src_buffer_32_2[i+1];
			}
			#else
			for (int i = 0, j = 0, k = (BUFFER_SIZE/2); i < BUFFER_SIZE; i += 2, ++j, ++k)
			{
				src_buffer_f32_2[j] = (float32_t)src_buffer_32_2[i];
				src_buffer_f32_2[k] = (float32_t)src_buffer_32_2[i+1];
			}
			#endif

			#if Q31_USED
//					fir_process_batch((q31_t *)src_buffer_q31_2, (q31_t *)dst_buffer_q31_2);
//					iir_df1_process_batch((q31_t *)src_buffer_q31_2, (q31_t *)dst_buffer_q31_2);
			drc_full_stereo_balanced((q31_t *)src_buffer_q31_2, (q31_t *)dst_buffer_q31_2);
			#else
//					fir_process_batch((float32_t *)src_buffer_f32_2, (float32_t *)dst_buffer_f32_2);
//					iir_df1_process_batch((float32_t *)src_buffer_f32_2, (float32_t *)dst_buffer_f32_2);
//					iir_df2T_process_batch((float32_t *)src_buffer_f32_2, (float32_t *)dst_buffer_f32_2);
//			drc_full_stereo_balanced((float32_t *)src_buffer_f32_2, (float32_t *)dst_buffer_f32_2);
			#endif

			#if Q31_USED
			for (int i = 0, j = 0, k = (BUFFER_SIZE/2); i < BUFFER_SIZE; i += 2, ++j, ++k)
			{
				dst_buffer_32_2[i] = (int32_t)dst_buffer_q31_2[j];
				dst_buffer_32_2[i+1] = (int32_t)dst_buffer_q31_2[k];
			}
			#else
			for (int i = 0, j = 0, k = (BUFFER_SIZE/2); i < BUFFER_SIZE; i += 2, ++j, ++k)
			{
				dst_buffer_32_2[i] = (int32_t)dst_buffer_f32_2[j];
				dst_buffer_32_2[i+1] = (int32_t)dst_buffer_f32_2[k];
			}
			#endif
		}

		MU_SetFlags(APP_MU, SEMA42_DSP_UNLOCK_FLAG);
		SEMA42_Unlock(APP_SEMA42, SEMA42_GATE);

		while (SEMA42_LOCK_FLAG != MU_GetFlags(APP_MU))
		{
		}

		is_intA = (is_intA == true ? false : true);
    }
}
