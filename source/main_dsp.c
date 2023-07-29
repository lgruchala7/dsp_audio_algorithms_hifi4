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

#define BUFFER_SIZE		320U
#define SAMPLE_RATE		48000U

#define APP_MU      	MUB
#define APP_MU_IRQn 	6
/* Channel transmit and receive register */
#define CHN_MU_REG_NUM 	0U
#define APP_SEMA42		SEMA42
#define SEMA42_GATE 	0U
#define BOOT_FLAG 		0x01U
#define SEMA42_LOCK_FLAG 0x02U
#define SEMA42_DSP_LOCK_FLAG 0x03U
/* How many message is used to test message sending */
#define MSG_LENGTH 		1U

#define ITER_COUNT		100
/*******************************************************************************
 * Type definitions
 ******************************************************************************/
enum {
	SRC_BUFFER_RCV,
	DST_BUFFER_RCV,
	RUN,
	STAGES_MAX,
};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void APP_MU_IRQHandler_0();
void APP_MU_IRQHandler_1();
//static void measure_algorithm_time_16(void (*algorithm_func)(uint16_t *, size_t), uint16_t * buffer, size_t buffer_size, uint32_t iterations);
static void test_cmsis_dsp(float32_t * src_arr, float32_t * dst_arr, uint32_t arr_size, uint32_t fs, uint32_t iterations);
static void test_drc_algorithm(int16_t * src_arr, int16_t * dst_arr, uint32_t arr_size, uint32_t fs, uint32_t iterations);

/*******************************************************************************
 * Variables
 ******************************************************************************/
extern int NonCacheable_start, NonCacheable_end;
extern int NonCacheable_init_start, NonCacheable_init_end;

#if (XCHAL_DCACHE_SIZE > 0)
AT_NONCACHEABLE_SECTION_ALIGN(static int16_t src_buffer[BUFFER_SIZE], 4);
AT_NONCACHEABLE_SECTION_ALIGN(static int16_t dst_buffer[BUFFER_SIZE], 4);
#else
//SDK_ALIGN(static uint8_t s_buffer[BUFFER_SIZE * BUFFER_NUM], 4);
#endif
static float32_t src_test_arr_f32[BUFFER_SIZE] = {0.0f};
static float32_t dst_test_arr_f32[BUFFER_SIZE] = {0.0f};
static int16_t src_test_arr_16[BUFFER_SIZE] = {0};
static int16_t dst_test_arr_16[BUFFER_SIZE] = {0};

volatile float32_t * src_shared_buffer = NULL;
volatile float32_t * dst_shared_buffer = NULL;

arm_fir_instance_f32 fir_instance;
uint32_t num_blocks = BUFFER_SIZE / BLOCK_SIZE;

volatile static int start_processing = 0;
static uint32_t msg_recv[MSG_LENGTH];
volatile uint32_t cur_send = 0U;
volatile uint32_t cur_recv = 0U;
volatile static int program_stage = SRC_BUFFER_RCV;
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

/*!
 * @brief Function to clear the msg_recv array.
 * This function set msg_recv to be 0.
 */
static void clear_msg_recv(void)
{
    for (uint32_t i = 0U; i < MSG_LENGTH; i++)
    {
        msg_recv[i] = 0U;
    }
}

void APP_MU_IRQHandler_0(void)
{
    uint32_t flag = MU_GetStatusFlags(APP_MU);

    if ((flag & kMU_Rx0FullFlag) == kMU_Rx0FullFlag)
    {
        switch (program_stage)
        {
        	case SRC_BUFFER_RCV:
        		src_shared_buffer = (float32_t *)MU_ReceiveMsgNonBlocking(APP_MU, CHN_MU_REG_NUM);
        		program_stage = DST_BUFFER_RCV;
        		break;
        	case DST_BUFFER_RCV:
        		dst_shared_buffer = (float32_t *)MU_ReceiveMsgNonBlocking(APP_MU, CHN_MU_REG_NUM);
        		program_stage = RUN;
        	    MU_EnableInterrupts(APP_MU, kMU_Tx0EmptyInterruptEnable);
				MU_DisableInterrupts(APP_MU, kMU_Rx0FullInterruptEnable);
        		break;
        	case RUN:
        	default:
        		exit(-2);
        		break;
        }
    }
    else if (((flag & kMU_Tx0EmptyFlag) == kMU_Tx0EmptyFlag))
    {
        switch (program_stage)
        {
        	case RUN:
        		MU_SendMsgNonBlocking(APP_MU, CHN_MU_REG_NUM, 1U);
				MU_DisableInterrupts(APP_MU, kMU_Tx0EmptyInterruptEnable);
        		break;
        	case SRC_BUFFER_RCV:
        	case DST_BUFFER_RCV:
        	default:
        		exit(-2);
        		break;
        }
    }
}

void APP_MU_IRQHandler_1(void)
{
    uint32_t flag = MU_GetStatusFlags(APP_MU);

    if ((flag & kMU_Rx0FullFlag) == kMU_Rx0FullFlag)
    {
        if (cur_recv < MSG_LENGTH)
        {
            msg_recv[cur_recv++] = MU_ReceiveMsgNonBlocking(APP_MU, CHN_MU_REG_NUM);
        }
        else
        {
            MU_DisableInterrupts(APP_MU, kMU_Rx0FullInterruptEnable);
        }
    }
    if (((flag & kMU_Tx0EmptyFlag) == kMU_Tx0EmptyFlag) && (cur_recv == MSG_LENGTH))
    {
        if ((0 == start_processing) && (MSG_LENGTH > cur_send))
        {
        	start_processing = 1;
        	MU_DisableInterrupts(APP_MU, kMU_Tx0EmptyInterruptEnable);
        }
        else if (MSG_LENGTH == cur_send)
        {
            MU_DisableInterrupts(APP_MU, kMU_Tx0EmptyInterruptEnable);
        }
        else
        {
        	PRINTF("\r\nError: The program should never reach this statement\r\n");
        	exit(-1);
        }
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
//    /* Disable DSP cache for noncacheable sections. */
//    xthal_set_region_attribute((uint32_t *)&NonCacheable_start,
//                               (uint32_t)&NonCacheable_end - (uint32_t)&NonCacheable_start, XCHAL_CA_BYPASS, 0);
//    xthal_set_region_attribute((uint32_t *)&NonCacheable_init_start,
//                               (uint32_t)&NonCacheable_init_end - (uint32_t)&NonCacheable_init_start, XCHAL_CA_BYPASS,
//                               0);

    xos_start_main("main", 7, 0);
    INPUTMUX_Init(INPUTMUX);
    BOARD_InitBootPins();
    BOARD_InitDebugConsole();
    XOS_Init();

    check_coefficients();

//    /* Map DMA IRQ handler to INPUTMUX selection DSP_INT0_SEL18
//     * EXTINT19 = DSP INT 23 */
//    xos_register_interrupt_handler(XCHAL_EXTINT19_NUM, (XosIntFunc *)CTIMER1_IRQHandler, NULL);
//    xos_interrupt_enable(XCHAL_EXTINT19_NUM);

    calculate_coefficients();
//    BOARD_InitClock();

    /* MUB init */
    MU_Init(APP_MU);

    /* Clear the msg_recv array before receive */
    clear_msg_recv();

    SEMA42_Init(APP_SEMA42);

    arm_fir_init_f32(&fir_instance, FIR_COEFF_COUNT, (float32_t *)&fir_filter_coeff[0], &fir_state[0], block_size);

    /* Send flag to CM33 core to indicate DSP Core has startup */
    MU_SetFlags(APP_MU, BOOT_FLAG);

    /* Enable transmit and receive interrupt */
    MU_EnableInterrupts(APP_MU, (kMU_Rx0FullInterruptEnable));


//    test_cmsis_dsp(src_test_arr_f32, dst_test_arr_f32, BUFFER_SIZE, SAMPLE_RATE, ITER_COUNT);
//    test_drc_algorithm(src_test_arr_16, dst_test_arr_16, BUFFER_SIZE, SAMPLE_RATE, ITER_COUNT);
//    MU_SendMsgNonBlocking(APP_MU, CHN_MU_REG_NUM, msg_recv[cur_send++]);
//    start_processing = 0;
//    MU_EnableInterrupts(APP_MU, (kMU_Tx0EmptyInterruptEnable));

    while (1)
    {
		while (SEMA42_LOCK_FLAG != MU_GetFlags(APP_MU))
		{
		}

    	SEMA42_Lock(APP_SEMA42, SEMA42_GATE, (uint8_t)3U);
    	MU_SetFlags(APP_MU, SEMA42_DSP_LOCK_FLAG);

		for (uint32_t j = 0; j < num_blocks; j++)
		{
			arm_fir_f32(&fir_instance, src_shared_buffer + (j * block_size), dst_shared_buffer + (j * block_size), block_size);
		}
		print_buffer_data_f32(dst_shared_buffer, num_blocks * block_size);
		SEMA42_Unlock(APP_SEMA42, SEMA42_GATE);
    }
}

static void test_cmsis_dsp(float32_t * src_arr, float32_t * dst_arr, uint32_t arr_size, uint32_t fs, uint32_t iterations)
{
//	arm_fir_instance_f32 fir_instance;
//	uint32_t num_blocks = arr_size / BLOCK_SIZE;

	float32_t freq[] 	= {9000.0f, 7000.0f, 1500.0f};
	float32_t amp[] 	= {0.2f, 0.0f, 1.0f};
	int freq_cnt = sizeof(freq) / sizeof(freq[0]);

	generate_sine_wave_f32(&src_arr[0], arr_size, fs, (float)(INT16_MAX * 0.5), freq, amp, freq_cnt);

	/* Call FIR init function to initialize the instance structure. */
	arm_fir_init_f32(&fir_instance, FIR_COEFF_COUNT, (float32_t *)&fir_filter_coeff[0], &fir_state[0], block_size);

//	print_buffer_data_f32(src_arr, arr_size);

    /* Send flag to CM33 core to indicate DSP Core has startup */
    MU_SetFlags(APP_MU, BOOT_FLAG);

    /* Enable transmit and receive interrupt */
    MU_EnableInterrupts(APP_MU, (kMU_Tx0EmptyInterruptEnable | kMU_Rx0FullInterruptEnable));

    while (1 != start_processing)
    {

    }

	for (uint32_t i = 0; i < iterations; i++)
	{
		for (uint32_t j = 0; j < num_blocks; j++)
		{
			arm_fir_f32(&fir_instance, src_arr + (j * block_size), dst_arr + (j * block_size), block_size);
		}
	}

	MU_SendMsgNonBlocking(APP_MU, CHN_MU_REG_NUM, msg_recv[cur_send++]);
	start_processing = 0;
	MU_EnableInterrupts(APP_MU, (kMU_Tx0EmptyInterruptEnable));

//	print_buffer_data_f32(dst_arr, arr_size);
}

static void test_drc_algorithm(int16_t * src_arr, int16_t * dst_arr, uint32_t arr_size, uint32_t fs, uint32_t iterations)
{
	float32_t freq[] 	= {9000.0f, 7000.0f, 1500.0f};
	float32_t amp[] 	= {0.2f, 0.0f, 1.0f};
	int freq_cnt = sizeof(freq) / sizeof(freq[0]);

	generate_sine_wave_16(&src_arr[0], arr_size, fs, (float)(INT16_MAX * 0.5), freq, amp, freq_cnt);

//	print_buffer_data_f32(src_arr, arr_size);

    /* Send flag to CM33 core to indicate DSP Core has startup */
    MU_SetFlags(APP_MU, BOOT_FLAG);

    /* Enable transmit and receive interrupt */
    MU_EnableInterrupts(APP_MU, (kMU_Tx0EmptyInterruptEnable | kMU_Rx0FullInterruptEnable));

    while (1 != start_processing)
    {

    }

	for (uint32_t i = 0; i < iterations; i++)
	{
		limiter_16(src_arr, dst_arr, arr_size);
	}

	MU_SendMsgNonBlocking(APP_MU, CHN_MU_REG_NUM, msg_recv[cur_send++]);
	start_processing = 0;
	MU_EnableInterrupts(APP_MU, (kMU_Tx0EmptyInterruptEnable));

//	print_buffer_data_f32(dst_arr, arr_size);
}
