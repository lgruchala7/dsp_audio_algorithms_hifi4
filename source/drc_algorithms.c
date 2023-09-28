/*
 * Copyright 2016-2023 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    drc_algorithms_hifi4.c
 * @brief   DSP algorithms implementations.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fsl_debug_console.h"
#include <math.h>

#include "main_hifi4.h"
#include "drc_algorithms.h"
#include "drc_algorithms_cfg.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#if Q31_USED
/* LUT constants */
#define TABLE_SIZE      	4096
#define SCALE_FACTOR	   	8
#define SCALE_FACTOR_SHIFT  3 /* log2(SCALE_FACTOR) */
#define LUT_MIN_LN         	0.000335461f
#define LUT_MIN_LN_Q31     	0x00dcca71
#define LUT_MAX_LN         	1.0f
#define LUT_RANGE_LN       	0x7f23358f
#define LUT_RANGE_LN_F32   	(LUT_MAX_LN - LUT_MIN_LN)
#define LUT_MIN_EXP         -1.0f
#define LUT_MIN_EXP_Q31     0x80000000
#define LUT_MAX_EXP        	0.0f
#define LUT_RANGE_EXP      	0x80000000
#define LUT_RANGE_EXP_F32 	(LUT_MAX_EXP - LUT_MIN_EXP)
/* Q31 operations */
#define Q31_MUL(x, y)							((__SSAT((((q63_t) (x) * (y)) >> 32), 31)) << 1U)
#define Q31_ADD(x, y)							(__QADD((x), (y)))
#define Q31_SUB(x, y)							(__QSUB((x), (y)))
#define Q31_DIV(num, den, q_ptr, sh_ptr)		(arm_divide_q31((num), (den), (q_ptr), (sh_ptr)))
#define Q31_LOG2(src, result_ptr, shift_ptr) 	Q31_DIV(q31_ln(src), LN_OF_2_Q31, (result_ptr), (shift_ptr))
#define Q31_POW2(src, dst_ptr)					(*dst_ptr = q31_exp((src)))
#endif
#if PQ_USED
#define LOG2(src_ptr, dst_ptr) 	do { \
									PQ_LnF32((src_ptr), (dst_ptr)); \
									PQ_DivF32((dst_ptr), (float32_t *)&LN_OF_2, (dst_ptr)); \
								} while (0)
#else
#define LOG2(src_ptr, dst_ptr)	(*(dst_ptr) = (logf(*(src_ptr)) / LN_OF_2))
#endif

/*******************************************************************************
 * Type definitions
 ******************************************************************************/
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void check_coefficients(float32_t log2_LT, float32_t  log2_CT, float32_t  log2_ET, float32_t  log2_NT, float32_t  log2_NT_MUTE);
#if Q31_USED
static inline void peak_log2_calculate(q31_t src, q31_t * dst_ptr, uint32_t channel);
static inline void rms_log2_calculate(q31_t src, q31_t * dst_ptr, uint32_t channel);
static inline q31_t q31_ln(q31_t x_q31);
static inline q31_t q31_exp(q31_t x_q31);
static void initialize_exp_lut(void);
static void initialize_ln_lut(void);
#else
static inline void peak_log2_calculate(float32_t src, float32_t * dst_ptr, uint32_t channel);
static inline void rms_log2_calculate(float32_t src, float32_t * dst_ptr, uint32_t channel);
#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/
const static float32_t LN_OF_2 = 0.693147f;
#if !(Q31_USED)
static float32_t AT;
static float32_t RT;
static float32_t RT_ctrl_factor_1;
static float32_t AT_ctrl_factor_1;
static float32_t RT_ctrl_factor_2;
static float32_t AT_ctrl_factor_2;
static float32_t TAV;
static float32_t log2_LT;
static float32_t log2_CT;
static float32_t log2_ET;
static float32_t log2_NT;
static float32_t log2_NT_MUTE;
static float32_t CS_times_diff_CT_LT;
static float32_t ES_times_diff_ET_NT;
static float32_t one_minus_AT;
static float32_t one_minus_RT;
static float32_t one_minus_TAV;
#else
const static q31_t LN_OF_2_Q31 = (q31_t)0x58b90bfc;
static q31_t AT;
static q31_t RT;
static q31_t RT_ctrl_factor_1;
static q31_t AT_ctrl_factor_1;
static q31_t RT_ctrl_factor_2;
static q31_t AT_ctrl_factor_2;
static q31_t TAV;
static float32_t log2_LT_scaled_f32;
static float32_t log2_CT_scaled_f32;
static float32_t log2_ET_scaled_f32;
static float32_t log2_NT_scaled_f32;
static float32_t log2_NT_MUTE_scaled_f32;
static q31_t log2_LT_scaled;
static q31_t log2_CT_scaled;
static q31_t log2_ET_scaled;
static q31_t log2_NT_scaled;
static q31_t log2_NT_MUTE_scaled;
static q31_t pow2_ES_times_diff_ET_NT;
static q31_t pow2_CS_times_diff_CT_LT;
static q31_t one_minus_AT;
static q31_t one_minus_RT;
static q31_t one_minus_TAV;
#endif

#if Q31_USED
// Pre-calculated Q1.31 fixed-point natural logarithm values
static q31_t ln_lookup_table[TABLE_SIZE];
static q31_t exp_lookup_table[TABLE_SIZE];
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/
static void check_coefficients(float32_t log2_LT, float32_t  log2_CT, float32_t  log2_ET, float32_t  log2_NT, float32_t  log2_NT_MUTE)
{
	if (NT >= ET || ET >= CT || CT >= LT)
	{
		PRINTF("Wrong threshold value(s) defined\r\n");
		exit(1);
	}
	#if !(Q31_USED)
	if ((LS != 1.0f) || (CS >= 1.0f) || (CS <= 0.0f) || (ES <= -1.0f) || (ES >= 0.0f) || (NS >= -1.0f) ||
		(ES_times_diff_ET_NT >= 0.0f))
	{
		PRINTF("Wrong slope value(s) defined\r\n");
		exit(1);
	}
	#else
	if (((uint32_t)LS_INV != (uint32_t)Q31_ONE) ||
		IS_IN_RANGE_INCL((uint32_t)CS, (uint32_t)Q31_ONE, (uint32_t)Q31_ZERO) ||
		IS_IN_RANGE_INCL((uint32_t)ES, (uint32_t)Q31_ZERO, (uint32_t)Q31_ONE) ||
		IS_IN_RANGE_INCL((uint32_t)NS_INV, (uint32_t)Q31_ZERO, (uint32_t)Q31_ONE) ||
		IS_IN_RANGE_INCL((uint32_t)pow2_ES_times_diff_ET_NT, (uint32_t)Q31_ONE, (uint32_t)Q31_ZERO) ||
		IS_IN_RANGE_INCL((uint32_t)pow2_CS_times_diff_CT_LT, (uint32_t)Q31_ONE, (uint32_t)Q31_ZERO))
	{
		PRINTF("Wrong slope value(s) defined\r\n");
		exit(1);
	}
	if ((((log2_LT - 1.0f) / (float32_t)LS_INV) >= SCALE_FACTOR) ||
		((CS * (log2_CT - log2_LT)) >= SCALE_FACTOR) ||
		((ES * (log2_ET - log2_NT)) >= SCALE_FACTOR) ||
		(((log2_NT - log2_NT_MUTE) / NS_INV) >= SCALE_FACTOR))
	{
		PRINTF("Wrong ranges value(s) defined\r\n");
	}
	#endif
}

void calculate_coefficients(void)
{
	#if !(Q31_USED)
	float32_t diff_ET_NT, diff_CT_LT;

	AT = (float32_t)(1.0 - exp(-2.2 * 1000.0 / (t_at_ms * fs_hz)));
	RT = (float32_t)(1.0 - exp(-2.2 * 1000.0 / (t_re_ms * fs_hz)));
	TAV = (float32_t)(1.0 - exp((-2.2 * 1000.0) / (t_tav_ms * fs_hz)));
	AT_ctrl_factor_1 = (float32_t)(1.0 - exp(-2.2 * 1000.0 / ((t_at_cf_1_ms) * fs_hz)));
	RT_ctrl_factor_1 = (float32_t)(1.0 - exp(-2.2 * 1000.0 / ((t_re_cf_1_ms) * fs_hz)));
	AT_ctrl_factor_2 = (float32_t)(1.0 - exp(-2.2 * 1000.0 / ((t_at_cf_2_ms) * fs_hz)));
	RT_ctrl_factor_2 = (float32_t)(1.0 - exp(-2.2 * 1000.0 / ((t_re_cf_2_ms) * fs_hz)));

	float32_t lt = LT, ct = CT, et = ET, nt = NT, nt_mute = NT_MUTE;

	LOG2(&lt, &log2_LT);
	LOG2(&ct, &log2_CT);
	LOG2(&et, &log2_ET);
	LOG2(&nt, &log2_NT);
	LOG2(&nt_mute, &log2_NT_MUTE);

	one_minus_AT = (1.0f - AT);
	one_minus_RT = (1.0f - RT);
	one_minus_TAV = (1.0f - TAV);
	diff_ET_NT = (log2_ET - log2_NT);
	diff_CT_LT = (log2_CT - log2_LT);
	ES_times_diff_ET_NT = (ES * diff_ET_NT);
	CS_times_diff_CT_LT = (CS * diff_CT_LT);
	#else
	float32_t AT_f32, RT_f32, TAV_f32, AT_ctrl_factor_1_f32, RT_ctrl_factor_1_f32, AT_ctrl_factor_2_f32, RT_ctrl_factor_2_f32;

	AT_f32 = (float32_t)(1.0 - expf(-2.2f * 1000.0f / (t_at_ms * fs_hz)));
	RT_f32 = (float32_t)(1.0 - expf(-2.2f * 1000.0f / (t_re_ms * fs_hz)));
	TAV_f32 = (float32_t)(1.0 - expf((-2.2f * 1000.0f) / (t_tav_ms * fs_hz)));
	AT_ctrl_factor_1_f32 = (float32_t)(1.0f - expf(-2.2f * 1000.0f / ((t_at_cf_1_ms) * fs_hz)));
	RT_ctrl_factor_1_f32 = (float32_t)(1.0f - expf(-2.2f * 1000.0f / ((t_re_cf_1_ms) * fs_hz)));
	AT_ctrl_factor_2_f32 = (float32_t)(1.0f - expf(-2.2f * 1000.0f / ((t_at_cf_2_ms) * fs_hz)));
	RT_ctrl_factor_2_f32 = (float32_t)(1.0f - expf(-2.2f * 1000.0f / ((t_re_cf_2_ms) * fs_hz)));

	arm_float_to_q31(&AT_f32, &AT, 1);
	arm_float_to_q31(&RT_f32, &RT, 1);
	arm_float_to_q31(&TAV_f32, &TAV, 1);
	arm_float_to_q31(&AT_ctrl_factor_1_f32, &AT_ctrl_factor_1, 1);
	arm_float_to_q31(&RT_ctrl_factor_1_f32, &RT_ctrl_factor_1, 1);
	arm_float_to_q31(&AT_ctrl_factor_2_f32, &AT_ctrl_factor_2, 1);
	arm_float_to_q31(&RT_ctrl_factor_2_f32, &RT_ctrl_factor_2, 1);

	q31_t lt = LT, ct = CT, et = ET, nt = NT, nt_mute = NT_MUTE;
	float32_t lt_f32, ct_f32, et_f32, nt_f32, nt_mute_f32;
	float32_t log2_LT_f32, log2_CT_f32, log2_ET_f32, log2_NT_f32, log2_NT_MUTE_f32;

	arm_q31_to_float(&lt, &lt_f32, 1);
	arm_q31_to_float(&ct, &ct_f32, 1);
	arm_q31_to_float(&et, &et_f32, 1);
	arm_q31_to_float(&nt, &nt_f32, 1);
	arm_q31_to_float(&nt_mute, &nt_mute_f32, 1);

	LOG2(&lt_f32, &log2_LT_f32);
	LOG2(&ct_f32, &log2_CT_f32);
	LOG2(&et_f32, &log2_ET_f32);
	LOG2(&nt_f32, &log2_NT_f32);
	LOG2(&nt_mute_f32, &log2_NT_MUTE_f32);

	arm_scale_f32(&log2_LT_f32, (1.0f / (float32_t)SCALE_FACTOR), &log2_LT_scaled_f32, 1);
	arm_float_to_q31(&log2_LT_scaled_f32, &log2_LT_scaled, 1);

	arm_scale_f32(&log2_CT_f32, (1.0f / (float32_t)SCALE_FACTOR), &log2_CT_scaled_f32, 1);
	arm_float_to_q31(&log2_CT_scaled_f32, &log2_CT_scaled, 1);

	arm_scale_f32(&log2_ET_f32, (1.0f / (float32_t)SCALE_FACTOR), &log2_ET_scaled_f32, 1);
	arm_float_to_q31(&log2_ET_scaled_f32, &log2_ET_scaled, 1);

	arm_scale_f32(&log2_NT_f32, (1.0f / (float32_t)SCALE_FACTOR), &log2_NT_scaled_f32, 1);
	arm_float_to_q31(&log2_NT_scaled_f32, &log2_NT_scaled, 1);

	arm_scale_f32(&log2_NT_MUTE_f32, (1.0f / (float32_t)SCALE_FACTOR), &log2_NT_MUTE_scaled_f32, 1);
	arm_float_to_q31(&log2_NT_MUTE_scaled_f32, &log2_NT_MUTE_scaled, 1);

	one_minus_AT 	= Q31_SUB(Q31_ONE, AT);
	one_minus_RT	= Q31_SUB(Q31_ONE, RT);
	one_minus_TAV	= Q31_SUB(Q31_ONE, TAV);

	float32_t diff_ET_NT_f32 = log2_ET_f32 - log2_NT_f32;
	float32_t ES_times_diff_ET_NT_f32 = ES_F32 * diff_ET_NT_f32;
	float32_t pow2_ES_times_diff_ET_NT_f32 = powf(2.0f, ES_times_diff_ET_NT_f32);
	arm_float_to_q31(&pow2_ES_times_diff_ET_NT_f32, &pow2_ES_times_diff_ET_NT, 1);

	float32_t diff_CT_LT_f32 = log2_CT_f32 - log2_LT_f32;
	float32_t CS_times_diff_CT_LT_f32 = CS_F32 * diff_CT_LT_f32;
	float32_t pow2_CS_times_diff_CT_LT_f32 = powf(2.0f, CS_times_diff_CT_LT_f32);
	arm_float_to_q31(&pow2_CS_times_diff_CT_LT_f32, &pow2_CS_times_diff_CT_LT, 1);

	check_coefficients(log2_LT_f32, log2_CT_f32, log2_ET_f32, log2_NT_f32, log2_NT_MUTE_f32);

	initialize_ln_lut();
	initialize_exp_lut();
	#endif /* Q31_USED */
}

#if Q31_USED
static void initialize_ln_lut(void)
{
	/* fill LUT of ln(x) for range (LUT_MIN_LN, 1) / SCALE_FACTOR */
    for (int i = 0; i < TABLE_SIZE; i++)
    {
    	float32_t x = LUT_MIN_LN + ((float32_t)i / (float32_t)TABLE_SIZE) * LUT_RANGE_LN_F32;
    	ln_lookup_table[i] = (q31_t)((logf(x) / (float32_t)SCALE_FACTOR) * (float32_t)Q31_ONE_F32);
    }
}
#endif

#if Q31_USED
static void initialize_exp_lut(void)
{
	/* fill LUT of ln(x) for range (-1, 0) */
    for (int i = 0; i < TABLE_SIZE; i++)
    {
    	float32_t x = LUT_MIN_EXP + ((float32_t)i / (float32_t)TABLE_SIZE) * LUT_RANGE_EXP_F32;
    	exp_lookup_table[i] = (q31_t)(expf(x) * (float32_t)Q31_ONE_F32);
    }
}
#endif

#if Q31_USED
// Calculate natural logarithm using lookup table and interpolation
static inline q31_t q31_ln(q31_t x_q31)
{
    if (x_q31 <= 0)
    {
        // Handle invalid input (<= 0)
        return 0;
    }

    // Calculate index into the lookup table
    int index = (int)((q63_t)(x_q31 - LUT_MIN_LN_Q31) * TABLE_SIZE / LUT_RANGE_LN);

    if (x_q31 < LUT_MIN_LN_Q31)
    {
        return ln_lookup_table[0];
    }
    else if (index >= (TABLE_SIZE - 1))
    {
        return ln_lookup_table[TABLE_SIZE - 1];
    }

    // Linear interpolation
    q31_t y0 = ln_lookup_table[index];
    q31_t y1 = ln_lookup_table[index + 1];
    q31_t diff = x_q31 - (q31_t)((q63_t)index * LUT_RANGE_LN / TABLE_SIZE + LUT_MIN_LN_Q31);
    q31_t result = y0 + (q31_t)((y1 - y0) * (q63_t)diff * TABLE_SIZE / LUT_RANGE_LN);

    return result;
}
#endif

#if Q31_USED
// Calculate exp() using lookup table and interpolation
static inline q31_t q31_exp(q31_t x_q31)
{

    // Calculate index into the lookup table
    int index = (int)((q63_t)(x_q31 - LUT_MIN_EXP_Q31) * TABLE_SIZE / LUT_RANGE_EXP);

    if (x_q31 < LUT_MIN_EXP_Q31)
    {
        return exp_lookup_table[0];
    }
    else if (index >= (TABLE_SIZE - 1))
    {
        return exp_lookup_table[TABLE_SIZE - 1];
    }

    // Linear interpolation
    q31_t y0 = exp_lookup_table[index];
    q31_t y1 = exp_lookup_table[index + 1];
    q31_t diff = x_q31 - (q31_t)((q63_t)index * LUT_RANGE_EXP / TABLE_SIZE + LUT_MIN_EXP_Q31);
    q31_t result = y0 + (q31_t)((y1 - y0) * (q63_t)diff * TABLE_SIZE / LUT_RANGE_EXP);

    return result;
}
#endif /* Q31_USED */

#if Q31_USED
static inline void peak_log2_calculate(q31_t src, q31_t * dst_ptr, uint32_t channel)
{
	static q31_t peak[CHANNEL_CNT] = {Q31_ZERO};
	q31_t abs_src;
	q31_t peak_log2_temp;
	int16_t shift;

	arm_abs_q31(&src, &abs_src, 1);

	if (abs_src > peak[channel])
	{
		peak[channel] = Q31_ADD(Q31_MUL(one_minus_AT, peak[channel]), Q31_MUL(AT, abs_src));
	}
	else
	{
		peak[channel] = Q31_MUL(one_minus_RT, peak[channel]);
	}

	Q31_LOG2(peak[channel], &peak_log2_temp, &shift);

	if (shift != 0)
	{
		arm_shift_q31(&peak_log2_temp, (int8_t)(shift), dst_ptr, 1);
	}
	else
	{
		*dst_ptr = peak_log2_temp;
	}
}

#else

static inline void peak_log2_calculate(float32_t src, float32_t * dst_ptr, uint32_t channel)
{
	static float32_t peak[CHANNEL_CNT] = {0.0f};
	float32_t peak_abs;

	float32_t abs_src = fabsf(src);

	if (abs_src > peak[channel])
	{
		peak[channel] = (one_minus_AT * peak[channel]) +  (AT * abs_src);
	}
	else
	{
		peak[channel] = one_minus_RT * peak[channel];
	}

	if (peak[channel] < 0.0f)
	{
		peak_abs = fabsf(peak[channel]);
	}
	else
	{
		peak_abs = peak[channel];
	}

	LOG2(&peak_abs, dst_ptr);
}
#endif /* Q31_USED */

#if Q31_USED
static inline void rms_log2_calculate(q31_t src, q31_t * dst_ptr, uint32_t channel)
{
	static q31_t rms_pow2[CHANNEL_CNT] = {Q31_ZERO};
	q31_t rms_log2_temp;
	int16_t shift;

	rms_pow2[channel] = Q31_ADD(Q31_MUL(one_minus_TAV, rms_pow2[channel]), Q31_MUL(TAV, Q31_MUL(src, src)));
	Q31_LOG2(rms_pow2[channel], &rms_log2_temp, &shift);
	if (shift != 1)
	{
		arm_shift_q31(&rms_log2_temp, (int8_t)(shift - 1), dst_ptr, 1);
	}
	else
	{
		*dst_ptr = rms_log2_temp;
	}
}

#else

static inline void rms_log2_calculate(float32_t src, float32_t * dst_ptr, uint32_t channel)
{
	static float32_t rms_pow2[CHANNEL_CNT] = {0.0f};

	rms_pow2[channel] = (one_minus_TAV * rms_pow2[channel]) + (TAV * (src * src));
	LOG2(&rms_pow2[channel], dst_ptr);
	*dst_ptr = (*dst_ptr) * 0.5f;
}
#endif /* Q31_USED */

#if Q31_USED
void drc_full_stereo_balanced(q31_t * src_signal_arr, q31_t * dst_signal_arr)
{
	static q31_t ctrl_factor_old = 1.0f;
	static q31_t ctrl_factor_smooth = 1.0f;
	q31_t x_rms_log2;
	q31_t x_peak_log2;
	q31_t ctrl_factor;
	q31_t ctrl_factor_exp;
	q31_t k;
	int16_t shift;
	int channel_offset = BUFFER_SIZE / 2;
	bool is_limiter_active;

	for (int i = 0; i < channel_offset; ++i)
	{
		is_limiter_active = false;
		q31_t avg_sample = (q31_t)(((q63_t)src_signal_arr[i] + (q63_t)src_signal_arr[i + channel_offset]) >> 1); // >>1 ??
		peak_log2_calculate(avg_sample, &x_peak_log2, 0U);
		rms_log2_calculate(avg_sample, &x_rms_log2, 0U);

		if (x_peak_log2 > log2_LT_scaled)
		{
			Q31_DIV(Q31_SUB(log2_LT_scaled, x_peak_log2), LS_INV, &ctrl_factor_exp, &shift); //to delete?? division by 1
			assert(shift == 0U);
			Q31_POW2(ctrl_factor_exp, &ctrl_factor);

			q31_t temp = ctrl_factor;
			for (int i = 0; i < SCALE_FACTOR; i++)
			{
				ctrl_factor = Q31_MUL(ctrl_factor, temp);
			}
			ctrl_factor = Q31_MUL(ctrl_factor, pow2_CS_times_diff_CT_LT);
			is_limiter_active = true;
		}
		else if (x_rms_log2 > log2_CT_scaled)
		{
			ctrl_factor_exp = Q31_MUL(CS, Q31_SUB(log2_CT_scaled, x_rms_log2));
			Q31_POW2(ctrl_factor_exp, &ctrl_factor);

			q31_t temp = ctrl_factor;
			for (int i = 0; i < SCALE_FACTOR; i++)
			{
				ctrl_factor = Q31_MUL(ctrl_factor, temp);
			}
		}
		else if (x_rms_log2 < log2_NT_MUTE_scaled)
		{
			ctrl_factor = Q31_ZERO;
		}
		else if (x_rms_log2 < log2_NT_scaled)
		{
			Q31_DIV(Q31_SUB(log2_NT_scaled, x_rms_log2), NS_INV, &ctrl_factor_exp, &shift);
			assert(shift == 0U);
			Q31_POW2(ctrl_factor_exp, &ctrl_factor);

			q31_t temp = ctrl_factor;
			for (int i = 0; i < SCALE_FACTOR; i++)
			{
				ctrl_factor = Q31_MUL(ctrl_factor, temp);
			}
			ctrl_factor = Q31_MUL(ctrl_factor, pow2_ES_times_diff_ET_NT);
		}
		else if (x_rms_log2 < log2_ET_scaled)
		{
			ctrl_factor_exp = Q31_MUL(ES, Q31_SUB(log2_ET_scaled, x_rms_log2));
			Q31_POW2(ctrl_factor_exp, &ctrl_factor);

			q31_t temp = ctrl_factor;
			for (int i = 0; i < SCALE_FACTOR; i++)
			{
				ctrl_factor = Q31_MUL(ctrl_factor, temp);
			}
		}
		else
		{
			ctrl_factor = Q31_ONE;
		}

		if (ctrl_factor < ctrl_factor_old)
		{
			if (is_limiter_active)
			{
				k = RT_ctrl_factor_1;
			}
			else
			{
				k = RT_ctrl_factor_2;
			}
		}
		else
		{
			if (is_limiter_active)
			{
				k = AT_ctrl_factor_1;
			}
			else
			{
				k = AT_ctrl_factor_2;
			}
		}

		/* smooth control factor */
		ctrl_factor_smooth = Q31_ADD(Q31_MUL(Q31_SUB(Q31_ONE, k), ctrl_factor_smooth), Q31_MUL(k, ctrl_factor));
		ctrl_factor_old = ctrl_factor_smooth;
		/* calculate output sample */
		dst_signal_arr[i] = Q31_MUL(src_signal_arr[i], ctrl_factor_smooth);
		dst_signal_arr[i + channel_offset] = Q31_MUL(src_signal_arr[i + channel_offset], ctrl_factor_smooth);
	}
}

#else

void drc_full_stereo_balanced(float32_t * src_signal_arr, float32_t * dst_signal_arr)
{
	static float32_t ctrl_factor_old = 1.0f;
	static float32_t ctrl_factor_smooth = 1.0f;
	float32_t x_rms_log2;
	float32_t x_peak_log2;
	float32_t ctrl_factor;
	float32_t ctrl_factor_exp;
	float32_t k;
	static const int channel_offset = BUFFER_SIZE / CHANNEL_CNT;
	bool is_limiter_active;

	for (int i = 0; i < channel_offset; ++i)
	{
		is_limiter_active = false;

		float32_t avg_sample = ((src_signal_arr[i] + src_signal_arr[i + channel_offset]) / (float32_t)CHANNEL_CNT);
		peak_log2_calculate(avg_sample, &x_peak_log2, 0U);
		rms_log2_calculate(avg_sample, &x_rms_log2, 0U);

		if (x_peak_log2 > log2_LT)
		{
			ctrl_factor_exp = LS * (log2_LT - x_peak_log2) + CS_times_diff_CT_LT;
			ctrl_factor = powf(2.0f, ctrl_factor_exp);
			is_limiter_active = true;
		}
		else if (x_rms_log2 > log2_CT)
		{
			ctrl_factor_exp = CS * (log2_CT - x_rms_log2);
			ctrl_factor = powf(2.0f, ctrl_factor_exp);
		}
		else if (x_rms_log2 < log2_NT_MUTE)
		{
			ctrl_factor = 0.0f;
		}
		else if (x_rms_log2 < log2_NT)
		{
			ctrl_factor_exp = NS * (log2_NT - x_rms_log2) + ES_times_diff_ET_NT;
			ctrl_factor = powf(2.0f, ctrl_factor_exp);
		}
		else if (x_rms_log2 < log2_ET)
		{
			ctrl_factor_exp = ES * (log2_ET - x_rms_log2);
			ctrl_factor = powf(2.0f, ctrl_factor_exp);
		}
		else
		{
			ctrl_factor = 1.0f;
		}

		/* determine if control factor (in/de)creasing */
		if (ctrl_factor < ctrl_factor_old)
		{
			if (is_limiter_active)
			{
				k = RT_ctrl_factor_1;
			}
			else
			{
				k = RT_ctrl_factor_2;
			}
		}
		else
		{
			if (is_limiter_active)
			{
				k = AT_ctrl_factor_1;
			}
			else
			{
				k = AT_ctrl_factor_2;
			}
		}

		/* smooth control factor */
		ctrl_factor_smooth = (1.0f - k) * ctrl_factor_smooth + k * ctrl_factor;
		ctrl_factor_old = ctrl_factor_smooth;
		/* calculate output sample */
		dst_signal_arr[i] = src_signal_arr[i] * ctrl_factor_smooth;
		dst_signal_arr[i + channel_offset] = src_signal_arr[i + channel_offset] * ctrl_factor_smooth;
	}
}
#endif /* Q31_USED */
