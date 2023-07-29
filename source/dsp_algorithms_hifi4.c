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
 * @file    dsp_algorithms_hifi4.c
 * @brief   DSP algorithms implementations.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fsl_debug_console.h"
#include <math.h>

#include "dsp_algorithms_hifi4.h"
#include "dsp_algorithms_hifi4_conf.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define LOG2(x) (logf(x) / LN_OF_2)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
enum {
	CHANNEL_LEFT,
	CHANNEL_RIGHT,
	CHANNEL_CNT,
};

/*******************************************************************************
 * Variables
 ******************************************************************************/
const static float LN_OF_2 = 0.693147f;
static float AT;
static float RT;
static float RT_ctrl_fctr;
static float AT_ctrl_fctr;
static float TAV;
static float log2_LT;
static float log2_CT;
static float log2_ET;
static float log2_NT;
static float diff_ET_NT;
static float ES_times_diff_ET_NT;
static float one_minus_AT;
static float one_minus_RT;
static float one_minus_TAV;

/*******************************************************************************
 * Code
 ******************************************************************************/
void check_coefficients(void)
{
	if (NT >= ET || ET >= CT || ES >= 0.0 || NS >= 0.0)
	{
		PRINTF("Wrong coefficient value defined\r\n");
		exit(1);
	}
}

void calculate_coefficients(void)
{
	AT = (float)(1.0 - exp(-2.2 * 1000.0 / (t_at_ms * fs_hz)));
	RT = (float)(1.0 - exp(-2.2 * 1000.0 / (t_re_ms * fs_hz)));
	TAV = (float)(1.0 - exp((-2.2 * 1000.0) / (t_tav_ms * fs_hz)));
	AT_ctrl_fctr = (float)(1.0 - exp(-2.2 * 1000.0 / ((t_at_ms) * fs_hz)));
	RT_ctrl_fctr = (float)(1.0 - exp(-2.2 * 1000.0 / ((t_re_ms) * fs_hz)));
	log2_LT = LOG2(LT);
	log2_CT = LOG2(CT);
	log2_ET = LOG2(ET);
	log2_NT = LOG2(NT);
	one_minus_AT = (1.0f - AT);
	one_minus_RT = (1.0f - RT);
	one_minus_TAV = (1.0f - TAV);
	diff_ET_NT = (log2_ET - log2_NT);
	ES_times_diff_ET_NT = (ES * diff_ET_NT);
}

void limiter_16(int16_t * src_signal_arr, int16_t * dst_signal_arr, size_t signal_arr_count)
{
	static float x_peak[CHANNEL_CNT] = {[0 ... (CHANNEL_CNT - 1)] = 0.0f};
	static float ctrl_factor_old[CHANNEL_CNT] = {[0 ... (CHANNEL_CNT - 1)] = 1.0f};
	static float ctrl_factor_smooth[CHANNEL_CNT] = {[0 ... (CHANNEL_CNT - 1)] = 1.0f};
	float x_peak_abs = 0.0f;
	float x_peak_log = 0.0f;
	float ctrl_factor = 0.0f;
	float ctrl_factor_exp = 0.0f;
	float k = 0.0f;

#ifdef _DEBUG
	PRINTF("$");
#endif
	for (uint32_t channel = 0; channel < CHANNEL_CNT; ++channel)
	{
		for (uint32_t i = channel; i < signal_arr_count; i += 2)
		{
			float abs_sample = fabsf((float)src_signal_arr[i]);
			if (abs_sample > x_peak[channel])
			{
				x_peak[channel] = one_minus_AT * x_peak[channel] + AT * abs_sample;
			}
			else
			{
				x_peak[channel] = one_minus_RT * x_peak[channel];
			}

			x_peak_abs = fabsf(x_peak[channel]);
			x_peak_log = LOG2(x_peak_abs);

			if (x_peak_log > log2_LT)
			{
				ctrl_factor_exp = -LS * (x_peak_log - log2_LT);
				ctrl_factor = powf(2.0f, ctrl_factor_exp);
			}
			else
			{
				ctrl_factor = 1.0f;
			}

			if (ctrl_factor < ctrl_factor_old[channel])
			{
				k = RT_ctrl_fctr;
			}
			else
			{
				k = AT_ctrl_fctr;
			}

			ctrl_factor_smooth[channel] = (1.0f - k) * ctrl_factor_smooth[channel] + k * ctrl_factor;
			ctrl_factor_old[channel] = ctrl_factor;
#ifdef _DEBUG
			if (channel == CHANNEL_LEFT)
			{
				PRINTF("%.3f, ", x_peak_log);
				if (i%20 == 18)
				{
					PRINTF("\r\n");
				}
			}
#endif
			dst_signal_arr[i] = (int16_t)(src_signal_arr[i] * ctrl_factor_smooth[channel]);
		}
#ifdef _DEBUG
		PRINTF("\r\n");
#endif
	}
}

void compressor_expander_ngate_16(int16_t * src_signal_arr, int16_t * dst_signal_arr, size_t signal_arr_count)
{
	static float x_rms_pow2[CHANNEL_CNT] = {[0 ... (CHANNEL_CNT - 1)] = 0.0f};
	static float ctrl_factor_old[CHANNEL_CNT] = {[0 ... (CHANNEL_CNT - 1)] = 1.0f};
	static float ctrl_factor_smooth[CHANNEL_CNT] = {[0 ... (CHANNEL_CNT - 1)] = 1.0f};
	float x_rms_log;
	float ctrl_factor;
	float ctrl_factor_exp;
	float k;

#ifdef _DEBUG
	PRINTF("$");
#endif
	for (uint32_t channel = 0; channel < CHANNEL_CNT; ++channel)
	{
		for (uint32_t i = channel; i < signal_arr_count; i += 2U)
		{
			x_rms_pow2[channel] = (one_minus_TAV * x_rms_pow2[channel]) + (TAV * src_signal_arr[i] * src_signal_arr[i]);
			x_rms_log = 0.5f * LOG2(x_rms_pow2[channel]);

			if (x_rms_log > log2_CT)
			{
				ctrl_factor_exp = -CS * (x_rms_log - log2_CT);
				ctrl_factor = powf(2.0f, ctrl_factor_exp);
			}
			else if (x_rms_log >= log2_ET)
			{
				ctrl_factor = 1.0f;
			}
			else if (x_rms_log >= log2_NT)
			{
				ctrl_factor_exp = -ES * (x_rms_log - log2_ET);
				ctrl_factor = powf(2.0f, ctrl_factor_exp);
			}
			else
			{
				ctrl_factor_exp = -NS * (x_rms_log - log2_NT) + ES_times_diff_ET_NT;
				ctrl_factor = powf(2.0f, ctrl_factor_exp);
			}

			if (ctrl_factor < ctrl_factor_old[channel])
			{
				k = RT_ctrl_fctr;
			}
			else
			{
				k = AT_ctrl_fctr;
			}

			ctrl_factor_smooth[channel] = (1.0f - k) * ctrl_factor_smooth[channel] + k * ctrl_factor;
			ctrl_factor_old[channel] = ctrl_factor;
#ifdef _DEBUG
			if (channel == CHANNEL_LEFT)
			{
				PRINTF("%.3f, ", x_rms_log);
				if (i%20 == 18)
				{
					PRINTF("\r\n");
				}
			}
#endif
			dst_signal_arr[i] = (int16_t)(src_signal_arr[i] * ctrl_factor_smooth[channel]);
		}
	}
}
