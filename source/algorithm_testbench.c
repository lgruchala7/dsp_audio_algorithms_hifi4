/*
 * algorithm_testbench.c
 *
 *  Created on: 26 cze 2023
 *      Author: Lukasz
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fsl_debug_console.h"
#include "fsl_device_registers.h"

#include "drc_algorithms.h"
#include "algorithm_testbench.h"
#include "filters_cfg.h"

#include <time.h>
#include <math.h>

/*******************************************************************************
 * Code
 ******************************************************************************/
void generate_sine_wave_16(int16_t * input_vec, uint32_t vec_len, uint32_t fs, float max_amplitude, float * freq, float * amp, int freq_cnt)
{
	float * rad 	= (float *)malloc(sizeof(float) * freq_cnt);
	float * comp = (float *)malloc(sizeof(float) * freq_cnt);
	float temp_comp_sum 	= 0.0f;
	float amp_sum 			= 0.0f;

	if (max_amplitude > (float)INT16_MAX)
	{
		PRINTF("\r\nChange amplitude!!!\r\n");
	}

	for (uint32_t i = 0U; i < freq_cnt; i++)
	{
		amp_sum += amp[i];
	}

	for (uint32_t i = 0U, sample_num = 0U; i < vec_len; i += 2, sample_num++)
	{
		/* convert to radians */
		for (uint32_t j = 0U; j < freq_cnt; j++)
		{
			rad[j] = (2 * PI * freq[j] * sample_num / fs);
		}

		/* calculate component signals values */
		for (uint32_t j = 0U; j < freq_cnt; j++)
		{
			comp[j] = (max_amplitude * amp[j] * arm_sin_f32(rad[j]));
		}

		temp_comp_sum = 0.0f;
		/* sum component signals */
		for (uint32_t j = 0U; j < freq_cnt; j++)
		{
			temp_comp_sum += comp[j];
		}
		input_vec[i] = (int16_t)(temp_comp_sum / amp_sum);
	}

	free(rad);
	free(comp);
}

void generate_sine_wave_f32(float32_t * input_vec, uint32_t vec_len, uint32_t fs, float max_amplitude, float * freq, float * amp, int freq_cnt)
{
	float * rad 	= (float *)malloc(sizeof(float) * freq_cnt);
	float * comp = (float *)malloc(sizeof(float) * freq_cnt);
	float temp_comp_sum 	= 0.0f;
	float amp_sum 			= 0.0f;

	if (max_amplitude > (float)INT16_MAX)
	{
		PRINTF("\r\nChange amplitude!!!\r\n");
	}

	for (uint32_t i = 0U; i < freq_cnt; i++)
	{
		amp_sum += amp[i];
	}

	for (uint32_t i = 0U; i < vec_len; i++)
	{
		/* convert to radians */
		for (uint32_t j = 0U; j < freq_cnt; j++)
		{
			rad[j] = (2 * PI * freq[j] * i / fs);
		}

		/* calculate component signals values */
		for (uint32_t j = 0U; j < freq_cnt; j++)
		{
			comp[j] = (max_amplitude * amp[j] * arm_sin_f32(rad[j]));
		}

		temp_comp_sum = 0.0f;
		/* sum component signals */
		for (uint32_t j = 0U; j < freq_cnt; j++)
		{
			temp_comp_sum += comp[j];
		}
		input_vec[i] = (float32_t)(temp_comp_sum / amp_sum);
	}

	free(rad);
	free(comp);
}


void print_buffer_data_16(int16_t * data, size_t data_size)
{
	for (size_t i = 0; i < data_size; i++)
	{
//		PRINTF("0x%0X, ", data[i]);
		PRINTF("%d, ", data[i]);
		if (i%20 == 19)
		{
			PRINTF("\r\n");
		}
	}
	PRINTF("\r\n\n");
}

void print_buffer_data_f32(float32_t * buffer, size_t buffer_size)
{
	for (size_t i = 0; i < buffer_size; i++)
	{
		PRINTF("%.3f, ", buffer[i]);
		if (i%20 == 19)
		{
			PRINTF("\r\n");
		}
	}
	PRINTF("\r\n\n");
}

void write_buffer_data_to_file_16(int16_t * data, size_t data_size)
{
	PRINTF("$");
	for (size_t i = 0; i < data_size; i++)
	{
		PRINTF("%d, ", data[i]);
		if (i%20 == 19)
		{
			PRINTF("\r\n");
		}
	}
}

//void test_drc_algorithm(void (*algorithm_func)(int16_t *, int16_t *, size_t), int16_t * src_buffer, int16_t * dst_buffer,
//		size_t buffer_size, uint32_t fs)
//{
//	float freq[] 	= {9000.0f, 7000.0f, 1500.0f};
//	float amp[] 	= {0.1f, 0.2f, 2.0f};
//	int freq_cnt = sizeof(freq) / sizeof(freq[0]);
//
//	generate_sine_wave_16(&src_buffer[0], buffer_size/5, fs, (float)INT16_MAX * 0.15, freq, amp, freq_cnt);
//	generate_sine_wave_16(&src_buffer[1000],buffer_size/5, fs, (float)INT16_MAX * 0.40, freq, amp, freq_cnt);
//	generate_sine_wave_16(&src_buffer[2000], buffer_size/5, fs, (float)INT16_MAX * 0.65, freq, amp, freq_cnt);
//	generate_sine_wave_16(&src_buffer[3000], buffer_size/5, fs, (float)INT16_MAX * 0.8, freq, amp, freq_cnt);
//	generate_sine_wave_16(&src_buffer[4000], buffer_size/5, fs, (float)INT16_MAX * 0.95, freq, amp, freq_cnt);
//
//	write_buffer_data_to_file_16(src_buffer, buffer_size);
//	algorithm_func(src_buffer, dst_buffer, buffer_size);
//	write_buffer_data_to_file_16(dst_buffer, buffer_size);
//}
