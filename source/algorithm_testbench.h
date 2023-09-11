/*
 * algorithm_testbench.h
 *
 *  Created on: 26 cze 2023
 *      Author: Lukasz
 */

#ifndef _ALGORITHM_TESTBENCH_H_
#define _ALGORITHM_TESTBENCH_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "arm_math.h"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void measure_algorithm_time_16(void (*algorithm_func)(int16_t *, int16_t *, size_t), int16_t * src_buffer, int16_t * dst_buffer, size_t buffer_size, uint32_t iterations);
void print_buffer_data_16(int16_t * data, size_t data_size);
void print_buffer_data_f32(float32_t * buffer, size_t buffer_size);
void init_arr_with_rand_16(int16_t * arr, size_t arr_size);
void generate_sine_wave_f32(float32_t * input_vec, uint32_t vec_len, uint32_t fs, float max_amplitude, float * freq, float * amp, int freq_cnt);
void generate_sine_wave_16(int16_t * input_vec, uint32_t vec_len, uint32_t fs, float max_amplitude, float * freq, float * amp, int freq_cnt);

#endif /* _ALGORITHM_TESTBENCH_H_ */
