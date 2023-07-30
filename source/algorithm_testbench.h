/*
 * algorithm_testbench.h
 *
 *  Created on: 26 cze 2023
 *      Author: Lukasz
 */

#ifndef ALGORITHM_TESTBENCH_H_
#define ALGORITHM_TESTBENCH_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "arm_math.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define FIR_ORDER		256U
#define FIR_COEFF_COUNT	(FIR_ORDER + 1U)
#define BLOCK_SIZE		32U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void measure_algorithm_time_16(void (*algorithm_func)(int16_t *, int16_t *, size_t), int16_t * src_buffer, int16_t * dst_buffer, size_t buffer_size, uint32_t iterations);
void print_buffer_data_16(int16_t * data, size_t data_size);
//void test_drc_algorithm(void (*algorithm_func)(int16_t *, int16_t *, size_t), int16_t * src_buffer, int16_t * dst_buffer, size_t buffer_size, uint32_t fs);
void print_buffer_data_f32(float32_t * buffer, size_t buffer_size);
//void test_cmsis_dsp(float32_t * src_arr, float32_t * dst_arr, uint32_t arr_size, uint32_t fs, uint32_t iterations);
void init_arr_with_rand_16(int16_t * arr, size_t arr_size);
void generate_sine_wave_f32(float32_t * input_vec, uint32_t vec_len, uint32_t fs, float max_amplitude, float * freq, float * amp, int freq_cnt);
void generate_sine_wave_16(int16_t * input_vec, uint32_t vec_len, uint32_t fs, float max_amplitude, float * freq, float * amp, int freq_cnt);

/*******************************************************************************
 * Exported variables
 ******************************************************************************/
extern const float32_t fir_filter_coeff_f32 [FIR_COEFF_COUNT];
extern float32_t fir_state_f32[BLOCK_SIZE + FIR_COEFF_COUNT - 1];
extern q31_t fir_filter_coeff_q31 [FIR_COEFF_COUNT];
extern q31_t fir_state_q31[BLOCK_SIZE + FIR_COEFF_COUNT - 1];

#endif /* ALGORITHM_TESTBENCH_H_ */
