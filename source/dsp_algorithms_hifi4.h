/*
 * dsp_algorithms_hifi4.h
 *
 *  Created on: 22 may 2023
 *      Author: Lukasz
 */

#ifndef dsp_algorithms_hifi4_H_
#define dsp_algorithms_hifi4_H_

void limiter_16(int16_t * src_signal_arr, int16_t * dst_signal_arr, size_t signal_arr_count);
void compressor_expander_ngate_16(int16_t * src_signal_arr, int16_t * dst_signal_arr, size_t signal_arr_count);
void calculate_coefficients(void);
void check_coefficients(void);

#endif /* dsp_algorithms_hifi4_H_ */
