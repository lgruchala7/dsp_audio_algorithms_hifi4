/*
 * drc_algorithms_hifi4.h
 *
 *  Created on: 22 may 2023
 *      Author: Lukasz
 */

#ifndef _DRC_ALGORITHMS_HIFI4_H_
#define _DRC_ALGORITHMS_HIFI4_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "main_hifi4.h"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void calculate_coefficients(void);
#if Q31_USED
void drc_full_stereo_balanced(q31_t * src_signal_arr, q31_t * dst_signal_arr);
arm_status arm_divide_q31(q31_t numerator, q31_t denominator, q31_t *quotient, int16_t *shift);
#else
void drc_full_stereo_balanced(float32_t * src_signal_arr, float32_t * dst_signal_arr);
#endif

#endif /* _DRC_ALGORITHMS_HIFI4_H_ */
