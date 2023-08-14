/*
 * filters_cfg.h
 *
 *  Created on: 14 sie 2023
 *      Author: £ukasz
 */

#ifndef FILTERS_CFG_H_
#define FILTERS_CFG_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "dsp_config.h"
#include "arm_math.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define FIR_ORDER		256U
#define FIR_COEFF_COUNT	(FIR_ORDER + 1U)
#define BLOCK_SIZE		16U

/*******************************************************************************
 * Exported variables
 ******************************************************************************/
extern const float32_t fir_filter_coeff_f32 [FIR_COEFF_COUNT];
#ifndef Q31_USED
extern float32_t fir_state_f32_1[BLOCK_SIZE + FIR_COEFF_COUNT - 1];
extern float32_t fir_state_f32_2[BLOCK_SIZE + FIR_COEFF_COUNT - 1];
#else
extern q31_t fir_filter_coeff_q31 [FIR_COEFF_COUNT];
extern q31_t fir_state_q31_1[BLOCK_SIZE + FIR_COEFF_COUNT - 1];
extern q31_t fir_state_q31_2[BLOCK_SIZE + FIR_COEFF_COUNT - 1];
#endif

#endif /* FILTERS_CFG_H */
