/*
 * filters_cfg.h
 *
 *  Created on: 14 sie 2023
 *      Author: £ukasz
 */

#ifndef _FILTERS_CFG_H_
#define _FILTERS_CFG_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "main_hifi4.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define FIR_ORDER			256
#define FIR_COEFF_COUNT		(FIR_ORDER + 1)
#define FIR_BLOCK_SIZE		256
#define IIR_SOS				33
#define COEFFS_PER_STAGE	5
#define IIR_COEFF_COUNT		(IIR_SOS * COEFFS_PER_STAGE)
#define IIR_BLOCK_SIZE		16
#define IIR_Q31_POSTSHIFT	0

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void init_fir_filter(void);
void init_iir_df1_filter(void);
void init_iir_df2T_filter(void);
void init_iir_df2_filter(void);
#if !(Q31_USED)
void fir_process_batch(float32_t * src_buffer, float32_t * dst_buffer);
void iir_df1_process_batch(float32_t * src_buffer, float32_t * dst_buffer);
void iir_df2T_process_batch(float32_t * src_buffer, float32_t * dst_buffer);
#else
void fir_process_batch(q31_t * src_buffer, q31_t * dst_buffer);
void iir_df1_process_batch(q31_t * src_buffer, q31_t * dst_buffer);
void iir_df2T_process_batch(q31_t * src_buffer, q31_t * dst_buffer);
#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/
#if !(Q31_USED)
extern arm_fir_instance_f32 fir_instance_f32_1;
extern arm_fir_instance_f32 fir_instance_f32_2;
extern float32_t fir_state_f32_1[FIR_BLOCK_SIZE + FIR_COEFF_COUNT - 1];
extern float32_t fir_state_f32_2[FIR_BLOCK_SIZE + FIR_COEFF_COUNT - 1];
extern arm_biquad_casd_df1_inst_f32 iir_df1_instance_f32_1;
extern arm_biquad_casd_df1_inst_f32 iir_df1_instance_f32_2;
extern float32_t iir_df1_state_f32_1[IIR_SOS * 4];
extern float32_t iir_df1_state_f32_2[IIR_SOS * 4];
#else
extern arm_fir_instance_q31 fir_instance_q31_1;
extern arm_fir_instance_q31 fir_instance_q31_2;
extern q31_t fir_state_q31_1[FIR_BLOCK_SIZE + FIR_COEFF_COUNT - 1];
extern q31_t fir_state_q31_2[FIR_BLOCK_SIZE + FIR_COEFF_COUNT - 1];
extern q31_t fir_coeff_q31[FIR_COEFF_COUNT];
#endif

#endif /* _FILTERS_CFG_H */
