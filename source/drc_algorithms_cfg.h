/*
 * drc_algorithms_hifi4_cfg.h
 *
 *  Created on: 22 maj 2023
 *      Author: Lukasz
 */

#ifndef _DRC_ALGORITHMS_HIFI4_CONF_H_
#define _DRC_ALGORITHMS_HIFI4_CONF_H_

/*******************************************************************************
 * Definitions
 ******************************************************************************/
//#define _DEBUG
//#define _16bit

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*!
 * Limiter:
 * 	- peak:			t_AT = {0.02 ... 10.24 ms}. t_RT = {1 ... 5000 ms}
 * 	- ctrl factor:	t_AT = {0.02 ... 10.24 ms}. t_RT = {1 ... 5000 ms}
 * Compressor/Expander/Noise Gate:
 * 	- RMS:			t_AT = {5 ms}. t_RT = {130 ms}
 * 	- ctrl factor:	t_AT = {0.16 ... 2600 ms}. t_RT = {1 ... 5000 ms}
 * 	- :
 */

/* attack time */
#define t_at_ms 		0.05f
/* attack time ctrl_factor */
#define t_at_cf_ms 		0.1f
/* release time */
#define t_re_ms 		1.0f
/* release time ctrl_factor */
#define t_re_cf_ms 		5.0f
/* averaging time */
#define t_tav_ms 		0.5f
/* sampling frequency in Hz */
#define fs_hz 			48000.0f

#if !(Q31_USED)
/* limiter slope */
#define LS 				1.0f
/* compressor slope */
#define CS 				0.5f
/* expander slope */
#define ES 				-0.5f
/* noise gate slope */
#define NS 				-5.0f

#define MAX_AMPL		INT32_MAX
#define HALF_SQRT_2		0.707f

/* limiter threshold */
#define LT 				((float32_t)(MAX_AMPL * 0.8f * HALF_SQRT_2))
/* compressor threshold */
#define CT 				((float32_t)(MAX_AMPL * 0.6f * HALF_SQRT_2))
/* expander threshold */
#define ET 				((float32_t)(MAX_AMPL * 0.4f * HALF_SQRT_2))
/* noise gate threshold */
#define NT 				((float32_t)(MAX_AMPL * 0.2f * HALF_SQRT_2))
/* noise gate threshold lower */
#define NT_MUTE			((float32_t)(MAX_AMPL * 0.1f * HALF_SQRT_2))
#else
/* limiter slope */
#define LS_INV			(q31_t)0x7fffffff
#define LS_INV_F32		1.0f
/* compressor slope */
#define CS 				(q31_t)0x40000000
#define CS_F32			0.5f
/* expander slope */
#define ES				(q31_t)0xc0000000
#define ES_F32			-0.5f
/* noise gate slope */
#define NS_INV			(q31_t)0xf3333333
#define NS_INV_F32		-0.1f

#define MAX_AMPL		(q31_t)0x7fffffff
#define HALF_SQRT_2_Q31	0x5a7ef9db

/* limiter threshold */
#define LT 				0x48686148 /* 1.0 * 0.8  * sqrt(2) * 0.5 */
/* compressor threshold */
#define CT 				0x364e48f6 /* 1.0 * 0.6  * sqrt(2) * 0.5 */
/* expander threshold */
#define ET 				0x243430a4 /* 1.0 * 0.4  * sqrt(2) * 0.5 */
/* noise gate threshold upper */
#define NT 				0x121a1852 /* 1.0 * 0.2  * sqrt(2) * 0.5 */
/* noise gate threshold lower*/
#define NT_MUTE			0x0d93923d /* 1.0 * 0.15 * sqrt(2) * 0.5 */
#endif /* Q31_USED */


#endif /* _DRC_ALGORITHMS_HIFI4_CONF_H_ */
