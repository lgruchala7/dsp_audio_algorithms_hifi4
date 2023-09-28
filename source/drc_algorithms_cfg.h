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
#define MAX_AMPL INT32_MAX

/*!
 * Limiter:
 * 	- peak:			t_AT = {0.02 ... 10.24 ms}. t_RT = {1 ... 5000 ms}
 * 	- ctrl factor:	t_AT = {0.02 ... 10.24 ms}. t_RT = {1 ... 5000 ms}
 * Compressor/Expander/Noise Gate:
 * 	- RMS:			t_AT = {5 ms}. t_RT = {130 ms} ??
 * 	- ctrl factor:	t_AT = {0.16 ... 2600 ms}. t_RT = {1 ... 5000 ms}
 * 	- :
 */

/* attack time for limiter */
#define t_at_ms 			0.1f
/* release time for limiter */
#define t_re_ms 			10.0f
/* averaging time */
#define t_tav_ms 			10.0f
/* attack time ctrl_factor for limiter */
#define t_at_cf_1_ms 		0.1f
/* release time ctrl_factor for limiter */
#define t_re_cf_1_ms 		10.0f
/* attack time ctrl_factor */
#define t_at_cf_2_ms 		0.8f
/* release time ctrl_factor */
#define t_re_cf_2_ms 		80.0f
/* sampling frequency in Hz*/
#define fs_hz 				48000.0f

#if !(Q31_USED)
/* limiter slope */
#define LS 				1.0f
/* compressor slope */
#define CS 				0.25f
/* expander slope */
#define ES 				-0.25f
/* noise gate slope */
#define NS 				-10.0f

/* limiter threshold */
#define LT 				((float32_t)(MAX_AMPL * 0.9f  * 0.5f))
/* compressor threshold */
#define CT 				((float32_t)(MAX_AMPL * 0.8f  * 0.5f))
/* expander threshold */
#define ET 				((float32_t)(MAX_AMPL * 0.3f  * 0.5f))
/* noise gate threshold */
#define NT 				((float32_t)(MAX_AMPL * 0.1f  * 0.5f))
/* noise gate threshold lower */
#define NT_MUTE			((float32_t)(MAX_AMPL * 0.06f * 0.5f))
#else
/* limiter slope */
#define LS_INV			(q31_t)0x7fffffff
#define LS_INV_F32		1.0f
/* compressor slope */
#define CS 				(q31_t)0x20000000
#define CS_F32			0.25f
/* expander slope */
#define ES				(q31_t)0xe0000000
#define ES_F32			-0.25f
/* noise gate slope */
#define NS_INV			(q31_t)0xf3333333
#define NS_INV_F32		-0.1f

/* limiter threshold */
#define LT 				0x73333333 /* 1.0 * 0.9  * 0.5 */
/* compressor threshold */
#define CT 				0x33333333 /* 1.0 * 0.8  * 0.5 */
/* expander threshold */
#define ET 				0x13333333 /* 1.0 * 0.3  * 0.5 */
/* noise gate threshold upper */
#define NT 				0x06666666 /* 1.0 * 0.1  * 0.5 */
/* noise gate threshold lower*/
#define NT_MUTE			0x03d70a3d /* 1.0 * 0.06 * 0.5 */
#endif /* Q31_USED */


#endif /* _DRC_ALGORITHMS_HIFI4_CONF_H_ */
