/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/
/*
 * Copyright (c) 2016, Alliance for Open Media. All rights reserved
 *
 * This source code is subject to the terms of the BSD 2 Clause License and
 * the Alliance for Open Media Patent License 1.0. If the BSD 2 Clause License
 * was not distributed with this source code in the LICENSE file, you can
 * obtain it at www.aomedia.org/license/software. If the Alliance for Open
 * Media Patent License 1.0 was not distributed with this source code in the
 * PATENTS file, you can obtain it at www.aomedia.org/license/patent.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "EbPictureControlSet.h"
#include "EbMotionEstimationProcess.h"
#include "EbSequenceControlSet.h"
#include "EbDefinitions.h"
#include "EbBitstreamUnit.h"

#define ALTREF_MAX_NFRAMES 10

#define COLOR_CHANNELS 3
#define C_Y 0
#define C_U 1
#define C_V 2

#define EDGE_THRESHOLD 50
#define SQRT_PI_BY_2 1.25331413732
#define SMOOTH_THRESHOLD 16
// Block size used in temporal filtering
#define BW 64
#define BH 64
#define BW_CH BW>>1
#define BH_CH BH>>1
#define BLK_PELS 4096  // Pixels in the block
#define N_16X16_BLOCKS 16
#define N_32X32_BLOCKS 4

#define INT_MAX 2147483647 //max value for an int
#define INT_MIN (-2147483647-1) //min value for an int
#define THR_SHIFT 2 // should be 2

#define INIT_WEIGHT 2
#define WEIGHT_MULTIPLIER 16

// ALT-REF debug-specific defines
#define DEBUG_TEMPORAL_FILTER 0
#define DEBUG_MC 0
#define VANILA_ME 0
#define LIBAOM_FILTERING 0
#define MC_CHROMA 1
#define USE_ONLY_16X16 1
#define USE_ONLY_32X32 0
#define LIBAOM_VF 1
#define USE_SSE4_FW_32X32 1
#define USE_C_FW_32x32 0

#if LIBAOM_VF
#define THRES_LOW 10000
#define THRES_HIGH 20000
#define THRES_DIFF_LOW 6000
#define THRES_DIFF_HIGH 12000
#else
#define THRES_LOW 768 // mean SAD of 3
#define THRES_HIGH 1280 // mean SAD of 5
#define THRES_DIFF_LOW 300
#define THRES_DIFF_HIGH 450
#endif

#define _MM_HINT_T2  1
#define OD_DIVU_DMAX (1024)

#if MOVE_TF


EbErrorType init_temporal_filtering(PictureParentControlSet **list_picture_control_set_ptr,
#if FIX_SHORT
    PictureParentControlSet *picture_control_set_ptr_central,
#endif
#if ME_CLEAN
	MotionEstimationContext_t *me_context_ptr,
#endif
	int32_t segment_index);

#else
EbErrorType init_temporal_filtering(PictureParentControlSet **list_picture_control_set_ptr);
#endif