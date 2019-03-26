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
#include "EbSequenceControlSet.h"
#include "EbDefinitions.h"

static void adjust_filter_params(EbPictureBufferDesc_t *input_picture_ptr, uint8_t *altref_strength, uint8_t *altref_nframes);
static double estimate_noise(EbByte src, uint16_t width, uint16_t height, uint16_t stride_y);
static double highbd_estimate_noise(const uint8_t *src8, int width, int height, int stride, int bd, int edge_thresh);
EbErrorType init_temporal_filtering(PictureParentControlSet_t *picture_control_set_ptr);