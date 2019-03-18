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

static void adjust_filter_params(int* altref_strength, int* altref_nframes);
static double estimate_noise(const uint8_t *src, int width, int height, int stride, int edge_thresh);
static double highbd_estimate_noise(const uint8_t *src8, int width, int height, int stride, int bd, int edge_thresh);
void init_temporal_filtering(int altref_strength, int altref_nframes);