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
#include "EbTemporalFiltering.h"

#define MAX_FRAMES_TO_FILTER 10
#define EDGE_THRESHOLD 50
#define SQRT_PI_BY_2 1.25331413732
#define SMOOTH_THRESHOLD 16
// Block size used in temporal filtering
#define TF_BLOCK BLOCK_32X32
#define BH 32
#define BW 32
#define BLK_PELS 1024  // Pixels in the block
#define THR_SHIFT 2
#define TF_SUB_BLOCK BLOCK_16X16
#define SUB_BH 16
#define SUB_BW 16

static INLINE int get_filter_weight(unsigned int i,
                                    unsigned int j,
                                    unsigned int block_height,
                                    unsigned int block_width,
                                    const int *blk_fw,
                                    int use_32x32) {

    if (use_32x32)
        // blk_fw[0] ~ blk_fw[3] are the same.
        return blk_fw[0];

    int filter_weight = 0;
    if (i < block_height / 2) {
        if (j < block_width / 2)
            filter_weight = blk_fw[0];
        else
            filter_weight = blk_fw[1];
    } else {
        if (j < block_width / 2)
            filter_weight = blk_fw[2];
        else
            filter_weight = blk_fw[3];
    }
    return filter_weight;

}

// Only used in single plane case
void apply_filtering(uint8_t *frame1,
                    unsigned int stride,
                    uint8_t *frame2,
                    unsigned int block_width,
                    unsigned int block_height,
                    int strength,
                    const int *blk_fw,
                    int use_32x32,
                    unsigned int *accumulator,
                    uint16_t *count) {

    unsigned int i, j, k;
    int modifier;
    int byte = 0;
    const int rounding = strength > 0 ? 1 << (strength - 1) : 0;

    for (i = 0, k = 0; i < block_height; i++) {
        for (j = 0; j < block_width; j++, k++) {
            int pixel_value = *frame2;
            int filter_weight =
                    get_filter_weight(i, j, block_height, block_width, blk_fw, use_32x32);

            // non-local mean approach
            int diff_sse[9] = { 0 };
            int idx, idy, index = 0;

            for (idy = -1; idy <= 1; ++idy) {
                for (idx = -1; idx <= 1; ++idx) {
                    int row = (int)i + idy;
                    int col = (int)j + idx;

                    if (row >= 0 && row < (int)block_height && col >= 0 &&
                        col < (int)block_width) {
                        int diff = frame1[byte + idy * (int)stride + idx] -
                                   frame2[idy * (int)block_width + idx];
                        diff_sse[index] = diff * diff;
                        ++index;
                    }
                }
            }

            assert(index > 0);

            modifier = 0;
            for (idx = 0; idx < 9; ++idx) modifier += diff_sse[idx];

            modifier *= 3;
            modifier /= index;

            ++frame2;

            modifier += rounding;
            modifier >>= strength;

            if (modifier > 16) modifier = 16;

            modifier = 16 - modifier;
            modifier *= filter_weight;

            count[k] += modifier;
            accumulator[k] += modifier * pixel_value;

            byte++;
        }

        byte += stride - block_width;
    }
}

// buf_stride is the block size: 32 for Y and 16 for U and V
static void apply_filtering_self(const uint8_t *pred,
                                 int blk_luma_height,
                                 int blk_luma_width,
                                 int blk_chroma_height,
                                 int blk_chroma_width,
                                 uint32_t *accumulator,
                                 uint16_t *count) {

    int filter_weight = 2;
    const int modifier = filter_weight * 16; // TODO: defines with these constants
    unsigned int i, j, k = 0;
    int block_height = blk_luma_height;
    int block_width = blk_luma_width;
    int buf_stride = blk_luma_height;

    for (i = 0; i < block_height; i++) {
        for (j = 0; j < block_width; j++) {
            const int pixel_value = pred[i * buf_stride + j];
            count[k] += modifier;
            accumulator[k] += modifier * pixel_value;
            ++k;
        }
    }

}

// Produce the filtered alt-ref picture
static void produce_temporally_filtered_pic(EbPictureBufferDesc_t *input_picture_ptr,
                                            uint8_t altref_strength,
                                            uint8_t altref_nframes) {

    int frame_index;
    DECLARE_ALIGNED(16, uint32_t, accumulator[BLK_PELS * 3]);
    DECLARE_ALIGNED(16, uint16_t, count[BLK_PELS * 3]);
    DECLARE_ALIGNED(32, uint8_t, predictor[BLK_PELS * 3]);
    const int blk_chroma_height = BH >> 1; // TODO: implement for 420 now and extend it later
    const int blk_chroma_width = BW >> 1;
    int index_center = (altref_nframes - 1) >> 1;
    int blk_row, blk_col;
    int blk_cols = (input_picture_ptr->width + BW - 1) / BW; // I think only the part of the picture
    int blk_rows = (input_picture_ptr->height + BH - 1) / BH; // that fits to the 32x32 blocks are actually filtered
    EbByte src;
    int stride_y = input_picture_ptr->stride_y;

    // index of the center frame
    index_center += (altref_nframes + 1) & 0x1;

    // first position of the buffer
    src = input_picture_ptr->buffer_y +
          input_picture_ptr->origin_y*input_picture_ptr->stride_y +
          input_picture_ptr->origin_x;
    
    // for each block
    for (blk_row = 0; blk_row < blk_rows; blk_row++) {

        for (blk_col = 0; blk_col < blk_cols; blk_col++) {

            // reset accumulator and count
            memset(accumulator, 0, BLK_PELS * 3 * sizeof(accumulator[0]));
            memset(count, 0, BLK_PELS * 3 * sizeof(count[0]));

            // for every frame to filter
            for(frame_index=0; frame_index < (int)altref_nframes; frame_index++){

                // Step 1: motion compensation TODO: motion compensation
                // Just for testing purposes - copy the original block (Y only now)
                memcpy(predictor, src + blk_col * BH * stride_y + blk_row, BLK_PELS * sizeof(uint8_t));

                // Step 2: temporal filtering using the motion compensated blocks

                // if frame to process is the center frame
                if (frame_index == 0) { // delete
                //if (frame_index == index_center) {

                    apply_filtering_self(predictor,
                                         BH,
                                         BW,
                                         blk_chroma_height,
                                         blk_chroma_width,
                                         accumulator,
                                         count);

                }

            }
        }
    }

}

// This is an adaptation of the mehtod in the following paper:
// Shen-Chuan Tai, Shih-Ming Yang, "A fast method for image noise
// estimation using Laplacian operator and adaptive edge detection,"
// Proc. 3rd International Symposium on Communications, Control and
// Signal Processing, 2008, St Julians, Malta.
// Return noise estimate, or -1.0 if there was a failure
// function from libaom
// Standard bit depht input (=8 bits) to estimate the noise, I don't think there needs to be two methods for this
// Operates on the Y component only
static double estimate_noise(EbByte src, uint16_t width, uint16_t height,
                             uint16_t stride_y) {
    int64_t sum = 0;
    int64_t num = 0;

    for (int i = 1; i < height - 1; ++i) {
        for (int j = 1; j < width - 1; ++j) {
            const int k = i * stride_y + j;
            // Sobel gradients
            const int Gx = (src[k - stride_y - 1] - src[k - stride_y + 1]) +
                           (src[k + stride_y - 1] - src[k + stride_y + 1]) +
                           2 * (src[k - 1] - src[k + 1]);
            const int Gy = (src[k - stride_y - 1] - src[k + stride_y - 1]) +
                           (src[k - stride_y + 1] - src[k + stride_y + 1]) +
                           2 * (src[k - stride_y] - src[k + stride_y]);
            const int Ga = abs(Gx) + abs(Gy);
            if (Ga < EDGE_THRESHOLD) {  // Do not consider edge pixels to estimate the noise
                // Find Laplacian
                const int v =
                        4 * src[k] -
                        2 * (src[k - 1] + src[k + 1] + src[k - stride_y] + src[k + stride_y]) +
                        (src[k - stride_y - 1] + src[k - stride_y + 1] + src[k + stride_y - 1] +
                         src[k + stride_y + 1]);
                sum += abs(v);
                ++num;
            }
        }
    }
    // If very few smooth pels, return -1 since the estimate is unreliable
    if (num < SMOOTH_THRESHOLD)
        return -1.0;

    const double sigma = (double)sum / (6 * num) * SQRT_PI_BY_2;

    return sigma;
}

// High bit depht funcion (>8 bits) to estimate the noise, I don't think I need two functions for this. I can combine both
// function from libaom
static double highbd_estimate_noise(const uint8_t *src8, int width, int height,
                                    int stride, int bd, int edge_thresh) {
    uint16_t *src = CONVERT_TO_SHORTPTR(src8);
    int64_t sum = 0;
    int64_t num = 0;
    for (int i = 1; i < height - 1; ++i) {
        for (int j = 1; j < width - 1; ++j) {
            const int k = i * stride + j;
            // Sobel gradients
            const int Gx = (src[k - stride - 1] - src[k - stride + 1]) +
                           (src[k + stride - 1] - src[k + stride + 1]) +
                           2 * (src[k - 1] - src[k + 1]);
            const int Gy = (src[k - stride - 1] - src[k + stride - 1]) +
                           (src[k - stride + 1] - src[k + stride + 1]) +
                           2 * (src[k - stride] - src[k + stride]);
            const int Ga = ROUND_POWER_OF_TWO(abs(Gx) + abs(Gy), bd - 8);
            if (Ga < edge_thresh) {  // Smooth pixels
                // Find Laplacian
                const int v =
                        4 * src[k] -
                        2 * (src[k - 1] + src[k + 1] + src[k - stride] + src[k + stride]) +
                        (src[k - stride - 1] + src[k - stride + 1] + src[k + stride - 1] +
                         src[k + stride + 1]);
                sum += ROUND_POWER_OF_TWO(abs(v), bd - 8);
                ++num;
            }
        }
    }
    // If very few smooth pels, return -1 since the estimate is unreliable
    if (num < 16) return -1.0;

    const double sigma = (double)sum / (6 * num) * SQRT_PI_BY_2;
    return sigma;
}

// Apply buffer limits and context specific adjustments to arnr filter.
static void adjust_filter_params(EbPictureBufferDesc_t *input_picture_ptr,
                        uint64_t distance_to_key,
                        uint8_t *altref_strength,
                        uint8_t *altref_nframes) {

    int q;
    EbByte src;
    double noiselevel;
    int nframes = *altref_nframes;
    int strength = *altref_strength, adj_strength=strength;
    int frames_fwd = (nframes - 1) >> 1;
    int frames_bwd;

    // TODO: Adjust the number of forward frames if the look ahead doesn't alow it
    /*if (frames_fwd > frames_after_arf)
        frames_fwd = frames_after_arf;*/

    frames_bwd = frames_fwd;
    // if forward frames is a even number, use one more bwd frame than forward frame
    frames_bwd += (nframes + 1) & 0x1;

    // Set the baseline active filter size.
    nframes = frames_bwd + 1 + frames_fwd;

    // adjust the starting point of buffer_y of the starting pixel values of the source picture
    src = input_picture_ptr->buffer_y +
            input_picture_ptr->origin_y*input_picture_ptr->stride_y +
            input_picture_ptr->origin_x;

    // Adjust the strength based on the noise level
    noiselevel = estimate_noise(src,
            input_picture_ptr->width,
            input_picture_ptr->height,
            input_picture_ptr->stride_y);

    if (noiselevel > 0) {
        // Adjust the strength of the temporal filtering
        // based on the amount of noise present in the frame
        // adjustment in the integer range [-2, 1]
        int noiselevel_adj;
        if (noiselevel < 0.75)
            noiselevel_adj = -2;
        else if (noiselevel < 1.75)
            noiselevel_adj = -1;
        else if (noiselevel < 4.0)
            noiselevel_adj = 0;
        else
            noiselevel_adj = 1;
        adj_strength += noiselevel_adj;
    }
    printf("[noise level: %g, strength = %d, adj_strength = %d]\n", noiselevel, strength, adj_strength);

    // TODO: does it make sense to use negative strength?
    strength = adj_strength;

    // TODO: libaom applies some more refinements to the number of filtered frames
    // and the strength based on the quantization level and a stat called group_boost
    // for now, this is ignored.
    /* // Adjust the strength based on active max q.
    // (q = ) get Quantization parameter using some heuristics
    if (q > 16) {
        strength = adj_strength;
    } else {
        strength = adj_strength - ((16 - q) / 2);
        if (strength < 0)
            strength = 0;
    }
    // Adjust number of frames in filter and strength based on gf boost level.
    if (nframes > group_boost / 150) {
        nframes = group_boost / 150;
        nframes += !(nframes & 1);
    }
    if (strength > group_boost / 300) {
        strength = group_boost / 300;
    }*/

    *altref_nframes = (uint8_t)nframes;
    *altref_strength = (uint8_t)strength;
}

EbErrorType init_temporal_filtering(PictureParentControlSet_t *picture_control_set_ptr) {

    int start_frame;
    int frame;
    int frames_to_blur_backward;
    int frames_to_blur_forward;
    EbPictureBufferDesc_t *frames[MAX_FRAMES_TO_FILTER] = { NULL };
    EbBool enable_alt_refs;
    uint8_t altref_strength, altref_nframes;
    EbPictureBufferDesc_t *input_picture_ptr;

    // distance to key frame
    uint64_t distance_to_key = picture_control_set_ptr->picture_number - picture_control_set_ptr->decode_order;

    // source picture buffer
    input_picture_ptr = picture_control_set_ptr->enhanced_picture_ptr;

    // user-defined encoder parameters related to alt-refs
    altref_strength = picture_control_set_ptr->sequence_control_set_ptr->static_config.altref_strength;
    altref_nframes = picture_control_set_ptr->sequence_control_set_ptr->static_config.altref_nframes;

    // adjust filter parameter based on the estimated noise of the picture
    adjust_filter_params(input_picture_ptr, distance_to_key, &altref_strength, &altref_nframes);

    //int which_arf = gf_group->arf_update_idx[gf_group->index];

    // Set the temporal filtering status for the corresponding OVERLAY frame
    if (altref_strength == 0 && altref_nframes == 1){
        // temporal filtering is off
        // is showable frame
        printf("It is not an alt-ref filtered frame");
    }
    else{
        // temporal filtering is on
        // is not showable frame
        printf("It is an alt-ref filtered frame");
    }

    // For even length filter there is one more frame backward
    // than forward: e.g. len=6 ==> bbbAff, len=7 ==> bbbAfff.
    frames_to_blur_backward = (altref_nframes / 2);
    frames_to_blur_forward = ((altref_nframes - 1) / 2);

    start_frame = distance_to_key + frames_to_blur_forward;

    // Setup frame pointers, NULL indicates frame not included in filter.
    for (frame = 0; frame < altref_nframes; ++frame) {
        const int which_buffer = start_frame - frame;
        // get input forward or backward frames from input picture buffer
        //frames[frames_to_blur - 1 - frame] = &buf->img;
    }

    produce_temporally_filtered_pic(input_picture_ptr, altref_strength, altref_nframes);

    // Initialize errorperbit, sadperbit16 and sadperbit4.
    // rdmult = av1_compute_rd_mult_based_on_qindex(cpi, ARNR_FILT_QINDEX);
    // set_error_per_bit(&cpi->td.mb, rdmult);
    // av1_initialize_me_consts(cpi, &cpi->td.mb, ARNR_FILT_QINDEX);
    // av1_initialize_cost_tables(&cpi->common, &cpi->td.mb);

    // function that implements the temporal filtering in libaom
    //temporal_filter_iterate_c(cpi, frames, frames_to_blur, frames_to_blur_backward, strength, &sf);

    return EB_ErrorNone;
}

void init_altRefs(){

}