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
#define DEBUG 1

static unsigned int index_mult[14] = {
        0, 0, 0, 0, 49152, 39322, 32768, 28087, 24576, 21846, 19661, 17874, 0, 15124
};

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

static INLINE int mod_index(int sum_dist,
                            int index,
                            int rounding,
                            int strength,
                            int filter_weight) {

    assert(index >= 0 && index <= 13);
    assert(index_mult[index] != 0);

    int mod = (clamp(sum_dist, 0, UINT16_MAX) * index_mult[index]) >> 16;
    mod += rounding;
    mod >>= strength;

    mod = AOMMIN(16, mod);

    mod = 16 - mod;
    mod *= filter_weight;

    return mod;

}

static INLINE void calculate_squared_errors(const uint8_t *s,
                                            int s_stride,
                                            const uint8_t *p,
                                            int p_stride,
                                            uint16_t *diff_sse,
                                            unsigned int w,
                                            unsigned int h) {

    int idx = 0;
    unsigned int i, j;

    for (i = 0; i < h; i++) {
        for (j = 0; j < w; j++) {
            const int16_t diff = s[i * s_stride + j] - p[i * p_stride + j];
            diff_sse[idx] = diff * diff;
            idx++;
        }
    }

}


void apply_filtering(const uint8_t *y_orig,
                    int y_stride,
                    const uint8_t *y_pred,
                    int y_buf_stride,
                    const uint8_t *u_orig,
                    const uint8_t *v_orig,
                    int uv_stride,
                    const uint8_t *u_pred,
                    const uint8_t *v_pred,
                    int uv_buf_stride,
                    unsigned int block_width,
                    unsigned int block_height,
                    int ss_x,
                    int ss_y,
                    int strength,
                    const int *blk_fw,
                    int use_32x32,
                    uint32_t *y_accumulator,
                    uint16_t *y_count,
                    uint32_t *u_accumulator,
                    uint16_t *u_count,
                    uint32_t *v_accumulator,
                    uint16_t *v_count) {

    unsigned int i, j, k, m;
    int modifier;
    const int rounding = (1 << strength) >> 1;
    const unsigned int uv_block_width = block_width >> ss_x;
    const unsigned int uv_block_height = block_height >> ss_y;
    DECLARE_ALIGNED(16, uint16_t, y_diff_sse[BLK_PELS]);
    DECLARE_ALIGNED(16, uint16_t, u_diff_sse[BLK_PELS]);
    DECLARE_ALIGNED(16, uint16_t, v_diff_sse[BLK_PELS]);

    int idx = 0, idy;

    memset(y_diff_sse, 0, BLK_PELS * sizeof(uint16_t));
    memset(u_diff_sse, 0, BLK_PELS * sizeof(uint16_t));
    memset(v_diff_sse, 0, BLK_PELS * sizeof(uint16_t));

    // Calculate squared differences for each pixel of the block (pred-orig)
    calculate_squared_errors(y_orig, y_stride, y_pred, y_buf_stride, y_diff_sse,
                             block_width, block_height);
    calculate_squared_errors(u_orig, uv_stride, u_pred, uv_buf_stride,
                             u_diff_sse, uv_block_width, uv_block_height);
    calculate_squared_errors(v_orig, uv_stride, v_pred, uv_buf_stride,
                             v_diff_sse, uv_block_width, uv_block_height);

    for (i = 0, k = 0, m = 0; i < block_height; i++) {
        for (j = 0; j < block_width; j++) {
            const int pixel_value = y_pred[i * y_buf_stride + j];
            int filter_weight =
                    get_filter_weight(i, j, block_height, block_width, blk_fw, use_32x32);

            // non-local mean approach
            int y_index = 0;

            const int uv_r = i >> ss_y;
            const int uv_c = j >> ss_x;
            modifier = 0;

            for (idy = -1; idy <= 1; ++idy) {
                for (idx = -1; idx <= 1; ++idx) {
                    const int row = (int)i + idy;
                    const int col = (int)j + idx;

                    if (row >= 0 && row < (int)block_height && col >= 0 &&
                        col < (int)block_width) {
                        modifier += y_diff_sse[row * (int)block_width + col];
                        ++y_index;
                    }
                }
            }

            assert(y_index > 0);

            modifier += u_diff_sse[uv_r * uv_block_width + uv_c];
            modifier += v_diff_sse[uv_r * uv_block_width + uv_c];

            y_index += 2;

            modifier =
                    (int)mod_index(modifier, y_index, rounding, strength, filter_weight);

            y_count[k] += modifier;
            y_accumulator[k] += modifier * pixel_value;

            ++k;

            // Process chroma component
            if (!(i & ss_y) && !(j & ss_x)) {
                const int u_pixel_value = u_pred[uv_r * uv_buf_stride + uv_c];
                const int v_pixel_value = v_pred[uv_r * uv_buf_stride + uv_c];

                // non-local mean approach
                int cr_index = 0;
                int u_mod = 0, v_mod = 0;
                int y_diff = 0;

                for (idy = -1; idy <= 1; ++idy) {
                    for (idx = -1; idx <= 1; ++idx) {
                        const int row = uv_r + idy;
                        const int col = uv_c + idx;

                        if (row >= 0 && row < (int)uv_block_height && col >= 0 &&
                            col < (int)uv_block_width) {
                            u_mod += u_diff_sse[row * uv_block_width + col];
                            v_mod += v_diff_sse[row * uv_block_width + col];
                            ++cr_index;
                        }
                    }
                }

                assert(cr_index > 0);

                for (idy = 0; idy < 1 + ss_y; ++idy) {
                    for (idx = 0; idx < 1 + ss_x; ++idx) {
                        const int row = (uv_r << ss_y) + idy;
                        const int col = (uv_c << ss_x) + idx;
                        y_diff += y_diff_sse[row * (int)block_width + col];
                        ++cr_index;
                    }
                }

                u_mod += y_diff;
                v_mod += y_diff;

                u_mod =
                        (int)mod_index(u_mod, cr_index, rounding, strength, filter_weight);
                v_mod =
                        (int)mod_index(v_mod, cr_index, rounding, strength, filter_weight);

                u_count[m] += u_mod;
                u_accumulator[m] += u_mod * u_pixel_value;
                v_count[m] += v_mod;
                v_accumulator[m] += v_mod * v_pixel_value;

                ++m;
            }
        }
    }
}


// Only used in single plane case
void apply_filtering_single_plane(uint8_t *orig,
                    unsigned int stride,
                    uint8_t *pred,
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
            int pixel_value = *pred;
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
                        int diff = orig[byte + idy * (int)stride + idx] -
                                   pred[idy * (int)block_width + idx];
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

            ++pred;

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
static void apply_filtering_central(const uint8_t *y_pred,
                                    const uint8_t *u_pred,
                                    const uint8_t *v_pred,
                                    uint32_t *y_accumulator,
                                    uint32_t *u_accumulator,
                                    uint32_t *v_accumulator,
                                    uint16_t *y_count,
                                    uint16_t *u_count,
                                    uint16_t *v_count,
                                    int blk_height,
                                    int blk_width) {

    unsigned int i, j, k;
    int blk_height_y = blk_height;
    int blk_width_y = blk_width;
    int blk_height_ch= blk_height>>1; // only works for 420 now!
    int blk_width_ch = blk_width>>1; // only works for 420 now!
    int blk_stride_y = blk_width;
    int blk_stride_ch = blk_width>>1; // only works for 420 now!

    int filter_weight = 2; // TODO: defines with these constants
    const int modifier = filter_weight * 16; // TODO: defines with these constants

    k = 0;
    for (i = 0; i < blk_height_y; i++) {
        for (j = 0; j < blk_width_y; j++) {
            y_accumulator[k] += modifier * y_pred[i * blk_stride_y + j];
            y_count[k] += modifier;
            ++k;
        }
    }

    k = 0;
    for (i = 0; i < blk_height_ch; i++) {
        for (j = 0; j < blk_width_ch; j++) {
            u_accumulator[k] += modifier * u_pred[i * blk_stride_ch + j];
            u_count[k] += modifier;

            v_accumulator[k] += modifier * v_pred[i * blk_stride_ch + j];
            v_count[k] += modifier;
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
    EbByte src_y, src_u, src_v;
    int stride_y = input_picture_ptr->stride_y;
    int stride_ch = input_picture_ptr->strideCb;
    int blk_width_ch = BW >> 1; // 420
    int blk_height_ch = BW >> 1; // 420
    int blk_y_offset = 0, blk_y_src_offset = 0, blk_ch_offset = 0, blk_ch_src_offset = 0;

    // index of the center frame
    index_center += (altref_nframes + 1) & 0x1;

    // first position of the buffer
    src_y = input_picture_ptr->buffer_y +
          input_picture_ptr->origin_y*input_picture_ptr->stride_y +
          input_picture_ptr->origin_x;

    src_u = input_picture_ptr->bufferCb +
            (input_picture_ptr->origin_y>>1)*input_picture_ptr->strideCb +
            (input_picture_ptr->origin_x>>1);

    src_v = input_picture_ptr->bufferCr +
            (input_picture_ptr->origin_y>>1)*input_picture_ptr->strideCr +
            (input_picture_ptr->origin_x>>1);

    // for each block
    for (blk_row = 0; blk_row < blk_rows; blk_row++) {

        for (blk_col = 0; blk_col < blk_cols; blk_col++) {

            // reset accumulator and count
            memset(accumulator, 0, BLK_PELS * 3 * sizeof(accumulator[0]));
            memset(count, 0, BLK_PELS * 3 * sizeof(count[0]));

            // for every frame to filter
            for (frame_index = 0; frame_index < (int) altref_nframes; frame_index++) {

                // Step 1: motion compensation TODO: motion compensation
                // Just for testing purposes - copy the orignal block (Y only now)
                memcpy(predictor, src_y + blk_col * BH * stride_y + blk_row, BLK_PELS * sizeof(uint8_t));
                memcpy(predictor + BLK_PELS, src_u + blk_col * BH * stride_ch + blk_row, BLK_PELS * sizeof(uint8_t));
                memcpy(predictor + (BLK_PELS << 1), src_v + blk_col * BH * stride_ch + blk_row, BLK_PELS * sizeof(uint8_t));

                // Step 2: temporal filtering using the motion compensated blocks

                // if frame to process is the center frame
                if (frame_index == 0) { // delete
                    //if (frame_index == index_center) {

                    apply_filtering_central(predictor,
                                            predictor + BLK_PELS,
                                            predictor + (BLK_PELS << 1),
                                            accumulator,
                                            accumulator + BLK_PELS,
                                            accumulator + (BLK_PELS << 1),
                                            count,
                                            count + BLK_PELS,
                                            count + (BLK_PELS << 1),
                                            BH,
                                            BW);
                }else{

//                    apply_filtering(y_orig,
//                                    y_stride,
//                                    y_pred,
//                                    y_buf_stride,
//                                    u_orig,
//                                    v_orig,
//                                    uv_stride,
//                                    u_pred,
//                                    v_pred,
//                                    uv_buf_stride,
//                                    block_width,
//                                    block_height,
//                                    ss_x,
//                                    ss_y,
//                                    strength,
//                                    blk_fw,
//                                    use_32x32,
//                                    y_accumulator,
//                                    y_count,
//                                    u_accumulator,
//                                    u_count,
//                                    v_accumulator,
//                                    v_count)
                }
            }
            // this is how libaom was implementing the blocks indexes (I did differently in the memcpy above)
            blk_y_offset += BW;
            blk_y_src_offset += BW;
            blk_ch_offset += blk_width_ch;
            blk_ch_src_offset += blk_width_ch;
        }
        blk_y_offset += BH * stride_y - BW * blk_cols;
        blk_y_src_offset += BH * stride_y - BW * blk_cols;
        blk_ch_offset += blk_height_ch * stride_ch - blk_width_ch * blk_cols;
        blk_ch_src_offset += stride_ch - blk_width_ch * blk_cols;
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

#if DEBUG
    printf("Estimated noise level: %lf\n", sigma);
#endif

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
        // if noiselevel < 0, it means that the estimation was
        // unsuccessful and therefore keep the strength as it was set
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
#if DEBUG
    printf("[noise level: %g, strength = %d, adj_strength = %d]\n", noiselevel, strength, adj_strength);
#endif

    // TODO: does it make sense to use negative strength after it has been adjusted?
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
#if DEBUG
        printf("It is not an alt-ref filtered frame");
#endif
    }
    else{
        // temporal filtering is on
        // is not showable frame
#if DEBUG
        printf("It is an alt-ref filtered frame");
#endif
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