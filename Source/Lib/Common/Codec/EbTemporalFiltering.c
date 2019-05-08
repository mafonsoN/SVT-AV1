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
#include "EbComputeSAD.h"
#include "EbMotionEstimation.h"
#include "EbMotionEstimationProcess.h"
#include "EbMotionEstimationContext.h"
#include "EbDefinitions.h"
#include "EbLambdaRateTables.h"
#include "EbPictureAnalysisProcess.h"
#include "EbMcp.h"

#if ALTREF_FILTERING_SUPPORT

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

#define INT_MAX 2147483647 //max value for an int
#define INT_MIN (-2147483647-1) //min value for an int
#define THR_SHIFT 4
#define THRES_LOW 768 // mean SAD of 3
#define THRES_HIGH 1280 // mean SAD of 5
#define THRES_DIFF_LOW 300
#define THRES_DIFF_HIGH 450

#define INIT_WEIGHT 2
#define WEIGHT_MULTIPLIER 16

// Debug-specific defines
#define DEBUG_TEMPORAL_FILTER 0
#define VANILA_ME 0
#define LIBAOM_FILTERING 0
#define MC_CHROMA 1

#define _MM_HINT_T2  1
#define OD_DIVU_DMAX (1024)

/*#define OD_DIVU_SMALL(_x, _d)                                     \
  ((uint32_t)((OD_DIVU_SMALL_CONSTS[(_d)-1][0] * (uint64_t)(_x) + \
               OD_DIVU_SMALL_CONSTS[(_d)-1][1]) >>                \
              32) >>                                              \
   (OD_ILOG_NZ(_d) - 1))

#define OD_DIVU(_x, _d) \
  (((_d) < OD_DIVU_DMAX) ? (OD_DIVU_SMALL((_x), (_d))) : ((_x) / (_d)))
*/

static unsigned int index_mult[14] = {
        0, 0, 0, 0, 49152, 39322, 32768, 28087, 24576, 21846, 19661, 17874, 0, 15124
};

// relationship between pu_index and row and col of the 32x32 sub-blocks
static const uint32_t subblock_xy_32x32[4][2] = { {0,0}, {0,1}, {1,0}, {1,1} };

static const uint32_t subblock_xy_16x16[N_16X16_BLOCKS][2] = { {0,0}, {0,1}, {0,2}, {0,3},
                                                               {1,0}, {1,1}, {1,2}, {1,3},
                                                               {2,0}, {2,1}, {2,2}, {2,3},
                                                               {3,0}, {3,1}, {3,2}, {3,3} };

// Debug function
void print_block_uint8(EbByte src, int width, int height, int stride){

    int i, j, k=0;

    for(i=0; i<height; i++){
        for(j=0; j<width; j++){
            printf("%d ", src[k]);
            k++;
        }
        printf("\n");
        k += stride - width;
    }

}

// Debug function
void save_YUV_to_file(char *filename, EbByte buffer_y, EbByte buffer_u, EbByte buffer_v,
                      uint16_t width, uint16_t height,
                      uint16_t stride_y, uint16_t stride_u, uint16_t stride_v,
                      uint16_t origin_y, uint16_t origin_x){

    FILE *fid = NULL;
    EbByte pic_point;
    int h;

    // save current source picture to a YUV file
    if ((fid = fopen(filename, "wb")) == NULL) {
        printf("Unable to open file %s to write.\n", "temp_picture.yuv");
    }else{

        // the source picture saved in the enchanced_picture_ptr contains a border in x and y dimensions
        pic_point = buffer_y + (origin_y*stride_y) + origin_x;
        for (h = 0; h < height; h++) {
            fwrite(pic_point, 1, (size_t)width, fid);
            pic_point = pic_point + stride_y;
        }
        pic_point = buffer_u + ((origin_y>>1)*stride_u) + (origin_x>>1);
        for (h = 0; h < height>>1; h++) {
            fwrite(pic_point, 1, (size_t)width>>1, fid);
            pic_point = pic_point + stride_u;
        }
        pic_point = buffer_v + ((origin_y>>1)*stride_v) + (origin_x>>1);
        for (h = 0; h < height>>1; h++) {
            fwrite(pic_point, 1, (size_t)width>>1, fid);
            pic_point = pic_point + stride_v;
        }
        fclose(fid);
    }
}

// Debug function
void save_Y_to_file(char *filename, EbByte buffer_y,
                    uint16_t width, uint16_t height,
                    uint16_t stride_y,
                    uint16_t origin_y, uint16_t origin_x){

    FILE *fid = NULL;
    EbByte pic_point;
    int h;

    // save current source picture to a YUV file
    if ((fid = fopen(filename, "wb")) == NULL) {
        printf("Unable to open file %s to write.\n", "temp_picture.yuv");
    }else{

        // the source picture saved in the enchanced_picture_ptr contains a border in x and y dimensions
        pic_point = buffer_y + (origin_y*stride_y) + origin_x;
        for (h = 0; h < height; h++) {
            fwrite(pic_point, 1, (size_t)width, fid);
            pic_point = pic_point + stride_y;
        }
        fclose(fid);
    }
}

// Copy block/picture of size width x height from src to dst
void copy_pixels(EbByte dst, int stride_dst, EbByte src, int stride_src, int width, int height){

    int h;
    EbByte src_cpy = src, dst_cpy = dst;

    for (h=0; h<height; h++){
        memcpy(dst_cpy, src_cpy, width * sizeof(uint8_t));
        dst_cpy += stride_dst;
        src_cpy += stride_src;
    }

}

// Copy block/picture of size width x height from src to dst with origin_src and origin_dst
void copy_pixels_with_origin(EbByte dst, int stride_dst,
                          int origin_dst_y, int origin_dst_x,
                          EbByte src, int stride_src,
                          int origin_src_y, int origin_src_x,
                          int width, int height){

    int h;
    EbByte src_cpy = src + origin_src_y*stride_src + origin_src_x;
    EbByte dst_cpy = dst + origin_dst_y*stride_dst + origin_dst_x;

    for (h=0; h<height; h++){
        EB_MEMCPY(dst_cpy, src_cpy, width * sizeof(uint8_t));
        dst_cpy += stride_dst;
        src_cpy += stride_src;
    }

}

// assign a single value to all elements in an array
static void populate_list_with_value(int *list, int nelements, const int value){

    for(int i=0; i<nelements; i++){
        list[i] = value;
    }

}

// get block filter weights using a distance metric
void get_blk_fw_using_dist(int const *me_32x32_total_sad, int const *me_16x16_subblock_sad, int *use_16x16_subblocks, int *blk_fw){

    int blk_idx;

    int me_sum_subblock_sad = 0;
    int max_me_sad = INT_MIN, min_me_sad = INT_MAX;

    for (blk_idx = 0; blk_idx < N_16X16_BLOCKS; blk_idx++) {
        if (min_me_sad > me_16x16_subblock_sad[blk_idx])
            min_me_sad = me_16x16_subblock_sad[blk_idx];
        if (max_me_sad < me_16x16_subblock_sad[blk_idx])
            max_me_sad = me_16x16_subblock_sad[blk_idx];
        me_sum_subblock_sad += me_16x16_subblock_sad[blk_idx];
    }

    if (((*me_32x32_total_sad * 15 < (me_sum_subblock_sad << 4)) && max_me_sad - min_me_sad < THRES_DIFF_HIGH) ||
        ((*me_32x32_total_sad * 14 < (me_sum_subblock_sad << 4)) && max_me_sad - min_me_sad < THRES_DIFF_LOW)) {

        // do not consider MC and weighting at a sub-block level

        *use_16x16_subblocks = 0;

        int weight = *me_32x32_total_sad < (THRES_LOW << THR_SHIFT)
                    ? 2
                    : *me_32x32_total_sad < (THRES_HIGH << THR_SHIFT) ? 1 : 0;

        populate_list_with_value(blk_fw, N_16X16_BLOCKS, weight);

    } else {

        // split into 16x16 sub-blocks

        *use_16x16_subblocks = 1;

        for (blk_idx = 0; blk_idx < N_16X16_BLOCKS; blk_idx++){
            blk_fw[blk_idx] = me_16x16_subblock_sad[blk_idx] < THRES_LOW
                              ? 2
                              : me_16x16_subblock_sad[blk_idx] < THRES_HIGH ? 1 : 0;

        }

    }
}

// get distortion (SAD) from ME context
void get_ME_distortion(MeContext *context_ptr, int* me_32x32_total_sad, int *me_16x16_subblock_sad){

    uint32_t mv_index;

    *me_32x32_total_sad = 0;

    // Block 32x32 distortion
    for(uint32_t pu_index = 1; pu_index <= 4; pu_index++){

        mv_index = pu_index;

        // sad
        *me_32x32_total_sad += context_ptr->p_best_sad32x32[mv_index-1];

#if 0
        printf("[SAD on 32x32 block = %d]\n", context_ptr->p_best_sad32x32[mv_index-1]);
#endif
    }

    // 16x16 distortion
    for(uint32_t pu_index = 5; pu_index <= 20; pu_index++){

        mv_index = tab16x16[pu_index - 5] + 5;

        // sad
        me_16x16_subblock_sad[pu_index-5] = context_ptr->p_best_sad16x16[mv_index-5];

#if 0
        printf("[SAD on 16x16 block = %d]\n", me_16x16_subblock_sad[pu_index-5]);
#endif
    }

}

// Create and initialize all necessary ME context structures
void create_ME_context_and_picture_control(MotionEstimationContext_t *context_ptr,
                                            PictureParentControlSet *picture_control_set_ptr_frame,
                                            PictureParentControlSet *picture_control_set_ptr_central,
                                            EbPictureBufferDesc *input_picture_ptr_central,
                                            int blk_row,
                                            int blk_col){

    uint32_t lcuRow;

    // set reference picture for alt-refs
    context_ptr->me_context_ptr->alt_ref_reference_ptr = (EbPaReferenceObject*)picture_control_set_ptr_frame->pa_reference_picture_wrapper_ptr->object_ptr;
    context_ptr->me_context_ptr->me_alt_ref = EB_TRUE;

    signal_derivation_me_kernel_oq(
            picture_control_set_ptr_frame->sequence_control_set_ptr,
            picture_control_set_ptr_frame,
            context_ptr);

    // set the buffers with the original, quarter and sixteenth pixels version of the source frame
    EbPaReferenceObject *src_object = (EbPaReferenceObject*)picture_control_set_ptr_central->pa_reference_picture_wrapper_ptr->object_ptr;
    EbPictureBufferDesc *padded_pic_ptr = src_object->input_padded_picture_ptr;
    EbPictureBufferDesc *quarter_pic_ptr = src_object->quarter_decimated_picture_ptr;
    EbPictureBufferDesc *sixteenth_pic_ptr = src_object->sixteenth_decimated_picture_ptr;

    // Parts from MotionEstimationKernel()
    uint32_t sb_origin_x = (uint32_t)(blk_col * BW);
    uint32_t sb_origin_y = (uint32_t)(blk_row * BH);

    uint32_t sb_width = (input_picture_ptr_central->width - sb_origin_x) < BLOCK_SIZE_64 ? input_picture_ptr_central->width - sb_origin_x : BLOCK_SIZE_64;
    uint32_t sb_height = (input_picture_ptr_central->height - sb_origin_y) < BLOCK_SIZE_64 ? input_picture_ptr_central->height - sb_origin_y : BLOCK_SIZE_64;

    // Load the SB from the input to the intermediate SB buffer
    int bufferIndex = (input_picture_ptr_central->origin_y + sb_origin_y) * input_picture_ptr_central->stride_y + input_picture_ptr_central->origin_x + sb_origin_x;

    // set search type
    context_ptr->me_context_ptr->hme_search_type = HME_RECTANGULAR;

    // set search method
    context_ptr->me_context_ptr->hme_search_method = FULL_SAD_SEARCH;

    // set Lambda
    context_ptr->me_context_ptr->lambda = lambda_mode_decision_ra_sad[picture_control_set_ptr_central->picture_qp];

    // populate src block buffers: sb_buffer, quarter_sb_buffer and sixteenth_sb_buffer
    for (lcuRow = 0; lcuRow < BLOCK_SIZE_64; lcuRow++) {
        EB_MEMCPY((&(context_ptr->me_context_ptr->sb_buffer[lcuRow * BLOCK_SIZE_64])), (&(input_picture_ptr_central->buffer_y[bufferIndex + lcuRow * input_picture_ptr_central->stride_y])), BLOCK_SIZE_64 * sizeof(uint8_t));
    }

#if DEBUG_TEMPORAL_FILTER
#if 0
//    printf("sb_buffer:\n");
//    print_block_uint8(context_ptr->me_context_ptr->sb_buffer, 64, 64, 64);
#endif
#endif

    {
        uint8_t * src_ptr = &(padded_pic_ptr->buffer_y[bufferIndex]);

        //_MM_HINT_T0     //_MM_HINT_T1    //_MM_HINT_T2    //_MM_HINT_NTA
        uint32_t i;
        for (i = 0; i < sb_height; i++)
        {
            char const* p = (char const*)(src_ptr + i * padded_pic_ptr->stride_y);
            _mm_prefetch(p, _MM_HINT_T2);
        }
    }

    context_ptr->me_context_ptr->sb_src_ptr = &(padded_pic_ptr->buffer_y[bufferIndex]);
    context_ptr->me_context_ptr->sb_src_stride = padded_pic_ptr->stride_y;

    // Load the 1/4 decimated SB from the 1/4 decimated input to the 1/4 intermediate SB buffer
    bufferIndex = (quarter_pic_ptr->origin_y + (sb_origin_y >> 1)) * quarter_pic_ptr->stride_y + quarter_pic_ptr->origin_x + (sb_origin_x >> 1);

    for (lcuRow = 0; lcuRow < (sb_height >> 1); lcuRow++) {
        EB_MEMCPY((&(context_ptr->me_context_ptr->quarter_sb_buffer[lcuRow * context_ptr->me_context_ptr->quarter_sb_buffer_stride])), (&(quarter_pic_ptr->buffer_y[bufferIndex + lcuRow * quarter_pic_ptr->stride_y])), (sb_width >> 1) * sizeof(uint8_t));
    }

#if DEBUG_TEMPORAL_FILTER
#if 0
//    printf("quarter_sb_buffer:\n");
//    print_block_uint8(context_ptr->me_context_ptr->quarter_sb_buffer, 32, 32, 32);
#endif
#endif

    // Load the 1/16 decimated SB from the 1/16 decimated input to the 1/16 intermediate SB buffer
    bufferIndex = (sixteenth_pic_ptr->origin_y + (sb_origin_y >> 2)) * sixteenth_pic_ptr->stride_y + sixteenth_pic_ptr->origin_x + (sb_origin_x >> 2);

    {
        uint8_t *framePtr = &(sixteenth_pic_ptr->buffer_y[bufferIndex]);
        uint8_t *localPtr = context_ptr->me_context_ptr->sixteenth_sb_buffer;

        if (context_ptr->me_context_ptr->hme_search_method == FULL_SAD_SEARCH) {
            for (lcuRow = 0; lcuRow < (sb_height >> 2); lcuRow += 1) {
                EB_MEMCPY(localPtr, framePtr, (sb_width >> 2) * sizeof(uint8_t));
                localPtr += 16;
                framePtr += sixteenth_pic_ptr->stride_y;
            }
        }
        else {
            for (lcuRow = 0; lcuRow < (sb_height >> 2); lcuRow += 2) {
                EB_MEMCPY(localPtr, framePtr, (sb_width >> 2) * sizeof(uint8_t));
                localPtr += 16;
                framePtr += sixteenth_pic_ptr->stride_y << 1;
            }
        }
#if 0
        for (lcuRow = 0; lcuRow < (sb_height >> 2); lcuRow += 2) {
            EB_MEMCPY(localPtr, framePtr, (sb_width >> 2) * sizeof(uint8_t));
            localPtr += 16;
            framePtr += sixteenth_pic_ptr->stride_y << 1;
        }
#endif

    }

#if DEBUG_TEMPORAL_FILTER
#if 0
//    printf("sixteenth_sb_buffer:\n");
//    print_block_uint8(context_ptr->me_context_ptr->sixteenth_sb_buffer, 16, 16, 16);
#endif
#endif

}

// Get sub-block filter weights
// blk_fw - block filter weight
// TODO: ugly code, clean up
static INLINE int get_subblock_filter_weight(unsigned int y,
                                    unsigned int x,
                                    unsigned int block_height,
                                    unsigned int block_width,
                                    const int *blk_fw,
                                    int use_16x16_subblocks) {

    if (!use_16x16_subblocks)
        // blk_fw[0] ~ blk_fw[3] are the same.
        return blk_fw[0];

    const double four_thirds = 1.333333;

    int filter_weight = 0;
    if (y < block_height / 4) {
        if (x < block_width / 4)
            filter_weight = blk_fw[0];
        else if(x < block_width / 2)
            filter_weight = blk_fw[1];
        else if(x < (double)block_width / four_thirds)
            filter_weight = blk_fw[2];
        else
            filter_weight = blk_fw[3];
    } else if(y < block_height / 2){
        if (x < block_width / 4)
            filter_weight = blk_fw[4];
        else if(x < block_width / 2)
            filter_weight = blk_fw[5];
        else if(x < (double)block_width / four_thirds)
            filter_weight = blk_fw[6];
        else
            filter_weight = blk_fw[7];
    } else if(y < (double)block_height / four_thirds){
        if (x < block_width / 4)
            filter_weight = blk_fw[8];
        else if(x < block_width / 2)
            filter_weight = blk_fw[9];
        else if(x < (double)block_width / four_thirds)
            filter_weight = blk_fw[10];
        else
            filter_weight = blk_fw[11];
    } else {
        if (x < block_width / 4)
            filter_weight = blk_fw[12];
        else if(x < block_width / 2)
            filter_weight = blk_fw[13];
        else if(x < (double)block_width / four_thirds)
            filter_weight = blk_fw[14];
        else
            filter_weight = blk_fw[15];
    }

    return filter_weight;

}

// Adjust value of the modified (weight of filtering) based on the distortion and strength parameter
static INLINE int adjust_modifier(int sum_dist,
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
            diff_sse[idx] = (uint16_t) diff * diff;
            idx++;
        }
    }

}

// Main function that applies filtering to a block according to weights
// TODO: libaom uses an intrinsics version of this function
void apply_filtering(EbByte *src,
                    EbByte *pred,
                    uint32_t **accum,
                    uint16_t **count,
                    int *stride_src,
                    int *stride_pred,
                    unsigned int block_width,
                    unsigned int block_height,
                    int ss_x, // chroma sub-sampling in x
                    int ss_y, // chroma sub-sampling in y
                    int strength,
                    const int *blk_fw, // sub-block filter weights
                    int use_16x16_subblocks) {

    unsigned int i, j, k, m;
    int idx, idy;
    int modifier;
    const int rounding = (1 << strength) >> 1;
    const unsigned int uv_block_width = block_width >> ss_x;
    const unsigned int uv_block_height = block_height >> ss_y;
    DECLARE_ALIGNED(16, uint16_t, y_diff_sse[BLK_PELS]);
    DECLARE_ALIGNED(16, uint16_t, u_diff_sse[BLK_PELS]);
    DECLARE_ALIGNED(16, uint16_t, v_diff_sse[BLK_PELS]);

    memset(y_diff_sse, 0, BLK_PELS * sizeof(uint16_t));
    memset(u_diff_sse, 0, BLK_PELS * sizeof(uint16_t));
    memset(v_diff_sse, 0, BLK_PELS * sizeof(uint16_t));

    EbByte src_y = src[0], src_u = src[1], src_v = src[2];
    EbByte pred_y = pred[0], pred_u = pred[1], pred_v = pred[2];
    uint32_t *accum_y = accum[0], *accum_u = accum[1], *accum_v = accum[2];
    uint16_t *count_y = count[0], *count_u = count[1], *count_v = count[2];

    // Calculate squared differences for each pixel of the block (pred-orig)
    calculate_squared_errors(src_y, stride_src[0], pred_y, stride_pred[0], y_diff_sse,
                             block_width, block_height);
    calculate_squared_errors(src_u, stride_src[1], pred_u, stride_pred[1],
                             u_diff_sse, uv_block_width, uv_block_height);
    calculate_squared_errors(src_v, stride_src[2], pred_v, stride_pred[2],
                             v_diff_sse, uv_block_width, uv_block_height);

    for (i = 0, k = 0, m = 0; i < block_height; i++) {
        for (j = 0; j < block_width; j++) {

            const int pixel_value = pred_y[i * stride_pred[0] + j];

            int filter_weight = get_subblock_filter_weight(i, j, block_height, block_width, blk_fw, use_16x16_subblocks);

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

            modifier = adjust_modifier(modifier, y_index, rounding, strength, filter_weight);

            if(modifier==0){
                modifier=0;
            }

            count_y[k] += modifier;
            accum_y[k] += modifier * pixel_value;

            ++k;

            // Process chroma component
            if (!(i & ss_y) && !(j & ss_x)) {
                const int u_pixel_value = pred_u[uv_r * stride_pred[1] + uv_c];
                const int v_pixel_value = pred_v[uv_r * stride_pred[2] + uv_c];

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

                u_mod = adjust_modifier(u_mod, cr_index, rounding, strength, filter_weight);
                v_mod = adjust_modifier(v_mod, cr_index, rounding, strength, filter_weight);

                count_u[m] += u_mod;
                accum_u[m] += u_mod * u_pixel_value;
                count_v[m] += v_mod;
                accum_v[m] += v_mod * v_pixel_value;

                ++m;
            }
        }
    }
}

// Apply filtering to the central picture
static void apply_filtering_central(EbByte *pred,
                                    uint32_t **accum,
                                    uint16_t **count,
                                    uint16_t blk_height,
                                    uint16_t blk_width) {

    EbByte pred_y = pred[0], pred_u = pred[1], pred_v = pred[2];
    uint32_t *accum_y = accum[0], *accum_u = accum[1], *accum_v = accum[2];
    uint16_t *count_y = count[0], *count_u = count[1], *count_v = count[2];

    uint16_t i, j, k;
    uint16_t blk_height_y = blk_height;
    uint16_t blk_width_y = blk_width;
    uint16_t blk_height_ch= blk_height>>1;
    uint16_t blk_width_ch = blk_width>>1;
    uint16_t blk_stride_y = blk_width;
    uint16_t blk_stride_ch = blk_width>>1;

    int filter_weight = INIT_WEIGHT;
    const int modifier = filter_weight * WEIGHT_MULTIPLIER;

    // Luma
    k = 0;
    for (i = 0; i < blk_height_y; i++) {
        for (j = 0; j < blk_width_y; j++) {
            accum_y[k] += modifier * pred_y[i * blk_stride_y + j];
            count_y[k] += modifier;
            ++k;
        }
    }

    // Chroma
    k = 0;
    for (i = 0; i < blk_height_ch; i++) {
        for (j = 0; j < blk_width_ch; j++) {
            accum_u[k] += modifier * pred_u[i * blk_stride_ch + j];
            count_u[k] += modifier;

            accum_v[k] += modifier * pred_v[i * blk_stride_ch + j];
            count_v[k] += modifier;
            ++k;
        }
    }

}

void pad_chroma_samples(EbPictureBufferDesc *pic){

#if DEBUG_TEMPORAL_FILTER
    save_Y_to_file("input_unpadded_ch.yuv", pic->buffer_cb,
                   pic->stride_cb, 360,
                   pic->stride_cb, 0, 0);
#endif

    generate_padding(pic->buffer_cb,
                     pic->stride_cb,
                     pic->width >> 1,
                     pic->height >> 1,
                     pic->origin_x >> 1,
                     pic->origin_y >> 1);

    generate_padding(pic->buffer_cr,
                     pic->stride_cr,
                     pic->width >> 1,
                     pic->height >> 1,
                     pic->origin_x >> 1,
                     pic->origin_y >> 1);

#if DEBUG_TEMPORAL_FILTER
    save_Y_to_file("input_padded_ch.yuv", pic->buffer_cb,
                   pic->stride_cb, 360,
                   pic->stride_cb, 0, 0);
#endif

}

// Unidirectional motion compensation using open-loop ME results and MC
void uni_motion_compensation(MeContext* context_ptr,
                            EbPictureBufferDesc *pic_ptr_ref,
                            EbByte *pred,
                            uint32_t sb_origin_x,
                            uint32_t sb_origin_y,
                            uint8_t **pos_b_buffer_ch,
                            uint8_t **pos_h_buffer_ch,
                            uint8_t **pos_j_buffer_ch,
                            uint8_t **one_d_intermediate_results_buf_ch,
                            int use_16x16_subblocks,
                            EbAsm asm_type){

    int16_t first_ref_pos_x;
    int16_t first_ref_pos_y;
    int16_t first_ref_integ_pos_x;
    int16_t first_ref_integ_pos_y;
    uint8_t first_ref_frac_pos_x;
    uint8_t first_ref_frac_pos_y;
    uint8_t first_ref_frac_pos;
    int32_t x_first_search_index;
    int32_t y_first_search_index;
    int32_t first_search_region_index_pos_integ;
    int32_t first_search_region_index_pos_b;
    int32_t first_search_region_index_pos_h;
    int32_t first_search_region_index_pos_j;
    EbByte  pred_ptr[COLOR_CHANNELS];
    int     row, col;

    uint8_t *input_padded_ch[2];
    uint8_t *integer_buffer_ptr_ch[2];
    uint32_t interpolated_stride_ch = MAX_SEARCH_AREA_WIDTH_CH;
    uint32_t interpolated_full_stride_ch = pic_ptr_ref->stride_cb;

    uint32_t mv_index;
    uint32_t pu_index_min, pu_index_max;
    uint16_t subblock_w, subblock_h;

    if(use_16x16_subblocks){
        // use 16x16 sub-blocks
        pu_index_min = 5;
        pu_index_max = 20;
        subblock_w = 16;
        subblock_h = 16;
    }else{
        // use 32x32 blocks
        pu_index_min = 1;
        pu_index_max = 4;
        subblock_w = 32;
        subblock_h = 32;
    }

    // ----- Interpolate chroma search area ------

    uint32_t sb_origin_x_ch = sb_origin_x / 2;
    uint32_t sb_origin_y_ch = sb_origin_y / 2;
    int x_search_area_origin_ch = context_ptr->x_search_area_origin[0][0] / 2;
    int y_search_area_origin_ch = context_ptr->y_search_area_origin[0][0] / 2;
    uint32_t search_area_width_ch = (context_ptr->search_area_width + (BLOCK_SIZE_64))/2 - 1;
    uint32_t search_area_height_ch = (context_ptr->search_area_height + (BLOCK_SIZE_64))/2 - 1;

    int x_top_left_search_region = (pic_ptr_ref->origin_x)/2 + sb_origin_x_ch - (ME_FILTER_TAP >> 1) + x_search_area_origin_ch;
    int y_top_left_search_region = (pic_ptr_ref->origin_y)/2 + sb_origin_y_ch - (ME_FILTER_TAP >> 1) + y_search_area_origin_ch;
    int searchRegionIndex_cb = x_top_left_search_region + y_top_left_search_region*pic_ptr_ref->stride_cb;
    int searchRegionIndex_cr = x_top_left_search_region + y_top_left_search_region*pic_ptr_ref->stride_cr;

    assert(x_top_left_search_region > 0);
    assert(y_top_left_search_region > 0);
    assert(x_top_left_search_region < pic_ptr_ref->stride_cb - search_area_width_ch);
    assert(y_top_left_search_region < (pic_ptr_ref->height)/2 + pic_ptr_ref->origin_y - search_area_height_ch);

    input_padded_ch[0] = pic_ptr_ref->buffer_cb;
    input_padded_ch[1] = pic_ptr_ref->buffer_cr;

    // Pad chroma reference samples - once only per picture
    if(sb_origin_x == 0 && sb_origin_y == 0){
        generate_padding(input_padded_ch[0],
                         pic_ptr_ref->stride_cb,
                         pic_ptr_ref->width >> 1,
                         pic_ptr_ref->height >> 1,
                         pic_ptr_ref->origin_x >> 1,
                         pic_ptr_ref->origin_y >> 1);

        generate_padding(input_padded_ch[1],
                         pic_ptr_ref->stride_cr,
                         pic_ptr_ref->width >> 1,
                         pic_ptr_ref->height >> 1,
                         pic_ptr_ref->origin_x >> 1,
                         pic_ptr_ref->origin_y >> 1);
    }

    integer_buffer_ptr_ch[0] = &(input_padded_ch[0][searchRegionIndex_cb]);
    integer_buffer_ptr_ch[1] = &(input_padded_ch[1][searchRegionIndex_cr]);

    interpolate_search_region_AVC_chroma(context_ptr,
                                         integer_buffer_ptr_ch[0] + (ME_FILTER_TAP >> 1) + ((ME_FILTER_TAP >> 1) * interpolated_full_stride_ch),
                                         integer_buffer_ptr_ch[1] + (ME_FILTER_TAP >> 1) + ((ME_FILTER_TAP >> 1) * interpolated_full_stride_ch),
                                         pos_b_buffer_ch,
                                         pos_h_buffer_ch,
                                         pos_j_buffer_ch,
                                         interpolated_stride_ch,
                                         interpolated_full_stride_ch,
                                         search_area_width_ch,
                                         search_area_height_ch,
                                         8, // bit depth
                                         0);

#if DEBUG_TEMPORAL_FILTER
    #if 0
        save_Y_to_file("input_padded_ch.yuv", input_padded_ch[0],
                       interpolated_full_stride_ch, 144,
                       interpolated_full_stride_ch, 0, 0);

        save_Y_to_file("pos_b_buffer_chroma.yuv", pos_b_buffer_ch[0],
                       MAX_SEARCH_AREA_WIDTH_CH, MAX_SEARCH_AREA_HEIGHT_CH,
                       MAX_SEARCH_AREA_WIDTH_CH, 0, 0);

        save_Y_to_file("pos_b_buffer_luma.yuv", context_ptr->pos_b_buffer[0][0],
                       MAX_SEARCH_AREA_WIDTH, MAX_SEARCH_AREA_HEIGHT,
                       MAX_SEARCH_AREA_WIDTH, 0, 0);
#endif
#endif

    // ----- Loop over all sub-blocks -----

    for(uint32_t pu_index = pu_index_min; pu_index <= pu_index_max; pu_index++){

        // ----- compensate luma ------

        if(use_16x16_subblocks) {
            row = subblock_xy_16x16[pu_index - pu_index_min][0];
            col = subblock_xy_16x16[pu_index - pu_index_min][1];
        }else{
            row = subblock_xy_32x32[pu_index - pu_index_min][0];
            col = subblock_xy_32x32[pu_index - pu_index_min][1];
        }

        pred_ptr[0] = pred[0] + row*subblock_h*BW + col*subblock_w;
        pred_ptr[1] = pred[1] + row*(subblock_h>>1)*(BW_CH) + col*(subblock_h>>1);
        pred_ptr[2] = pred[2] + row*(subblock_h>>1)*(BW_CH) + col*(subblock_h>>1);

        // get motion vectors
        if (use_16x16_subblocks) {
            mv_index = tab16x16[pu_index - pu_index_min];

            first_ref_pos_x = _MVXT(context_ptr->p_best_mv16x16[mv_index]);
            first_ref_pos_y = _MVYT(context_ptr->p_best_mv16x16[mv_index]);
        }
        else {
            mv_index = pu_index - pu_index_min;

            first_ref_pos_x = _MVXT(context_ptr->p_best_mv32x32[mv_index]);
            first_ref_pos_y = _MVYT(context_ptr->p_best_mv32x32[mv_index]);
        }

        first_ref_integ_pos_x = (first_ref_pos_x >> 2); // TODO: check if this left shift in an unsigned mv is correct
        first_ref_integ_pos_y = (first_ref_pos_y >> 2);
        first_ref_frac_pos_x = (uint8_t)(first_ref_pos_x & 0x03);
        first_ref_frac_pos_y = (uint8_t)(first_ref_pos_y & 0x03);

        first_ref_frac_pos = (uint8_t)(first_ref_frac_pos_x + (first_ref_frac_pos_y << 2)); // TODO: check if this right shift in an unsigned variable is correct

        x_first_search_index = (int32_t)first_ref_integ_pos_x - context_ptr->x_search_area_origin[0][0];
        y_first_search_index = (int32_t)first_ref_integ_pos_y - context_ptr->y_search_area_origin[0][0];
        first_search_region_index_pos_integ = (int32_t)(x_first_search_index + (ME_FILTER_TAP >> 1)) + (int32_t)context_ptr->interpolated_full_stride[0][0] * (int32_t)(y_first_search_index + (ME_FILTER_TAP >> 1));
        first_search_region_index_pos_b = (int32_t)(x_first_search_index + (ME_FILTER_TAP >> 1) - 1) + (int32_t)context_ptr->interpolated_stride * (int32_t)(y_first_search_index + (ME_FILTER_TAP >> 1));
        first_search_region_index_pos_h = (int32_t)(x_first_search_index + (ME_FILTER_TAP >> 1) - 1) + (int32_t)context_ptr->interpolated_stride * (int32_t)(y_first_search_index + (ME_FILTER_TAP >> 1) - 1);
        first_search_region_index_pos_j = (int32_t)(x_first_search_index + (ME_FILTER_TAP >> 1) - 1) + (int32_t)context_ptr->interpolated_stride * (int32_t)(y_first_search_index + (ME_FILTER_TAP >> 1) - 1);

        uint8_t *comp_block;
        uint32_t comp_block_stride;
        uni_pred_averaging(pu_index, // pu_index
                           EB_FALSE,
                           first_ref_frac_pos,
                           subblock_w, // pu_width
                           subblock_h, // pu_height
                           &(context_ptr->integer_buffer_ptr[0][0][first_search_region_index_pos_integ]),
                           &(context_ptr->pos_b_buffer[0][0][first_search_region_index_pos_b]),
                           &(context_ptr->pos_h_buffer[0][0][first_search_region_index_pos_h]),
                           &(context_ptr->pos_j_buffer[0][0][first_search_region_index_pos_j]),
                           context_ptr->interpolated_stride,
                           context_ptr->interpolated_full_stride[0][0],
                           &(context_ptr->one_d_intermediate_results_buf0[0]),
                           &comp_block,
                           &comp_block_stride,
                           asm_type);

#if 0
        save_Y_to_file("sub_block_Y_MC.yuv", comp_block,
                   subblock_w, subblock_h,
                       comp_block_stride, 0, 0);
#endif
        copy_pixels(pred_ptr[0], BW, comp_block, comp_block_stride, subblock_w, subblock_h);

        // ----- compensate chroma ------

#if MC_CHROMA

        // get motion vectors
        if (use_16x16_subblocks) {
            first_ref_pos_x = (int16_t)(_MVXT(context_ptr->p_best_mv16x16[mv_index])/2);
            first_ref_pos_y = (int16_t)(_MVYT(context_ptr->p_best_mv16x16[mv_index])/2);
        }
        else {
            first_ref_pos_x = (int16_t)(_MVXT(context_ptr->p_best_mv32x32[mv_index])/2);
            first_ref_pos_y = (int16_t)(_MVYT(context_ptr->p_best_mv32x32[mv_index])/2);
        }

        first_ref_integ_pos_x = (first_ref_pos_x >> 2);
        first_ref_integ_pos_y = (first_ref_pos_y >> 2);
        first_ref_frac_pos_x = (uint8_t)(first_ref_pos_x & 0x03);
        first_ref_frac_pos_y = (uint8_t)(first_ref_pos_y & 0x03);

        first_ref_frac_pos = (uint8_t)(first_ref_frac_pos_x + (first_ref_frac_pos_y << 2));

        x_first_search_index = (int32_t)first_ref_integ_pos_x - ((context_ptr->x_search_area_origin[0][0])/2);
        y_first_search_index = (int32_t)first_ref_integ_pos_y - ((context_ptr->y_search_area_origin[0][0])/2);
        first_search_region_index_pos_integ = (int32_t)(x_first_search_index + (ME_FILTER_TAP >> 1)) + interpolated_full_stride_ch * (int32_t)(y_first_search_index + (ME_FILTER_TAP >> 1));
        first_search_region_index_pos_b = (int32_t)(x_first_search_index + (ME_FILTER_TAP >> 1) - 1) + interpolated_stride_ch * (int32_t)(y_first_search_index + (ME_FILTER_TAP >> 1));
        first_search_region_index_pos_h = (int32_t)(x_first_search_index + (ME_FILTER_TAP >> 1) - 1) + interpolated_stride_ch * (int32_t)(y_first_search_index + (ME_FILTER_TAP >> 1) - 1);
        first_search_region_index_pos_j = (int32_t)(x_first_search_index + (ME_FILTER_TAP >> 1) - 1) + interpolated_stride_ch * (int32_t)(y_first_search_index + (ME_FILTER_TAP >> 1) - 1);

        assert(first_search_region_index_pos_b>=0);
        assert(first_search_region_index_pos_h>=0);
        assert(first_search_region_index_pos_j>=0);

        // compensate U
        uni_pred_averaging(pu_index, // pu_index
                           EB_TRUE,
                           first_ref_frac_pos,
                           subblock_w>>1, // pu_width
                           subblock_h>>1, // pu_height
                           &(integer_buffer_ptr_ch[0][first_search_region_index_pos_integ]),
                           &(pos_b_buffer_ch[0][first_search_region_index_pos_b]),
                           &(pos_h_buffer_ch[0][first_search_region_index_pos_h]),
                           &(pos_j_buffer_ch[0][first_search_region_index_pos_j]),
                           interpolated_stride_ch,
                           interpolated_full_stride_ch,
                           &(one_d_intermediate_results_buf_ch[0][0]),
                           &comp_block,
                           &comp_block_stride,
                           asm_type);

        copy_pixels(pred_ptr[1], BW_CH, comp_block, comp_block_stride, subblock_w>>1, subblock_h>>1);

#if 0
        save_Y_to_file("sub_block_U_MC.yuv", comp_block,
                       subblock_w>>1, subblock_h>>1,
                       comp_block_stride, 0, 0);
#endif
        // compensate V
        uni_pred_averaging(pu_index, // pu_index
                           EB_TRUE,
                           first_ref_frac_pos,
                           subblock_w>>1, // pu_width
                           subblock_h>>1, // pu_height
                           &(integer_buffer_ptr_ch[1][first_search_region_index_pos_integ]),
                           &(pos_b_buffer_ch[1][first_search_region_index_pos_b]),
                           &(pos_h_buffer_ch[1][first_search_region_index_pos_h]),
                           &(pos_j_buffer_ch[1][first_search_region_index_pos_j]),
                           interpolated_stride_ch,
                           interpolated_full_stride_ch,
                           &(one_d_intermediate_results_buf_ch[1][0]),
                           &comp_block,
                           &comp_block_stride,
                           asm_type);

        copy_pixels(pred_ptr[2], BW_CH, comp_block, comp_block_stride, subblock_w>>1, subblock_h>>1);
#endif

    }

}

// Produce the filtered alt-ref picture
static EbErrorType produce_temporally_filtered_pic(PictureParentControlSet **list_picture_control_set_ptr,
                                            EbPictureBufferDesc **list_input_picture_ptr,
                                            uint8_t altref_strength,
                                            uint8_t altref_nframes,
                                            uint8_t **alt_ref_buffer) {

    int frame_index;
    DECLARE_ALIGNED(16, uint32_t, accumulator[BLK_PELS * COLOR_CHANNELS]);
    DECLARE_ALIGNED(16, uint16_t, counter[BLK_PELS * COLOR_CHANNELS]);
    DECLARE_ALIGNED(32, uint8_t, predictor[BLK_PELS * COLOR_CHANNELS]);
    uint32_t *accum[COLOR_CHANNELS] = { accumulator, accumulator + BLK_PELS, accumulator + (BLK_PELS<<1) };
    uint16_t *count[COLOR_CHANNELS] = { counter, counter + BLK_PELS, counter + (BLK_PELS<<1) };
    EbByte pred[COLOR_CHANNELS] = { predictor, predictor + BLK_PELS, predictor + (BLK_PELS<<1) };
    int index_center;
    uint32_t blk_row, blk_col;
    int stride_pred[COLOR_CHANNELS] = {BW, BW_CH, BW_CH};
    uint16_t blk_width_ch = BW_CH;
    uint16_t blk_height_ch = BH_CH;
    int blk_y_offset = 0, blk_y_src_offset = 0, blk_ch_offset = 0, blk_ch_src_offset = 0;
    EbByte src_frame_index[COLOR_CHANNELS], src_altref_index[COLOR_CHANNELS];
    int i, j, k;

    PictureParentControlSet *picture_control_set_ptr_central;
    EbPictureBufferDesc *input_picture_ptr_central;

    // index of the center frame
    index_center = altref_nframes/2;

    picture_control_set_ptr_central = list_picture_control_set_ptr[index_center];
    input_picture_ptr_central = list_input_picture_ptr[index_center];

    EbAsm asm_type = picture_control_set_ptr_central->sequence_control_set_ptr->encode_context_ptr->asm_type;

    uint32_t blk_cols = (uint32_t)(input_picture_ptr_central->width + BW - 1) / BW; // I think only the part of the picture
    uint32_t blk_rows = (uint32_t)(input_picture_ptr_central->height + BH - 1) / BH; // that fits to the 32x32 blocks are actually filtered

    int stride[COLOR_CHANNELS] = { input_picture_ptr_central->stride_y, input_picture_ptr_central->stride_cb, input_picture_ptr_central->stride_cr };

#if !VANILA_ME
    // initialize chroma interpolated buffers and auxiliary buffers
    uint8_t *pos_b_buffer_ch[2];
    uint8_t *pos_h_buffer_ch[2];
    uint8_t *pos_j_buffer_ch[2];
    uint8_t *one_d_intermediate_results_buf_ch[2];

    pos_b_buffer_ch[0] = (uint8_t *)malloc(sizeof(uint8_t) * MAX_SEARCH_AREA_WIDTH_CH * MAX_SEARCH_AREA_HEIGHT_CH);
    pos_h_buffer_ch[0] = (uint8_t *)malloc(sizeof(uint8_t) * MAX_SEARCH_AREA_WIDTH_CH * MAX_SEARCH_AREA_HEIGHT_CH);
    pos_j_buffer_ch[0] = (uint8_t *)malloc(sizeof(uint8_t) * MAX_SEARCH_AREA_WIDTH_CH * MAX_SEARCH_AREA_HEIGHT_CH);
    pos_b_buffer_ch[1] = (uint8_t *)malloc(sizeof(uint8_t) * MAX_SEARCH_AREA_WIDTH_CH * MAX_SEARCH_AREA_HEIGHT_CH);
    pos_h_buffer_ch[1] = (uint8_t *)malloc(sizeof(uint8_t) * MAX_SEARCH_AREA_WIDTH_CH * MAX_SEARCH_AREA_HEIGHT_CH);
    pos_j_buffer_ch[1] = (uint8_t *)malloc(sizeof(uint8_t) * MAX_SEARCH_AREA_WIDTH_CH * MAX_SEARCH_AREA_HEIGHT_CH);

    one_d_intermediate_results_buf_ch[0] = (uint8_t *)malloc(sizeof(uint8_t)*(BLOCK_SIZE_64>>1)*(BLOCK_SIZE_64>>1));
    one_d_intermediate_results_buf_ch[1] = (uint8_t *)malloc(sizeof(uint8_t)*(BLOCK_SIZE_64>>1)*(BLOCK_SIZE_64>>1));
#endif

#if !VANILA_ME
    MotionEstimationContext_t *me_context_ptr;
    // Call ME context initializer
    me_context_ptr = (MotionEstimationContext_t*)malloc(sizeof(MotionEstimationContext_t));

#if MEMORY_FOOTPRINT_OPT_ME_MV
    me_context_ctor(&(me_context_ptr->me_context_ptr),
#if REDUCE_ME_SEARCH_AREA
                    input_picture_ptr_central->width,
                    input_picture_ptr_central->height,
#endif
                    1, // nsq
                    0); // mrp_mode
#else
    me_context_ctor(&(me_context_ptr->me_context_ptr));
#endif

    MeContext *context_ptr = me_context_ptr->me_context_ptr;
#endif

#if DEBUG_TEMPORAL_FILTER
    save_YUV_to_file("central_frame.yuv", input_picture_ptr_central->buffer_y, input_picture_ptr_central->buffer_cb, input_picture_ptr_central->buffer_cr,
                     input_picture_ptr_central->width, input_picture_ptr_central->height,
                     input_picture_ptr_central->stride_y, input_picture_ptr_central->stride_cb, input_picture_ptr_central->stride_cr,
                     input_picture_ptr_central->origin_y, input_picture_ptr_central->origin_x);
#endif

#if 0
    // pad chroma samples to be used by the motion compensation
    for (frame_index = 0; frame_index < altref_nframes; frame_index++) {
        pad_chroma_samples(list_input_picture_ptr[frame_index]);
    }
#endif

    // for each block
    for (blk_row = 0; blk_row < blk_rows; blk_row++) {

        for (blk_col = 0; blk_col < blk_cols; blk_col++) {

            // reset accumulator and count
            memset(accumulator, 0, BLK_PELS * COLOR_CHANNELS * sizeof(accumulator[0]));
            memset(counter, 0, BLK_PELS * COLOR_CHANNELS * sizeof(counter[0]));

            int blk_fw[N_16X16_BLOCKS];
            int use_16x16_subblocks = 0;
            int me_16x16_subblock_sad[N_16X16_BLOCKS];
            int me_32x32_total_sad = 0;

            populate_list_with_value(blk_fw, 16, INIT_WEIGHT);

            // for every frame to filter
            for (frame_index = 0; frame_index < altref_nframes; frame_index++) {

                // first position of the frame buffer according to frame index
                src_frame_index[C_Y] = list_input_picture_ptr[frame_index]->buffer_y +
                        list_input_picture_ptr[frame_index]->origin_y*list_input_picture_ptr[frame_index]->stride_y +
                        list_input_picture_ptr[frame_index]->origin_x;

                src_frame_index[C_U] = list_input_picture_ptr[frame_index]->buffer_cb +
                         (list_input_picture_ptr[frame_index]->origin_y>>1)*list_input_picture_ptr[frame_index]->stride_cb +
                         (list_input_picture_ptr[frame_index]->origin_x>>1);

                src_frame_index[C_V] = list_input_picture_ptr[frame_index]->buffer_cr +
                         (list_input_picture_ptr[frame_index]->origin_y>>1)*list_input_picture_ptr[frame_index]->stride_cr +
                         (list_input_picture_ptr[frame_index]->origin_x>>1);

                // first position of the frame buffer according to the index center
                src_altref_index[C_Y] = list_input_picture_ptr[index_center]->buffer_y +
                                      list_input_picture_ptr[index_center]->origin_y*list_input_picture_ptr[index_center]->stride_y +
                                      list_input_picture_ptr[index_center]->origin_x;

                src_altref_index[C_U] = list_input_picture_ptr[index_center]->buffer_cb +
                                      (list_input_picture_ptr[index_center]->origin_y>>1)*list_input_picture_ptr[index_center]->stride_cb +
                                      (list_input_picture_ptr[index_center]->origin_x>>1);

                src_altref_index[C_V] = list_input_picture_ptr[index_center]->buffer_cr +
                                      (list_input_picture_ptr[index_center]->origin_y>>1)*list_input_picture_ptr[index_center]->stride_cr +
                                      (list_input_picture_ptr[index_center]->origin_x>>1);

                src_altref_index[C_Y] = src_altref_index[C_Y] + blk_y_src_offset;
                src_altref_index[C_U] = src_altref_index[C_U] + blk_ch_src_offset;
                src_altref_index[C_V] = src_altref_index[C_V] + blk_ch_src_offset;

                // ------------
                // Step 1: motion compensation
                // ------------

                // if frame to process is the center frame
                if (frame_index == index_center) {

                    // skip MC (central frame)

                    populate_list_with_value(blk_fw, N_16X16_BLOCKS, 2);
                    use_16x16_subblocks = 0;

                    copy_pixels(pred[C_Y], BW, src_frame_index[C_Y] + blk_y_src_offset, stride[C_Y], BW, BH);
                    copy_pixels(pred[C_U], BW_CH, src_frame_index[C_U] + blk_ch_src_offset, stride[C_U], BW_CH, BH_CH);
                    copy_pixels(pred[C_V], BW_CH, src_frame_index[C_V] + blk_ch_src_offset, stride[C_V], BW_CH, BH_CH);

                }else{

#if VANILA_ME
                    int16_t  x_Level2_search_center;           // output parameter, Level2 xMV at (searchRegionNumberInWidth, searchRegionNumberInHeight)
                    int16_t  y_Level2_search_center;
                    x_Level2_search_center = 4;
                    y_Level2_search_center = 4;
                    int x_top_left_search_region = -4;
                    int y_top_left_search_region = -4;
                    int search_region_index_y = blk_y_src_offset + x_top_left_search_region + y_top_left_search_region * stride[C_Y];
                    int search_region_index_ch = blk_ch_src_offset + (x_top_left_search_region>>1) + (y_top_left_search_region>>1) * stride[C_U];
                    uint64_t level2_best_sad;                  // output parameter, Level2 SAD at (searchRegionNumberInWidth, searchRegionNumberInHeight)

                    // Move to the top left of the search region and search within a 8x8 region
                    sad_loop_kernel(
                            src_altref_index[C_Y],
                            (uint32_t)stride[C_Y],
                            src_frame_index[C_Y] + search_region_index_y,
                            (uint32_t)stride[C_Y],
                            BW, BH,
                            /* results */
                            &level2_best_sad,
                            &x_Level2_search_center,
                            &y_Level2_search_center,
                            /* range */
                            (uint32_t)stride[C_Y],
                            8, // search area set to 8x8 just for testing
                            8
                    );

                    printf("block #%d, best SAD = %d, x search center = %d, y search center = %d\n", blk_row*blk_cols + blk_col, (int)level2_best_sad, x_Level2_search_center, y_Level2_search_center);

                    int shift_y = search_region_index_y + y_Level2_search_center*stride[C_Y] + x_Level2_search_center;
                    int shift_ch = search_region_index_ch + (y_Level2_search_center>>1)*stride[C_U] + (x_Level2_search_center>>1);

                    // get predicted block
                    copy_pixels_and_remove_stride(pred[C_Y], src_frame_index[C_Y] + shift_y, BW, BH, stride[C_Y]);
                    copy_pixels_and_remove_stride(pred[C_U], src_frame_index[C_U] + shift_ch, BW_CH, BH_CH, stride[C_U]);
                    copy_pixels_and_remove_stride(pred[C_V], src_frame_index[C_V] + shift_ch, BW_CH, BH_CH, stride[C_V]);

#else

                    // Block-based MC using open-loop HME + refinement

                    // Initialize ME context
                    create_ME_context_and_picture_control(me_context_ptr,
                                                          list_picture_control_set_ptr[frame_index],
                                                          list_picture_control_set_ptr[index_center],
                                                          input_picture_ptr_central,
                                                          blk_row,
                                                          blk_col);

                    // Perform ME - context_ptr will store the outputs (MVs, buffers, etc)
                    motion_estimate_lcu( picture_control_set_ptr_central, // source picture control set -> references come from here
                                        (uint32_t)blk_row*blk_cols + blk_col,
                                        (uint32_t)blk_col*BW, // x block
                                        (uint32_t)blk_row*BH, // y block
                                        context_ptr,
                                        list_input_picture_ptr[index_center]); // source picture

                    // Retrieve distortion (SAD) on 32x32 and 16x16 sub-blocks
                    get_ME_distortion(context_ptr, &me_32x32_total_sad, me_16x16_subblock_sad);

                    // Get sub-block filter weights depending on the SAD
                    get_blk_fw_using_dist(&me_32x32_total_sad, me_16x16_subblock_sad, &use_16x16_subblocks, blk_fw);

                    // Perform MC using the information acquired using the ME step
                    uni_motion_compensation(context_ptr,
                                        list_input_picture_ptr[frame_index],
                                        pred,
                                        (uint32_t)blk_col*BW,
                                        (uint32_t)blk_row*BH,
                                        pos_b_buffer_ch,
                                        pos_h_buffer_ch,
                                        pos_j_buffer_ch,
                                        one_d_intermediate_results_buf_ch,
                                        use_16x16_subblocks,
                                        asm_type);

#if MC_CHROMA==0
                    copy_pixels(pred[C_U], BW_CH, src_frame_index[C_U] + blk_ch_src_offset, stride[C_U], BW_CH, BH_CH);
                    copy_pixels(pred[C_V], BW_CH, src_frame_index[C_V] + blk_ch_src_offset, stride[C_V], BW_CH, BH_CH);
#endif

#endif

                }

#if DEBUG_TEMPORAL_FILTER
                char filename3[30] = "input_block_";
                char frame_index_str[4];
                char block_number1[4];
                snprintf(block_number1, 4, "%d", blk_row*blk_cols + blk_col);
                snprintf(frame_index_str, 4, "%d", frame_index);
                strcat(filename3, block_number1);
                strcat(filename3, "_");
                strcat(filename3, frame_index_str);
                strcat(filename3, ".yuv");
                save_YUV_to_file(filename3, src_frame_index[C_Y] + blk_y_src_offset, src_frame_index[C_U] + blk_ch_src_offset, src_frame_index[C_V] + blk_ch_src_offset,
                                 BW, BH,
                                 stride[C_Y], stride[C_U], stride[C_V],
                                 0, 0);

                char filename1[30] = "pred_block_";
                strcat(filename1, block_number1);
                strcat(filename1, "_");
                strcat(filename1, frame_index_str);
                strcat(filename1, ".yuv");
                save_YUV_to_file(filename1, pred[C_Y], pred[C_U], pred[C_V],
                                    BW, BH,
                                    BW, blk_width_ch, blk_height_ch,
                                    0, 0);

                char filename5[30] = "central_block_";
                strcat(filename5, block_number1);
                strcat(filename5, "_");
                strcat(filename5, frame_index_str);
                strcat(filename5, ".yuv");
                save_YUV_to_file(filename5, src_altref_index[C_Y], src_altref_index[C_U], src_altref_index[C_V],
                                 BW, BH,
                                 stride[C_Y], stride[C_U], stride[C_V],
                                 0, 0);

#if 0
                printf("PRED\n");
                printf("Y:\n");
                print_block_uint8(pred[C_Y], 32, 32, 32);
                printf("U:\n");
                print_block_uint8(pred[C_U], 16, 16, 16);
                printf("V:\n");
                print_block_uint8(pred[C_V], 16, 16, 16);
#endif
#endif

                // ------------
                // Step 2: temporal filtering using the motion compensated blocks
                // ------------

                // if frame to process is the center frame
                if (frame_index == index_center) {

                    apply_filtering_central(pred,
                                            accum,
                                            count,
                                            BH,
                                            BW);

                }else{

                    // Apply the temporal filtering strategy
                    apply_filtering(src_altref_index,
                                    pred,
                                    accum,
                                    count,
                                    stride,
                                    stride_pred,
                                    BW,
                                    BH,
                                    1,
                                    1,
                                    altref_strength,
                                    blk_fw, // depends on the error of the MC step
                                    use_16x16_subblocks); // depends on the error of the MC step

                }
            }

            // Normalize filter output to produce AltRef frame
            // Process luma
            int byte = blk_y_offset;
            for (i = 0, k = 0; i < BH; i++) {
                for (j = 0; j < BW; j++, k++) {
                    //alt_ref_buffer_y[byte] = (uint8_t)OD_DIVU(accum[C_Y][k] + (count[C_Y][k] >> 1), count[C_Y][k]);
                    alt_ref_buffer[C_Y][byte] = (uint8_t)((accum[C_Y][k] + (count[C_Y][k] >> 1))/count[C_Y][k]);
                    // move to next pixel
                    byte++;
                }
                byte += stride[C_Y] - BW;
            }
            // Process chroma
            byte = blk_ch_offset;
            for (i = 0, k = 0; i < blk_height_ch; i++) {
                for (j = 0; j < blk_width_ch; j++, k++) {
                    // U
                    alt_ref_buffer[C_U][byte] = (uint8_t)((accum[C_U][k] + (count[C_U][k] >> 1))/count[C_U][k]);
                    // V
                    alt_ref_buffer[C_V][byte] = (uint8_t)((accum[C_V][k] + (count[C_V][k] >> 1))/count[C_V][k]);
                    // move to next pixel
                    byte++;
                }
                byte += stride[C_U] - (BW_CH);
            }

#if DEBUG_TEMPORAL_FILTER
            char filename[30] = "filtered_block_";
            char block_number[4];
            snprintf(block_number, 4, "%d", blk_row*blk_cols + blk_col);
            strcat(filename, block_number);
            strcat(filename, ".yuv");
            save_YUV_to_file(filename, alt_ref_buffer[C_Y]+blk_y_offset, alt_ref_buffer[C_U]+blk_ch_offset, alt_ref_buffer[C_V]+blk_ch_offset,
                             BW, BH,
                             input_picture_ptr_central->stride_y, input_picture_ptr_central->stride_cb, input_picture_ptr_central->stride_cr,
                             0, 0);

#endif

            // this is how libaom was implementing the blocks indexes (I did differently in the memcpy above)
            blk_y_offset += BW;
            blk_y_src_offset += BW;
            blk_ch_offset += blk_width_ch;
            blk_ch_src_offset += blk_width_ch;
        }
        blk_y_offset += BH * stride[C_Y] - BW * blk_cols;
        blk_y_src_offset += BH * stride[C_Y] - BW * blk_cols;
        blk_ch_offset += blk_height_ch * stride[C_U] - blk_width_ch * blk_cols;
        blk_ch_src_offset += blk_height_ch * stride[C_U] - blk_width_ch * blk_cols;
    }

#if 1
    {
        char filename[30] = "filtered_frame_svtav1_";
        char frame_index_str[4];
        snprintf(frame_index_str, 4, "%d", (int)picture_control_set_ptr_central->picture_number);
        strcat(filename, frame_index_str);
        strcat(filename, ".yuv");
        save_YUV_to_file(filename, alt_ref_buffer[C_Y], alt_ref_buffer[C_U], alt_ref_buffer[C_V],
                         input_picture_ptr_central->width, input_picture_ptr_central->height,
                         input_picture_ptr_central->stride_y, input_picture_ptr_central->stride_cb, input_picture_ptr_central->stride_cr,
                         0, 0);
    }
#endif

#if !VANILA_ME
    free(me_context_ptr);
    free(pos_b_buffer_ch[0]);
    free(pos_h_buffer_ch[0]);
    free(pos_j_buffer_ch[0]);
    free(pos_b_buffer_ch[1]);
    free(pos_h_buffer_ch[1]);
    free(pos_j_buffer_ch[1]);
    free(one_d_intermediate_results_buf_ch[0]);
    free(one_d_intermediate_results_buf_ch[1]);
#endif

    return EB_ErrorNone;
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
//static double highbd_estimate_noise(const uint8_t *src8, int width, int height,
//                                    int stride, int bd, int edge_thresh) {
//    uint16_t *src = CONVERT_TO_SHORTPTR(src8);
//    int64_t sum = 0;
//    int64_t num = 0;
//    for (int i = 1; i < height - 1; ++i) {
//        for (int j = 1; j < width - 1; ++j) {
//            const int k = i * stride + j;
//            // Sobel gradients
//            const int Gx = (src[k - stride - 1] - src[k - stride + 1]) +
//                           (src[k + stride - 1] - src[k + stride + 1]) +
//                           2 * (src[k - 1] - src[k + 1]);
//            const int Gy = (src[k - stride - 1] - src[k + stride - 1]) +
//                           (src[k - stride + 1] - src[k + stride + 1]) +
//                           2 * (src[k - stride] - src[k + stride]);
//            const int Ga = ROUND_POWER_OF_TWO(abs(Gx) + abs(Gy), bd - 8);
//            if (Ga < edge_thresh) {  // Smooth pixels
//                // Find Laplacian
//                const int v =
//                        4 * src[k] -
//                        2 * (src[k - 1] + src[k + 1] + src[k - stride] + src[k + stride]) +
//                        (src[k - stride - 1] + src[k - stride + 1] + src[k + stride - 1] +
//                         src[k + stride + 1]);
//                sum += ROUND_POWER_OF_TWO(abs(v), bd - 8);
//                ++num;
//            }
//        }
//    }
//    // If very few smooth pels, return -1 since the estimate is unreliable
//    if (num < 16) return -1.0;
//
//    const double sigma = (double)sum / (6 * num) * SQRT_PI_BY_2;
//    return sigma;
//}

// Apply buffer limits and context specific adjustments to arnr filter.
static void adjust_filter_params(EbPictureBufferDesc *input_picture_ptr,
                        uint8_t *altref_strength,
                        uint8_t *altref_nframes) {

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

    // Adjust the strength of the temporal filtering
    // based on the amount of noise present in the frame
    // adjustment in the integer range [-2, 1]
    // if noiselevel < 0, it means that the estimation was
    // unsuccessful and therefore keep the strength as it was set
    if (noiselevel > 0) {
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

#if DEBUG_TEMPORAL_FILTER
    printf("[DEBUG] noise level: %g, strength = %d, adj_strength = %d\n", noiselevel, strength, adj_strength);
#endif

    // TODO: does it make sense to use negative strength after it has been adjusted?
    strength = adj_strength;

    // TODO: libaom applies some more refinements to the number of frames to filter and strength
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

int replace_src_pic_buffers(PictureParentControlSet *picture_control_set_ptr_central, uint8_t **alt_ref_buffer){

#if DEBUG_TEMPORAL_FILTER
    save_YUV_to_file("src_enhanced_picture.yuv",
                     picture_control_set_ptr_central->enhanced_picture_ptr->buffer_y,
                     picture_control_set_ptr_central->enhanced_picture_ptr->buffer_cb,
                     picture_control_set_ptr_central->enhanced_picture_ptr->buffer_cr,
                     picture_control_set_ptr_central->enhanced_picture_ptr->width,
                     picture_control_set_ptr_central->enhanced_picture_ptr->height,
                     picture_control_set_ptr_central->enhanced_picture_ptr->stride_y,
                     picture_control_set_ptr_central->enhanced_picture_ptr->stride_cb,
                     picture_control_set_ptr_central->enhanced_picture_ptr->stride_cr,
                     0,
                     0);
#endif

    // Y
    copy_pixels_with_origin(picture_control_set_ptr_central->enhanced_picture_ptr->buffer_y,
                         picture_control_set_ptr_central->enhanced_picture_ptr->stride_y,
                         picture_control_set_ptr_central->enhanced_picture_ptr->origin_y,
                         picture_control_set_ptr_central->enhanced_picture_ptr->origin_x,
                         alt_ref_buffer[C_Y],
                         picture_control_set_ptr_central->enhanced_picture_ptr->stride_y,
                         0,
                         0,
                         picture_control_set_ptr_central->enhanced_picture_ptr->width,
                         picture_control_set_ptr_central->enhanced_picture_ptr->height);

    // U
    copy_pixels_with_origin(picture_control_set_ptr_central->enhanced_picture_ptr->buffer_cb,
                         picture_control_set_ptr_central->enhanced_picture_ptr->stride_cb,
                         picture_control_set_ptr_central->enhanced_picture_ptr->origin_y >> 1,
                         picture_control_set_ptr_central->enhanced_picture_ptr->origin_x >> 1,
                         alt_ref_buffer[C_U],
                         picture_control_set_ptr_central->enhanced_picture_ptr->stride_cb,
                         0,
                         0,
                         picture_control_set_ptr_central->enhanced_picture_ptr->width >> 1,
                         picture_control_set_ptr_central->enhanced_picture_ptr->height >> 1);

    // V
    copy_pixels_with_origin(picture_control_set_ptr_central->enhanced_picture_ptr->buffer_cr,
                         picture_control_set_ptr_central->enhanced_picture_ptr->stride_cr,
                         picture_control_set_ptr_central->enhanced_picture_ptr->origin_y >> 1,
                         picture_control_set_ptr_central->enhanced_picture_ptr->origin_x >> 1,
                         alt_ref_buffer[C_V],
                         picture_control_set_ptr_central->enhanced_picture_ptr->stride_cr,
                         0,
                         0,
                         picture_control_set_ptr_central->enhanced_picture_ptr->width >> 1,
                         picture_control_set_ptr_central->enhanced_picture_ptr->height >> 1);


    // Replace enhanced picture buffer

#if DEBUG_TEMPORAL_FILTER
    save_YUV_to_file("modified_enhanced_picture.yuv",
                     picture_control_set_ptr_central->enhanced_picture_ptr->buffer_y,
                     picture_control_set_ptr_central->enhanced_picture_ptr->buffer_cb,
                     picture_control_set_ptr_central->enhanced_picture_ptr->buffer_cr,
                     picture_control_set_ptr_central->enhanced_picture_ptr->width,
                     picture_control_set_ptr_central->enhanced_picture_ptr->height,
                     picture_control_set_ptr_central->enhanced_picture_ptr->stride_y,
                     picture_control_set_ptr_central->enhanced_picture_ptr->stride_cb,
                     picture_control_set_ptr_central->enhanced_picture_ptr->stride_cr,
                     0,
                     0);
#endif

    // get padded alt-ref picture

    // reference structures (padded pictures + downsampled versions)
    EbPaReferenceObject *src_object = (EbPaReferenceObject*)picture_control_set_ptr_central->pa_reference_picture_wrapper_ptr->object_ptr;
    EbPictureBufferDesc *padded_pic_ptr = src_object->input_padded_picture_ptr;
    EbPictureBufferDesc *quarter_pic_ptr = src_object->quarter_decimated_picture_ptr;
    EbPictureBufferDesc *sixteenth_pic_ptr = src_object->sixteenth_decimated_picture_ptr;

#if DEBUG_TEMPORAL_FILTER
    save_Y_to_file("input_padded_picture.yuv",
                   padded_pic_ptr->buffer_y,
                   picture_control_set_ptr_central->enhanced_picture_ptr->stride_y,
                   picture_control_set_ptr_central->enhanced_picture_ptr->height,
                   picture_control_set_ptr_central->enhanced_picture_ptr->stride_y,
                   0, 0);
#endif

    copy_pixels_with_origin(padded_pic_ptr->buffer_y,
                         padded_pic_ptr->stride_y,
                         padded_pic_ptr->origin_y,
                         padded_pic_ptr->origin_x,
                         alt_ref_buffer[C_Y],
                         picture_control_set_ptr_central->enhanced_picture_ptr->stride_y,
                         0,
                         0,
                         picture_control_set_ptr_central->enhanced_picture_ptr->width,
                         picture_control_set_ptr_central->enhanced_picture_ptr->height);

    generate_padding(
            &(padded_pic_ptr->buffer_y[0]),
            padded_pic_ptr->stride_y,
            padded_pic_ptr->width,
            padded_pic_ptr->height,
            padded_pic_ptr->origin_x,
            padded_pic_ptr->origin_y);

    DecimateInputPicture(picture_control_set_ptr_central,
                         padded_pic_ptr,
                         quarter_pic_ptr,
                         sixteenth_pic_ptr);

#if DEBUG_TEMPORAL_FILTER
    save_Y_to_file("padded_picture.yuv",
            padded_pic_ptr->buffer_y,
            picture_control_set_ptr_central->enhanced_picture_ptr->stride_y,
            picture_control_set_ptr_central->enhanced_picture_ptr->height,
            picture_control_set_ptr_central->enhanced_picture_ptr->stride_y,
            0, 0);

    save_Y_to_file("quarter_picture.yuv",
                   quarter_pic_ptr->buffer_y,
                   picture_control_set_ptr_central->enhanced_picture_ptr->stride_y >> 1,
                   picture_control_set_ptr_central->enhanced_picture_ptr->height >> 1,
                   picture_control_set_ptr_central->enhanced_picture_ptr->stride_y >> 1,
                   0, 0);

    save_Y_to_file("sixteenth_picture.yuv",
                   sixteenth_pic_ptr->buffer_y,
                   picture_control_set_ptr_central->enhanced_picture_ptr->stride_y >> 2,
                   picture_control_set_ptr_central->enhanced_picture_ptr->height >> 2,
                   picture_control_set_ptr_central->enhanced_picture_ptr->stride_y >> 2,
                   0, 0);
#endif

    return 0;

}

int read_YUV_frame_from_file(uint8_t **alt_ref_buffer, int picture_number, int width, int height, int stride){

    char filename[30] = "filtered_frame_libaom_";
    char pic_num_str[4];
    snprintf(pic_num_str, 4, "%d", picture_number);
    strcat(filename, pic_num_str);
    strcat(filename, ".yuv");

    FILE *fid = NULL;

    // save current source picture to a YUV file
    if ((fid = fopen(filename, "rb")) == NULL) {
        printf("Unable to open file %s for reading.\n", filename);
    }else {

        uint8_t* pic_point = alt_ref_buffer[C_Y];
        for (int h = 0; h < height; h++) {
            fread(pic_point, sizeof(uint8_t), (size_t)width, fid);
            pic_point = pic_point + stride;
        }
        pic_point = alt_ref_buffer[C_U];
        for (int h = 0; h < height >> 1; h++) {
            fread(pic_point, sizeof(uint8_t), (size_t)width >> 1, fid);
            pic_point = pic_point + (stride>>1);
        }
        pic_point = alt_ref_buffer[C_V];
        for (int h = 0; h < height >> 1; h++) {
            fread(pic_point, sizeof(uint8_t), (size_t)width >> 1, fid);
            pic_point = pic_point + (stride>>1);
        }
        fclose(fid);
    }

    return 0;
}

EbErrorType init_temporal_filtering(PictureParentControlSet **list_picture_control_set_ptr) {

    uint8_t altref_strength, altref_nframes, index_center;
    EbPictureBufferDesc *input_picture_ptr;
    PictureParentControlSet *picture_control_set_ptr_central = list_picture_control_set_ptr[0]; // picture control set of the first frame
    uint8_t *alt_ref_buffer[COLOR_CHANNELS];

    // user-defined encoder parameters related to alt-refs
    altref_strength = picture_control_set_ptr_central->sequence_control_set_ptr->static_config.altref_strength;
    altref_nframes = picture_control_set_ptr_central->sequence_control_set_ptr->static_config.altref_nframes;

    // index of the central source frame
    index_center = (uint8_t)(altref_nframes/2);
    picture_control_set_ptr_central = list_picture_control_set_ptr[index_center];

    // source central frame picture buffer
    input_picture_ptr = picture_control_set_ptr_central->enhanced_picture_ptr;

    // adjust filter parameter based on the estimated noise of the picture
    adjust_filter_params(input_picture_ptr, &altref_strength, &altref_nframes);

    // populate source frames picture buffer list
    EbPictureBufferDesc *list_input_picture_ptr[ALTREF_MAX_NFRAMES] = { NULL };
    for(int i=0; i<altref_nframes; i++){
        list_input_picture_ptr[i] = list_picture_control_set_ptr[i]->enhanced_picture_ptr;
    }

    // allocate memory for the alt-ref buffer - to be replaced by some other final structure
    for(int c=0; c<COLOR_CHANNELS; c++){
        alt_ref_buffer[c] = (EbByte) malloc(input_picture_ptr->luma_size * sizeof(uint8_t));
    }

#if LIBAOM_FILTERING==0
    EbErrorType ret;
    ret = produce_temporally_filtered_pic(list_picture_control_set_ptr, list_input_picture_ptr, altref_strength, altref_nframes, alt_ref_buffer);
#else
    int ret = read_YUV_frame_from_file(alt_ref_buffer,
                            (int)picture_control_set_ptr_central->picture_number,
                            list_input_picture_ptr[index_center]->width,
                            list_input_picture_ptr[index_center]->height,
                            list_input_picture_ptr[index_center]->stride_y);
#endif

    // TODO: for test purposes only - replacing the src buffers with the filtered frame (milestone 0)
    replace_src_pic_buffers(picture_control_set_ptr_central, alt_ref_buffer);

    for(int c=0; c<COLOR_CHANNELS; c++){
        free(alt_ref_buffer[c]);
    }

    return ret;
}

#endif