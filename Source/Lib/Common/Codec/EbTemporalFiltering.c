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

#define OD_DIVU_DMAX (1024)
#define OD_DIVU_SMALL(_x, _d)                                     \
  ((uint32_t)((OD_DIVU_SMALL_CONSTS[(_d)-1][0] * (uint64_t)(_x) + \
               OD_DIVU_SMALL_CONSTS[(_d)-1][1]) >>                \
              32) >>                                              \
   (OD_ILOG_NZ(_d) - 1))

#define OD_DIVU(_x, _d) \
  (((_d) < OD_DIVU_DMAX) ? (OD_DIVU_SMALL((_x), (_d))) : ((_x) / (_d)))


static unsigned int index_mult[14] = {
        0, 0, 0, 0, 49152, 39322, 32768, 28087, 24576, 21846, 19661, 17874, 0, 15124
};

void copy_block_and_remove_stride(EbByte dst, EbByte src, int width, int height, int stride ){

    int h;
    EbByte src_cpy = src, dst_cpy = dst;

    for (h=0; h<height; h++){
        memcpy(dst_cpy, src_cpy, width * sizeof(uint8_t));
        dst_cpy += width;
        src_cpy += stride;
    }

}


void print_block_uint32(uint32_t *src, int width, int height, int stride){

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

void print_block_uint16(uint16_t *src, int width, int height, int stride){

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

// Get sub-block filter weights using
// blk_fw - block filter weight
// -------------------------
// |           |           |
// | blk_fw[0] | blk_fw[1] |
// |           |           |
// |-----------|-----------|
// |           |           |
// | blk_fw[2] | blk_fw[3] |
// |           |           |
// --------------------------
static INLINE int get_subblock_filter_weight(unsigned int i,
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
                    int use_32x32) {

    unsigned int i, j, k, m, idx, idy;
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

            int filter_weight = get_subblock_filter_weight(i, j, block_height, block_width, blk_fw, use_32x32);

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

            modifier = (int)mod_index(modifier, y_index, rounding, strength, filter_weight);

            if(modifier == 0)
                printf("modified = 0");

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

                u_mod = (int)mod_index(u_mod, cr_index, rounding, strength, filter_weight);
                v_mod = (int)mod_index(v_mod, cr_index, rounding, strength, filter_weight);

                count_u[m] += u_mod;
                accum_u[m] += u_mod * u_pixel_value;
                count_v[m] += v_mod;
                accum_v[m] += v_mod * v_pixel_value;

                ++m;
            }
        }
    }
}


// Only used in single plane case
//void apply_filtering_single_plane(uint8_t *orig,
//                    unsigned int stride,
//                    uint8_t *pred,
//                    unsigned int block_width,
//                    unsigned int block_height,
//                    int strength,
//                    const int *blk_fw,
//                    int use_32x32,
//                    unsigned int *accumulator,
//                    uint16_t *count) {
//
//    unsigned int i, j, k;
//    int modifier;
//    int byte = 0;
//    const int rounding = strength > 0 ? 1 << (strength - 1) : 0;
//
//    for (i = 0, k = 0; i < block_height; i++) {
//        for (j = 0; j < block_width; j++, k++) {
//            int pixel_value = *pred;
//            int filter_weight =
//                    get_filter_weight(i, j, block_height, block_width, blk_fw, use_32x32);
//
//            // non-local mean approach
//            int diff_sse[9] = { 0 };
//            int idx, idy, index = 0;
//
//            for (idy = -1; idy <= 1; ++idy) {
//                for (idx = -1; idx <= 1; ++idx) {
//                    int row = (int)i + idy;
//                    int col = (int)j + idx;
//
//                    if (row >= 0 && row < (int)block_height && col >= 0 &&
//                        col < (int)block_width) {
//                        int diff = orig[byte + idy * (int)stride + idx] -
//                                   pred[idy * (int)block_width + idx];
//                        diff_sse[index] = diff * diff;
//                        ++index;
//                    }
//                }
//            }
//
//            assert(index > 0);
//
//            modifier = 0;
//            for (idx = 0; idx < 9; ++idx) modifier += diff_sse[idx];
//
//            modifier *= 3;
//            modifier /= index;
//
//            ++pred;
//
//            modifier += rounding;
//            modifier >>= strength;
//
//            if (modifier > 16) modifier = 16;
//
//            modifier = 16 - modifier;
//            modifier *= filter_weight;
//
//            count[k] += modifier;
//            accumulator[k] += modifier * pixel_value;
//
//            byte++;
//        }
//
//        byte += stride - block_width;
//    }
//}

// buf_stride is the block size: 32 for Y and 16 for U and V
static void apply_filtering_central(EbByte *pred,
                                    uint32_t **accum,
                                    uint16_t **count,
                                    int blk_height,
                                    int blk_width) {

    EbByte pred_y = pred[0], pred_u = pred[1], pred_v = pred[2];
    uint32_t *accum_y = accum[0], *accum_u = accum[1], *accum_v = accum[2];
    uint16_t *count_y = count[0], *count_u = count[1], *count_v = count[2];

    unsigned int i, j, k;
    int blk_height_y = blk_height;
    int blk_width_y = blk_width;
    int blk_height_ch= blk_height>>1; // only works for 420 now!
    int blk_width_ch = blk_width>>1; // only works for 420 now!
    int blk_stride_y = blk_width;
    int blk_stride_ch = blk_width>>1; // only works for 420 now!

    int filter_weight = 2; // TODO: defines with these constants
    const int modifier = filter_weight * 16; // TODO: defines with these constants

    // Y
    k = 0;
    for (i = 0; i < blk_height_y; i++) {
        for (j = 0; j < blk_width_y; j++) {
            accum_y[k] += modifier * pred_y[i * blk_stride_y + j];
            count_y[k] += modifier;
            ++k;
        }
    }

    // UV
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

char* itoa2(int val, int base){

    static char buf[32] = {0};

    int i = 30;

    for(; val && i ; --i, val /= base)

        buf[i] = "0123456789abcdef"[val % base];

    return &buf[i+1];

}

// Produce the filtered alt-ref picture
static void produce_temporally_filtered_pic(EbPictureBufferDesc_t **list_input_picture_ptr,
                                            uint8_t altref_strength,
                                            uint8_t altref_nframes) {

    int frame_index;
    DECLARE_ALIGNED(16, uint32_t, accumulator[BLK_PELS * 3]);
    DECLARE_ALIGNED(16, uint16_t, counter[BLK_PELS * 3]);
    DECLARE_ALIGNED(32, uint8_t, predictor[BLK_PELS * 3]);
    uint32_t *accum[3] = { accumulator, accumulator + BLK_PELS, accumulator + (BLK_PELS<<1) };
    uint16_t *count[3] = { counter, counter + BLK_PELS, counter + (BLK_PELS<<1) };
    EbByte pred[3] = { predictor, predictor + BLK_PELS, predictor + (BLK_PELS<<1) };
    int index_center;
    int blk_row, blk_col;
    int stride_pred[3] = {BW, BW>>1, BW>>1};
    uint16_t blk_width_ch = BW >> 1; // 420 // TODO: implement for 420 now and extend it later
    uint16_t blk_height_ch = BW >> 1; // 420
    int blk_y_offset = 0, blk_y_src_offset = 0, blk_ch_offset = 0, blk_ch_src_offset = 0;
    EbByte src[3], src_altref_index[3];
    int i, j, k;
    EbPictureBufferDesc_t *input_picture_ptr_central;

    // index of the center frame
    index_center = altref_nframes/2;

    input_picture_ptr_central = list_input_picture_ptr[index_center];

    int blk_cols = (input_picture_ptr_central->width + BW - 1) / BW; // I think only the part of the picture
    int blk_rows = (input_picture_ptr_central->height + BH - 1) / BH; // that fits to the 32x32 blocks are actually filtered

    int stride[3] = { input_picture_ptr_central->stride_y, input_picture_ptr_central->strideCb, input_picture_ptr_central->strideCr };

    // allocate memory for the alt-ref buffer - to be replaced by some other final structure
    EbByte alt_ref_buffer_y = (EbByte) malloc(input_picture_ptr_central->lumaSize * sizeof(uint8_t));
    EbByte alt_ref_buffer_u = (EbByte) malloc(input_picture_ptr_central->chromaSize * sizeof(uint8_t));
    EbByte alt_ref_buffer_v = (EbByte) malloc(input_picture_ptr_central->chromaSize * sizeof(uint8_t));

#if DEBUG
    save_YUV_to_file("input_frame.yuv", input_picture_ptr_central->buffer_y, input_picture_ptr_central->bufferCb, input_picture_ptr_central->bufferCr,
                     input_picture_ptr_central->width, input_picture_ptr_central->height,
                     input_picture_ptr_central->stride_y, input_picture_ptr_central->strideCb, input_picture_ptr_central->strideCr,
                     input_picture_ptr_central->origin_y, input_picture_ptr_central->origin_x);
#endif

    // for each block
    for (blk_row = 0; blk_row < blk_rows; blk_row++) {

        for (blk_col = 0; blk_col < blk_cols; blk_col++) {

            // reset accumulator and count
            memset(accumulator, 0, BLK_PELS * 3 * sizeof(accumulator[0]));
            memset(counter, 0, BLK_PELS * 3 * sizeof(counter[0]));

            int blk_fw[4] = { 2, 2, 2, 2};
            int use_32x32 = 0;

            // for every frame to filter
            for (frame_index = 0; frame_index < altref_nframes; frame_index++) {

                // first position of the frame buffer according to frame index
                src[0] = list_input_picture_ptr[frame_index]->buffer_y +
                        list_input_picture_ptr[frame_index]->origin_y*list_input_picture_ptr[frame_index]->stride_y +
                        list_input_picture_ptr[frame_index]->origin_x;

                src[1] = list_input_picture_ptr[frame_index]->bufferCb +
                         (list_input_picture_ptr[frame_index]->origin_y>>1)*list_input_picture_ptr[frame_index]->strideCb +
                         (list_input_picture_ptr[frame_index]->origin_x>>1);

                src[2] = list_input_picture_ptr[frame_index]->bufferCr +
                         (list_input_picture_ptr[frame_index]->origin_y>>1)*list_input_picture_ptr[frame_index]->strideCr +
                         (list_input_picture_ptr[frame_index]->origin_x>>1);

                // first position of the frame buffer according to the index center
                src_altref_index[0] = list_input_picture_ptr[index_center]->buffer_y +
                                      list_input_picture_ptr[index_center]->origin_y*list_input_picture_ptr[index_center]->stride_y +
                                      list_input_picture_ptr[index_center]->origin_x;

                src_altref_index[1] = list_input_picture_ptr[index_center]->bufferCb +
                                      (list_input_picture_ptr[index_center]->origin_y>>1)*list_input_picture_ptr[index_center]->strideCb +
                                      (list_input_picture_ptr[index_center]->origin_x>>1);

                src_altref_index[2] = list_input_picture_ptr[index_center]->bufferCr +
                                      (list_input_picture_ptr[index_center]->origin_y>>1)*list_input_picture_ptr[index_center]->strideCr +
                                      (list_input_picture_ptr[index_center]->origin_x>>1);

#if DEBUG
                char filename2[30] = "input_frame_";
                char frame_index_str1[4];
                snprintf(frame_index_str1, 4, "%d", frame_index);
                strcat(filename2, frame_index_str1);
                strcat(filename2, ".yuv");
                save_YUV_to_file(filename2, src[0], src[1], src[2],
                                 input_picture_ptr_central->width, input_picture_ptr_central->height,
                                 input_picture_ptr_central->stride_y, input_picture_ptr_central->strideCb, input_picture_ptr_central->strideCr,
                                 0, 0);
#endif

//                src[0] = src[0] + blk_y_src_offset;
//                src[1] = src[1] + blk_ch_src_offset;
//                src[2] = src[2] + blk_ch_src_offset;

                src_altref_index[0] = src_altref_index[0] + blk_y_src_offset;
                src_altref_index[1] = src_altref_index[1] + blk_ch_src_offset;
                src_altref_index[2] = src_altref_index[2] + blk_ch_src_offset;

                // Move to the top left of the search region
                int x_top_left_search_region = -4;
                int y_top_left_search_region = -4;
                int search_region_index_y = blk_y_src_offset + x_top_left_search_region + y_top_left_search_region * stride[0];
                int search_region_index_ch = blk_ch_src_offset + (x_top_left_search_region>>1) + (y_top_left_search_region>>1) * stride[1];
                uint64_t level2_best_sad;                  // output parameter, Level2 SAD at (searchRegionNumberInWidth, searchRegionNumberInHeight)
                int16_t  x_Level2_search_center;           // output parameter, Level2 xMV at (searchRegionNumberInWidth, searchRegionNumberInHeight)
                int16_t  y_Level2_search_center;

                // ------------
                // Step 1: motion compensation TODO: motion compensation
                // ------------

                // if frame to process is the center frame
                if (frame_index == index_center) {

                    // skip MC (central frame)

                    blk_fw[0] = blk_fw[1] = blk_fw[2] = blk_fw[3] = 2;
                    use_32x32 = 1;

                }else{

                    sad_loop_kernel(
                            src_altref_index[0],
                            (uint32_t)stride[0],
                            src[0] + search_region_index_y,
                            (uint32_t)stride[0],
                            BW, BH,
                            /* results */
                            &level2_best_sad,
                            &x_Level2_search_center,
                            &y_Level2_search_center,
                            /* range */
                            (uint32_t)stride[0],
                            8, // search area set to 8x8 just for testing
                            8
                    );

                    printf("best SAD = %d, x search center = %d, y search center = %d\n", (int)level2_best_sad, x_Level2_search_center, y_Level2_search_center);

                }

                // Just for testing purposes until the MC block is available - the prediction is the original block
                int shift_y = search_region_index_y + y_Level2_search_center*stride[0] + x_Level2_search_center;
                int shift_ch = search_region_index_ch + (y_Level2_search_center>>1)*stride[1] + (x_Level2_search_center>>1);

                copy_block_and_remove_stride(pred[0], src[0] + shift_y, BW, BH, stride[0]);
                copy_block_and_remove_stride(pred[1], src[1] + shift_ch, BW>>1, BH>>1, stride[1]);
                copy_block_and_remove_stride(pred[2], src[2] + shift_ch, BW>>1, BH>>1, stride[2]);

#if DEBUG
                char filename1[30] = "pred_block_";
                char block_number1[4];
                char frame_index_str[4];
                snprintf(block_number1, 4, "%d", blk_row*blk_cols + blk_col);
                snprintf(frame_index_str, 4, "%d", frame_index);
                strcat(filename1, block_number1);
                strcat(filename1, "_");
                strcat(filename1, frame_index_str);
                strcat(filename1, ".yuv");
                save_YUV_to_file(filename1, pred[0], pred[1], pred[2],
                                    BW, BH,
                                    BW, blk_width_ch, blk_height_ch,
                                    0, 0);

//                printf("PRED\n");
//                printf("Y:\n");
//                print_block_uint8(pred[0], 32, 32, 32);
//                printf("U:\n");
//                print_block_uint8(pred[1], 16, 16, 16);
//                printf("V:\n");
//                print_block_uint8(pred[2], 16, 16, 16);
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
                                    use_32x32); // depends on the error of the MC step

                }
            }

#if DEBUG
//            printf("ACCUM\n");
//            printf("Y:\n");
//            print_block_uint32(accum[0], 32, 32, 32);
//            printf("U:\n");
//            print_block_uint32(accum[1], 16, 16, 16);
//            printf("V:\n");
//            print_block_uint32(accum[2], 16, 16, 16);
//
//            printf("COUNT\n");
//            printf("Y:\n");
//            print_block_uint16(count[0], 32, 32, 32);
//            printf("U:\n");
//            print_block_uint16(count[1], 16, 16, 16);
//            printf("V:\n");
//            print_block_uint16(count[2], 16, 16, 16);
#endif

            // Normalize filter output to produce AltRef frame
            // Process luma
            int byte = blk_y_offset;
            for (i = 0, k = 0; i < BH; i++) {
                for (j = 0; j < BW; j++, k++) {

//                    alt_ref_buffer_y[byte] = (uint8_t)OD_DIVU(accum[0][k] + (count[0][k] >> 1), count[0][k]);
                    alt_ref_buffer_y[byte] = (accum[0][k] + (count[0][k] >> 1))/count[0][k];
//                    printf("accum = %d, count = %d and dst = %d\n", accum[0][k], count[0][k], alt_ref_buffer_y[byte]);
                    // move to next pixel
                    byte++;
                }
                byte += stride[0] - BW;
            }
            // Process chroma
            byte = blk_ch_offset;
            for (i = 0, k = 0; i < blk_height_ch; i++) {
                for (j = 0; j < blk_width_ch; j++, k++) {
                    // U
//                    alt_ref_buffer_u[byte] = (uint8_t)OD_DIVU(accum[1][k] + (count[1][k] >> 1), count[1][k]);
                    alt_ref_buffer_u[byte] = (accum[1][k] + (count[1][k] >> 1))/count[1][k];
                    // V
//                    alt_ref_buffer_v[byte] = (uint8_t)OD_DIVU(accum[1][m] + (count[1][m] >> 1), count[1][m]);
                    alt_ref_buffer_v[byte] = (accum[2][k] + (count[2][k] >> 1))/count[2][k];
                    // move to next pixel
                    byte++;
                }
                byte += stride[1] - (BW>>1);
            }

#if DEBUG
            char filename[30] = "filtered_block_";
            char block_number[4];
            snprintf(block_number, 4, "%d", blk_row*blk_cols + blk_col);
            strcat(filename, block_number);
            strcat(filename, ".yuv");
            save_YUV_to_file(filename, alt_ref_buffer_y+blk_y_offset, alt_ref_buffer_u+blk_ch_offset, alt_ref_buffer_v+blk_ch_offset,
                             BW, BH,
                             input_picture_ptr_central->stride_y, input_picture_ptr_central->strideCb, input_picture_ptr_central->strideCr,
                             0, 0);

            if(blk_row*blk_cols + blk_col == 60){
                printf("block60");
            }
#endif

            // this is how libaom was implementing the blocks indexes (I did differently in the memcpy above)
            blk_y_offset += BW;
            blk_y_src_offset += BW;
            blk_ch_offset += blk_width_ch;
            blk_ch_src_offset += blk_width_ch;
        }
        blk_y_offset += BH * stride[0] - BW * blk_cols;
        blk_y_src_offset += BH * stride[0] - BW * blk_cols;
        blk_ch_offset += blk_height_ch * stride[1] - blk_width_ch * blk_cols;
        blk_ch_src_offset += blk_height_ch * stride[1] - blk_width_ch * blk_cols;
    }

#if DEBUG
    save_YUV_to_file("filtered_frame.yuv", alt_ref_buffer_y, alt_ref_buffer_u, alt_ref_buffer_v,
                     input_picture_ptr_central->width, input_picture_ptr_central->height,
                     input_picture_ptr_central->stride_y, input_picture_ptr_central->strideCb, input_picture_ptr_central->strideCr,
                     0, 0);
#endif

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

EbErrorType init_temporal_filtering(PictureParentControlSet_t **list_picture_control_set_ptr) {

    int start_frame;
    int frame;
    int frames_to_blur_backward;
    int frames_to_blur_forward;
    EbPictureBufferDesc_t *frames[MAX_FRAMES_TO_FILTER] = { NULL };
    EbBool enable_alt_refs;
    uint8_t altref_strength, altref_nframes;
    EbPictureBufferDesc_t *input_picture_ptr;
    // second position assuming a size of 3 frames for testing purposes
    PictureParentControlSet_t *picture_control_set_ptr_central = list_picture_control_set_ptr[1];

    // distance to key frame
    uint64_t distance_to_key = picture_control_set_ptr_central->picture_number - picture_control_set_ptr_central->decode_order;

    // source central frame picture buffer
    input_picture_ptr = picture_control_set_ptr_central->enhanced_picture_ptr;

    // populate source frames picture buffer list
    // TODO: using fixed 3 frames just for testing
    EbPictureBufferDesc_t *list_input_picture_ptr[3] = {NULL};
    for(int i=0; i<3; i++){
        list_input_picture_ptr[i] = list_picture_control_set_ptr[i]->enhanced_picture_ptr;
    }

    // user-defined encoder parameters related to alt-refs
    altref_strength = picture_control_set_ptr_central->sequence_control_set_ptr->static_config.altref_strength;
    altref_nframes = picture_control_set_ptr_central->sequence_control_set_ptr->static_config.altref_nframes;

    // adjust filter parameter based on the estimated noise of the picture
    adjust_filter_params(input_picture_ptr, distance_to_key, &altref_strength, &altref_nframes);

    //int which_arf = gf_group->arf_update_idx[gf_group->index];

    // Set the temporal filtering status for the corresponding OVERLAY frame
    if (altref_strength == 0 && altref_nframes == 1){
        // temporal filtering is off
        // is showable frame
#if DEBUG
        printf("It is not an alt-ref filtered frame\n");
#endif
    }
    else{
        // temporal filtering is on
        // is not showable frame
#if DEBUG
        printf("It is an alt-ref filtered frame\n");
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

    produce_temporally_filtered_pic(list_input_picture_ptr, altref_strength, altref_nframes);

    // Initialize errorperbit, sadperbit16 and sadperbit4.
    // rdmult = av1_compute_rd_mult_based_on_qindex(cpi, ARNR_FILT_QINDEX);
    // set_error_per_bit(&cpi->td.mb, rdmult);
    // av1_initialize_me_consts(cpi, &cpi->td.mb, ARNR_FILT_QINDEX);
    // av1_initialize_cost_tables(&cpi->common, &cpi->td.mb);

    // function that implements the temporal filtering in libaom
    //temporal_filter_iterate_c(cpi, frames, frames_to_blur, frames_to_blur_backward, strength, &sf);

    return EB_ErrorNone;
}