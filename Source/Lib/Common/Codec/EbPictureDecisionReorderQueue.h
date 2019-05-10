/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/

#ifndef EbPictureDecisionReorderQueue_h
#define EbPictureDecisionReorderQueue_h

#include "EbDefinitions.h"
#include "EbSystemResourceManager.h"

/************************************************
 * Packetization Reorder Queue Entry
 ************************************************/
typedef struct PictureDecisionReorderEntry 
{
    uint64_t                              picture_number;
    EbObjectWrapper                    *parent_pcs_wrapper_ptr;
#if ALT_REF_OVERLAY
    uint8_t                             overlay_arrived;
#endif
} PictureDecisionReorderEntry;


extern EbErrorType picture_decision_reorder_entry_ctor(
    PictureDecisionReorderEntry       **entry_dbl_ptr,
    uint32_t                              picture_number);

#endif //EbPictureDecisionReorderQueue_h
