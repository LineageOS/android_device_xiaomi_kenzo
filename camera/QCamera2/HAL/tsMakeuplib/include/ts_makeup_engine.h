/*
 * Copyright (C) 2014,2015 Thundersoft Corporation
 * All rights Reserved
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __TS_MAKEUP_ENGINI_H__
#define __TS_MAKEUP_ENGINI_H__
#include "ts_makeup_data.h"
#include "ts_makeup_image.h"


    /*
     * FUNCTION   : ts_makeup_get_supported_face_num
     *
     * DESCRIPTION: get supported face number
     *
     * RETURN    : The supported face number
     *
     */
    int ts_makeup_get_supported_face_num();


    /*
     * FUNCTION   : ts_makeup_skin_beauty
     *
     * DESCRIPTION: skin beauty method.
     *
     * PARAMETERS :
     *   @param[in] pInData : The TSMakeupData pointer.MUST not NULL.
     *   @param[out] pOutData : The TSMakeupData pointer.MUST not NULL.
     *   @param[in] pFaceRect : The face rect.MUST not NULL.
     *   @param[in] cleanLevel : Skin clean level, value range [0,100].
     *   @param[in] whiteLevel : Skin white level, value range [0,100].
     * RETURN    : TS_OK if success, otherwise failed.
     *
     */
    int ts_makeup_skin_beauty(TSMakeupData *pInData, TSMakeupData *pOutData, const TSRect *pFaceRect, int cleanLevel,int whiteLevel);
    /*
     * FUNCTION   : ts_makeup_skin_beautyEx
     *
     * DESCRIPTION: skin beauty method.
     *
     * PARAMETERS :
     *   @param[in] pInData : The TSMakeupDataEx pointer.MUST not NULL.
     *   @param[out] pOutData : The TSMakeupDataEx pointer.MUST not NULL.
     *   @param[in] pFaceRect : The face rect.MUST not NULL.
     *   @param[in] cleanLevel : Skin clean level, value range [0,100].
     *   @param[in] whiteLevel : Skin white level, value range [0,100].
     * RETURN    : TS_OK if success, otherwise failed.
     *
     */
    int ts_makeup_skin_beautyEx(TSMakeupDataEx *pInData, TSMakeupDataEx *pOutData, const TSRect *pFaceRect, int cleanLevel, int whiteLevel);
    /*
     * FUNCTION   : ts_makeup_finish
     *
     * DESCRIPTION: Finish makeup,call this method at last time.
     * This method MUST be called After ts_makeup_skin_clean and ts_makeup_skin_whiten
     *
     */
    void ts_makeup_finish();


    /*
     * FUNCTION   : ts_makeup_warp_face
     *
     * DESCRIPTION: do warp face.
     *
     * PARAMETERS :
     *   @param[in] pInData : The TSMakeupData pointer.MUST not NULL.
     *   @param[out] pOutData : The TSMakeupData pointer.MUST not NULL.
     *   @param[in] pLeftEye : The left eye rect pointer.MUST not NULL.
     *   @param[in] pRightEye : The right eye rect pointer.MUST not NULL.
     *   @param[in] pMouth : The mouth rect pointer.MUST not NULL.
     *   @param[in] bigEyeLevel : The big eye level, value range [0,100].
     *   @param[in] trimFaceLevel : The trim face level, value range [0,100].
     *
     * RETURN    : TS_OK if success, otherwise failed.
     *
     */
    int ts_makeup_warp_face(TSMakeupData *pInData, TSMakeupData *pOutData,
            const TSRect *pLeftEye, const TSRect *pRightEye, const TSRect *pMouth, int bigEyeLevel, int trimFaceLevel);

#endif // __TS_MAKEUP_ENGINI_H__
