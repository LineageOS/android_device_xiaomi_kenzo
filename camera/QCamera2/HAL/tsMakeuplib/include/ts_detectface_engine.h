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

#ifndef __TS_DETECTFACE_ENGINE_H__
#define __TS_DETECTFACE_ENGINE_H__
#include "ts_makeup_data.h"
#include "ts_makeup_image.h"

    typedef void*                    TSHandle;

    /*===========================================================================
     * FUNCTION   : ts_detectface_create_context
     *
     * DESCRIPTION: create context.The method MUST call at first time.
     *
     *
     * RETURN    : TSHandle as the context handle
     *
     *==========================================================================*/
    TSHandle ts_detectface_create_context();


    /*===========================================================================
     * FUNCTION   : ts_detectface_destroy_context
     *
     * DESCRIPTION: destroy context. The method MUST call at last time.
     * Before you MUST call ts_detectface_create_context method
     * to create context and get context handle.
     *
     * PARAMETERS :
     *   @param[in] contexTSHandle : The context handle pointer.
     *
     *
     *==========================================================================*/
    void ts_detectface_destroy_context(TSHandle* contexTSHandle);


    /*===========================================================================
     * FUNCTION   : ts_detectface_detect
     *
     * DESCRIPTION: start detect.Before you MUST call ts_detectface_create_context method
     * to create context and get context handle.
     *
     * PARAMETERS :
     *   @param[in] contexTSHandle : The context handle.
     *   @param[in] pInData : The TSMakeupData pointer.MUST not NULL.
     *
     * RETURN    : int If less than zero failed, otherwise the number of the detected faces.
     *
     *==========================================================================*/
    int ts_detectface_detect(TSHandle contexTSHandle, TSMakeupData *pInData);

    /*===========================================================================
     * FUNCTION   : ts_detectface_detectEx
     *
     * DESCRIPTION: start detect.Before you MUST call ts_detectface_create_context method
     * to create context and get context handle.
     *
     * PARAMETERS :
     *   @param[in] contexTSHandle : The context handle.
     *   @param[in] pInData : The TSMakeupDataEx pointer.MUST not NULL.
     *
     * RETURN    : int If less than zero failed, otherwise the number of the detected faces.
     *
     *==========================================================================*/
    int ts_detectface_detectEx(TSHandle contexTSHandle, TSMakeupDataEx *pInData);
    /*===========================================================================
     * FUNCTION   : ts_detectface_get_face_info
     *
     * DESCRIPTION: get detected face information.Before you MUST call ts_detectface_detect method
     * to detect face.
     *
     * PARAMETERS :
     *   @param[in] contexTSHandle : The context handle.
     *   @param[in] index : The face index.MUST > 0.
     *   @param[out] pFaceRect : The face rects.MUST not NULL.
     *   @param[out] leftEye : The left eye rect.
     *   @param[out] rightEye : The right eye rect.
     *   @param[out] pMouth : The mount rect.
     *
     * RETURN    : TS_OK if success, otherwise failed.
     *
     *==========================================================================*/
    int ts_detectface_get_face_info(TSHandle contexTSHandle, int index, TSRect *pFaceRect, TSRect *leftEye, TSRect *rightEye, TSRect *pMouth);

#endif // __TS_DETECTFACE_ENGINE_H__
