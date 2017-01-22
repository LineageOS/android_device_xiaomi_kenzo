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

#ifndef __TS_MAKEUP_IMGAGE_H__
#define __TS_MAKEUP_IMGAGE_H__

    /*
     * Data struct : TSMakeupData
     */
    typedef struct  __tag_tsmakeupdata
    {
        int frameWidth;                 //NV21 Frame width.MUST > 0.
        int frameHeight;                //NV21 Frame height. MUST > 0.
        unsigned char *yBuf;            //NV21 Y buffer pointer.MUST not null.
        unsigned char *uvBuf;           //NV21 UV buffer pointer.MUST not null.
    }TSMakeupData;

     /*
     * Data struct : TSMakeupDataEx
     */
    typedef struct  __tag_tsmakeupdataEx
    {
        int frameWidth;                 //NV21 Frame width.MUST > 0.
        int frameHeight;                //NV21 Frame height. MUST > 0.
        unsigned char *yBuf;            //NV21 Y buffer pointer.MUST not null.
        unsigned char *uvBuf;           //NV21 UV buffer pointer.MUST not null.
        int yStride;                    //NV21 Y buffer stride len
        int uvStride;                   //NV21 uv buffer stride len
    }TSMakeupDataEx;


#endif // __TS_MAKEUP_IMGAGE_H__
