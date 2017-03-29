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

#ifndef __TS_MAKEUP_DATA_H__
#define __TS_MAKEUP_DATA_H__

    #define TS_OK                (0x00000000)    //Successful
    #define TS_ERROR_PARAM       (0x00000001)    //Parameters error
    #define TS_ERROR_IO          (0x00000002)    //Input or output error
    #define TS_ERROR_INTERNAL    (0x00000003)    //Internal error
    #define TS_NO_MEMORY         (0x00000004)    //No memory error


    /*
     * Data struct : rectangle
     */
    typedef struct __tag_tsrect
    {
        long left;
        long top;
        long right;
        long bottom;
    } TSRect;

    /*
     * Data struct : point
     */
    typedef struct __tag_tsmakeuppoint
    {
        long x;
        long y;
    } TSPoint;


#endif // __TS_MAKEUP_DATA_H__
