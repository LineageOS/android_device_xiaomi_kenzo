/* Copyright (c) 2015, The Linux Foundataion. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above
*       copyright notice, this list of conditions and the following
*       disclaimer in the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of The Linux Foundation nor the names of its
*       contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
* ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef __QCAMERA3CROPREGIONMAPPER_H__
#define __QCAMERA3CROPREGIONMAPPER_H__

#include <utils/Log.h>
#include <utils/Errors.h>
#include "QCamera3HALHeader.h"

using namespace android;

namespace qcamera {

class QCamera3CropRegionMapper {
public:
    QCamera3CropRegionMapper();
    virtual ~QCamera3CropRegionMapper();

    void update(uint32_t active_array_w, uint32_t active_array_h,
            uint32_t sensor_w, uint32_t sensor_h);
    void toActiveArray(int32_t& crop_left, int32_t& crop_top,
            int32_t& crop_width, int32_t& crop_height);
    void toSensor(int32_t& crop_left, int32_t& crop_top,
            int32_t& crop_width, int32_t& crop_height);
    void toActiveArray(uint32_t& x, uint32_t& y);
    void toSensor(uint32_t& x, uint32_t& y);

private:
    /* sensor output size */
    int32_t mSensorW, mSensorH;
    int32_t mActiveArrayW, mActiveArrayH;

    void boundToSize(int32_t& left, int32_t& top, int32_t& width,
            int32_t& height, int32_t bound_w, int32_t bound_h);
};

}; // namespace qcamera

#endif /* __QCAMERA3CROPREGIONMAPPER_H__ */
