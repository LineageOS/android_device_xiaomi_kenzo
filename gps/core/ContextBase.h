/* Copyright (c) 2011-2015, The Linux Foundation. All rights reserved.
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
 *     * Neither the name of The Linux Foundation, nor the names of its
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
#ifndef __LOC_CONTEXT_BASE__
#define __LOC_CONTEXT_BASE__

#include <stdbool.h>
#include <ctype.h>
#include <MsgTask.h>
#include <LocApiBase.h>
#include <LBSProxyBase.h>

#define MAX_XTRA_SERVER_URL_LENGTH 256

/* GPS.conf support */
/* NOTE: the implementaiton of the parser casts number
   fields to 32 bit. To ensure all 'n' fields working,
   they must all be 32 bit fields. */
typedef struct loc_gps_cfg_s
{
    uint32_t       INTERMEDIATE_POS;
    uint32_t       ACCURACY_THRES;
    uint32_t       SUPL_VER;
    uint32_t       SUPL_MODE;
    uint32_t       SUPL_ES;
    uint32_t       CAPABILITIES;
    uint32_t       LPP_PROFILE;
    uint32_t       XTRA_VERSION_CHECK;
    char           XTRA_SERVER_1[MAX_XTRA_SERVER_URL_LENGTH];
    char           XTRA_SERVER_2[MAX_XTRA_SERVER_URL_LENGTH];
    char           XTRA_SERVER_3[MAX_XTRA_SERVER_URL_LENGTH];
    uint32_t       USE_EMERGENCY_PDN_FOR_EMERGENCY_SUPL;
    uint32_t       NMEA_PROVIDER;
    uint32_t       GPS_LOCK;
    uint32_t       A_GLONASS_POS_PROTOCOL_SELECT;
    uint32_t       AGPS_CERT_WRITABLE_MASK;
} loc_gps_cfg_s_type;

/* NOTE: the implementaiton of the parser casts number
   fields to 32 bit. To ensure all 'n' fields working,
   they must all be 32 bit fields. */
/* Meanwhile, *_valid fields are 8 bit fields, and 'f'
   fields are double. Rigid as they are, it is the
   the status quo, until the parsing mechanism is
   change, that is. */
typedef struct
{
    uint8_t        GYRO_BIAS_RANDOM_WALK_VALID;
    double         GYRO_BIAS_RANDOM_WALK;
    uint32_t       SENSOR_ACCEL_BATCHES_PER_SEC;
    uint32_t       SENSOR_ACCEL_SAMPLES_PER_BATCH;
    uint32_t       SENSOR_GYRO_BATCHES_PER_SEC;
    uint32_t       SENSOR_GYRO_SAMPLES_PER_BATCH;
    uint32_t       SENSOR_ACCEL_BATCHES_PER_SEC_HIGH;
    uint32_t       SENSOR_ACCEL_SAMPLES_PER_BATCH_HIGH;
    uint32_t       SENSOR_GYRO_BATCHES_PER_SEC_HIGH;
    uint32_t       SENSOR_GYRO_SAMPLES_PER_BATCH_HIGH;
    uint32_t       SENSOR_CONTROL_MODE;
    uint32_t       SENSOR_USAGE;
    uint32_t       SENSOR_ALGORITHM_CONFIG_MASK;
    uint8_t        ACCEL_RANDOM_WALK_SPECTRAL_DENSITY_VALID;
    double         ACCEL_RANDOM_WALK_SPECTRAL_DENSITY;
    uint8_t        ANGLE_RANDOM_WALK_SPECTRAL_DENSITY_VALID;
    double         ANGLE_RANDOM_WALK_SPECTRAL_DENSITY;
    uint8_t        RATE_RANDOM_WALK_SPECTRAL_DENSITY_VALID;
    double         RATE_RANDOM_WALK_SPECTRAL_DENSITY;
    uint8_t        VELOCITY_RANDOM_WALK_SPECTRAL_DENSITY_VALID;
    double         VELOCITY_RANDOM_WALK_SPECTRAL_DENSITY;
    uint32_t       SENSOR_PROVIDER;
} loc_sap_cfg_s_type;

namespace loc_core {

class LocAdapterBase;

class ContextBase {
    static LBSProxyBase* getLBSProxy(const char* libName);
    LocApiBase* createLocApi(LOC_API_ADAPTER_EVENT_MASK_T excludedMask);
protected:
    const LBSProxyBase* mLBSProxy;
    const MsgTask* mMsgTask;
    LocApiBase* mLocApi;
    LocApiProxyBase *mLocApiProxy;
public:
    ContextBase(const MsgTask* msgTask,
                LOC_API_ADAPTER_EVENT_MASK_T exMask,
                const char* libName);
    inline virtual ~ContextBase() { delete mLocApi; delete mLBSProxy; }

    inline const MsgTask* getMsgTask() { return mMsgTask; }
    inline LocApiBase* getLocApi() { return mLocApi; }
    inline LocApiProxyBase* getLocApiProxy() { return mLocApiProxy; }
    inline bool hasAgpsExtendedCapabilities() { return mLBSProxy->hasAgpsExtendedCapabilities(); }
    inline bool hasCPIExtendedCapabilities() { return mLBSProxy->hasCPIExtendedCapabilities(); }
    inline bool hasNativeXtraClient() { return mLBSProxy->hasNativeXtraClient(); }
    inline void modemPowerVote(bool power) const { return mLBSProxy->modemPowerVote(power); }
    inline void requestUlp(LocAdapterBase* adapter,
                           unsigned long capabilities) {
        mLBSProxy->requestUlp(adapter, capabilities);
    }
    inline IzatDevId_t getIzatDevId() const {
        return mLBSProxy->getIzatDevId();
    }
    inline void sendMsg(const LocMsg *msg) { getMsgTask()->sendMsg(msg); }

    static loc_gps_cfg_s_type mGps_conf;
    static loc_sap_cfg_s_type mSap_conf;

    static uint32_t getCarrierCapabilities();

};

} // namespace loc_core

#endif //__LOC_CONTEXT_BASE__
