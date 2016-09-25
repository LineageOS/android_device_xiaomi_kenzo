/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
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
#define LOG_NDDEBUG 0
#define LOG_TAG "LocSvc_api_rpc"

#include <unistd.h>
#include <math.h>
#ifndef USE_GLIB
#include <utils/SystemClock.h>
#endif /* USE_GLIB */
#include <LocApiRpc.h>
#include <LocAdapterBase.h>
#include <loc_api_fixup.h>
#include <loc_api_rpc_glue.h>
#include <log_util.h>
#include <loc_log.h>
#include <loc_api_log.h>
#ifdef USE_GLIB
#include <glib.h>
#endif
#include <librpc.h>
#include <platform_lib_includes.h>

using namespace loc_core;

#define LOC_XTRA_INJECT_DEFAULT_TIMEOUT (3100)
#define XTRA_BLOCK_SIZE                 (3072)
#define LOC_IOCTL_DEFAULT_TIMEOUT 1000 // 1000 milli-seconds
#define LOC_NI_NOTIF_KEY_ADDRESS           "Address"

/*===========================================================================
FUNCTION    loc_event_cb

DESCRIPTION
   This is the callback function registered by loc_open.

DEPENDENCIES
   N/A

RETURN VALUE
   RPC_LOC_API_SUCCESS

SIDE EFFECTS
   N/A

===========================================================================*/
static int32 loc_event_cb
(
    void*                                user,
    rpc_loc_client_handle_type           client_handle,
    rpc_loc_event_mask_type              loc_event,
    const rpc_loc_event_payload_u_type*  loc_event_payload
)
{
    MODEM_LOG_CALLFLOW(%s, loc_get_event_name(loc_event));
    loc_callback_log(loc_event, loc_event_payload);
    int32 ret_val = ((LocApiRpc*)user)->locEventCB(client_handle, loc_event, loc_event_payload);
    EXIT_LOG(%d, ret_val);
    return ret_val;
}

/*===========================================================================
FUNCTION    loc_eng_rpc_global_cb

DESCRIPTION
   This is the callback function registered by loc_open for RPC global events

DEPENDENCIES
   N/A

RETURN VALUE
   RPC_LOC_API_SUCCESS

SIDE EFFECTS
   N/A

===========================================================================*/
static void loc_rpc_global_cb(void* user, CLIENT* clnt, enum rpc_reset_event event)
{
    MODEM_LOG_CALLFLOW(%s, loc_get_rpc_reset_event_name(event));
    ((LocApiRpc*)user)->locRpcGlobalCB(clnt, event);
    EXIT_LOG(%p, VOID_RET);
}

const LOC_API_ADAPTER_EVENT_MASK_T LocApiRpc::maskAll =
    LOC_API_ADAPTER_BIT_PARSED_POSITION_REPORT |
    LOC_API_ADAPTER_BIT_SATELLITE_REPORT |
    LOC_API_ADAPTER_BIT_LOCATION_SERVER_REQUEST |
    LOC_API_ADAPTER_BIT_ASSISTANCE_DATA_REQUEST |
    LOC_API_ADAPTER_BIT_IOCTL_REPORT |
    LOC_API_ADAPTER_BIT_STATUS_REPORT |
    LOC_API_ADAPTER_BIT_NMEA_1HZ_REPORT |
    LOC_API_ADAPTER_BIT_NI_NOTIFY_VERIFY_REQUEST;

const rpc_loc_event_mask_type LocApiRpc::locBits[] =
{
    RPC_LOC_EVENT_PARSED_POSITION_REPORT,
    RPC_LOC_EVENT_SATELLITE_REPORT,
    RPC_LOC_EVENT_NMEA_1HZ_REPORT,
    RPC_LOC_EVENT_NMEA_POSITION_REPORT,
    RPC_LOC_EVENT_NI_NOTIFY_VERIFY_REQUEST,
    RPC_LOC_EVENT_ASSISTANCE_DATA_REQUEST,
    RPC_LOC_EVENT_LOCATION_SERVER_REQUEST,
    RPC_LOC_EVENT_IOCTL_REPORT,
    RPC_LOC_EVENT_STATUS_REPORT,
    RPC_LOC_EVENT_WPS_NEEDED_REQUEST
};

LocApiRpc*
LocApiRpc::createLocApiRpc(const MsgTask* msgTask,
                     LOC_API_ADAPTER_EVENT_MASK_T exMask,
                     ContextBase* context)
{
    if (NULL == msgTask) {
        return NULL;
    }
    return new LocApiRpc(msgTask, exMask, context);
}

// constructor
LocApiRpc::LocApiRpc(const MsgTask* msgTask,
                     LOC_API_ADAPTER_EVENT_MASK_T exMask,
                     ContextBase* context) :
    LocApiBase(msgTask, exMask, context),
    client_handle(RPC_LOC_CLIENT_HANDLE_INVALID),
    dataEnableLastSet(-1)
{
    memset(apnLastSet, 0, sizeof(apnLastSet));
    loc_api_glue_init();
}

LocApiRpc::~LocApiRpc()
{
    close();
}

rpc_loc_event_mask_type
LocApiRpc::convertMask(LOC_API_ADAPTER_EVENT_MASK_T mask)
{
    rpc_loc_event_mask_type newMask = 0;

    for (unsigned int i = 0, bit=1; 0 != mask; i++, bit<<=1) {
        if (mask & bit) {
            newMask |= locBits[i];
            mask ^= bit;
        }
    }

    return newMask;
}

rpc_loc_lock_e_type
LocApiRpc::convertGpsLockMask(LOC_GPS_LOCK_MASK lockMask)
{
    if (isGpsLockAll(lockMask))
        return RPC_LOC_LOCK_ALL;
    if (isGpsLockMO(lockMask))
        return RPC_LOC_LOCK_MI;
    if (isGpsLockMT(lockMask))
        return RPC_LOC_LOCK_MT;
    if (isGpsLockNone(lockMask))
        return RPC_LOC_LOCK_NONE;
    return (rpc_loc_lock_e_type)lockMask;
}

enum loc_api_adapter_err
LocApiRpc::convertErr(int rpcErr)
{
    switch(rpcErr)
    {
    case RPC_LOC_API_SUCCESS:
        return LOC_API_ADAPTER_ERR_SUCCESS;
    case RPC_LOC_API_GENERAL_FAILURE:
        return LOC_API_ADAPTER_ERR_GENERAL_FAILURE;
    case RPC_LOC_API_UNSUPPORTED:
        return LOC_API_ADAPTER_ERR_UNSUPPORTED;
    case RPC_LOC_API_INVALID_HANDLE:
        return LOC_API_ADAPTER_ERR_INVALID_HANDLE;
    case RPC_LOC_API_INVALID_PARAMETER:
        return LOC_API_ADAPTER_ERR_INVALID_PARAMETER;
    case RPC_LOC_API_ENGINE_BUSY:
        return LOC_API_ADAPTER_ERR_ENGINE_BUSY;
    case RPC_LOC_API_PHONE_OFFLINE:
        return LOC_API_ADAPTER_ERR_PHONE_OFFLINE;
    case RPC_LOC_API_TIMEOUT:
        return LOC_API_ADAPTER_ERR_TIMEOUT;
    case RPC_LOC_API_RPC_MODEM_RESTART:
        return LOC_API_ADAPTER_ERR_ENGINE_DOWN;
    case RPC_LOC_API_RPC_FAILURE:
        return LOC_API_ADAPTER_ERR_FAILURE;
    default:
        return LOC_API_ADAPTER_ERR_UNKNOWN;
    }
}

void LocApiRpc::locRpcGlobalCB(CLIENT* clnt, enum rpc_reset_event event)
{
    static rpc_loc_engine_state_e_type last_state = RPC_LOC_ENGINE_STATE_MAX;

    switch (event) {
    case RPC_SUBSYSTEM_RESTART_BEGIN:
        if (RPC_LOC_ENGINE_STATE_OFF != last_state) {
            last_state = RPC_LOC_ENGINE_STATE_OFF;
            handleEngineDownEvent();
        }
        break;
    case RPC_SUBSYSTEM_RESTART_END:
        if (RPC_LOC_ENGINE_STATE_ON != last_state) {
            last_state = RPC_LOC_ENGINE_STATE_ON;
            handleEngineUpEvent();
        }
        break;
    }
}

int32 LocApiRpc::locEventCB(rpc_loc_client_handle_type client_handle,
                     rpc_loc_event_mask_type loc_event,
                     const rpc_loc_event_payload_u_type* loc_event_payload)
{
    // Parsed report
    if (loc_event & RPC_LOC_EVENT_PARSED_POSITION_REPORT)
    {
        reportPosition(&loc_event_payload->rpc_loc_event_payload_u_type_u.
                       parsed_location_report);
    }

    // Satellite report
    if (loc_event & RPC_LOC_EVENT_SATELLITE_REPORT)
    {
        reportSv(&loc_event_payload->rpc_loc_event_payload_u_type_u.gnss_report);
    }

    // Status report
    if (loc_event & RPC_LOC_EVENT_STATUS_REPORT)
    {
        reportStatus(&loc_event_payload->rpc_loc_event_payload_u_type_u.status_report);
    }

    // NMEA
    if (loc_event & RPC_LOC_EVENT_NMEA_1HZ_REPORT)
    {
        reportNmea(&(loc_event_payload->rpc_loc_event_payload_u_type_u.nmea_report));
    }
    // XTRA support: supports only XTRA download
    if (loc_event & RPC_LOC_EVENT_ASSISTANCE_DATA_REQUEST)
    {
        if (loc_event_payload->rpc_loc_event_payload_u_type_u.assist_data_request.event ==
            RPC_LOC_ASSIST_DATA_PREDICTED_ORBITS_REQ)
        {
            requestXtraData();
        } else if (loc_event_payload->rpc_loc_event_payload_u_type_u.assist_data_request.event ==
                   RPC_LOC_ASSIST_DATA_TIME_REQ)
        {
            requestTime();
        } else if (loc_event_payload->rpc_loc_event_payload_u_type_u.assist_data_request.event ==
                   RPC_LOC_ASSIST_DATA_POSITION_INJECTION_REQ)
        {
            requestLocation();
        }
    }

    // AGPS data request
    if (loc_event & RPC_LOC_EVENT_LOCATION_SERVER_REQUEST)
    {
        ATLEvent(&loc_event_payload->rpc_loc_event_payload_u_type_u.
                 loc_server_request);
    }

    // NI notify request
    if (loc_event & RPC_LOC_EVENT_NI_NOTIFY_VERIFY_REQUEST)
    {
        NIEvent(&loc_event_payload->rpc_loc_event_payload_u_type_u.ni_request);
    }

    return RPC_LOC_API_SUCCESS;//We simply want to return sucess here as we do not want to
    // cause any issues in RPC thread context
}

enum loc_api_adapter_err
LocApiRpc::open(LOC_API_ADAPTER_EVENT_MASK_T mask)
{
    enum loc_api_adapter_err ret_val = LOC_API_ADAPTER_ERR_SUCCESS;

    // RPC does not dynamically update the event mask. And in the
    // case of RPC, all we support are positioning (gps + agps)
    // masks anyways, so we simply mask all of them on always.
    // After doing so the first time in a power cycle, we know there
    // will the following if condition will never be true any more.
    mask = maskAll;

    if (mask != mMask) {
        if (RPC_LOC_CLIENT_HANDLE_INVALID != client_handle) {
            close();
        }

        mMask = mask;
        // it is important to cap the mask here, because not all LocApi's
        // can enable the same bits, e.g. foreground and bckground.
        client_handle = loc_open(convertMask(mask),
                                 loc_event_cb,
                                 loc_rpc_global_cb, this);

        if (client_handle < 0) {
            mMask = 0;
            client_handle = RPC_LOC_CLIENT_HANDLE_INVALID;
            ret_val = LOC_API_ADAPTER_ERR_INVALID_HANDLE;
        }
    }

    return ret_val;
}

enum loc_api_adapter_err
LocApiRpc::close()
{
    if (RPC_LOC_CLIENT_HANDLE_INVALID != client_handle) {
        loc_clear(client_handle);
    }

    loc_close(client_handle);
    mMask = 0;
    client_handle = RPC_LOC_CLIENT_HANDLE_INVALID;

    return LOC_API_ADAPTER_ERR_SUCCESS;
}

enum loc_api_adapter_err
LocApiRpc::startFix(const LocPosMode& posMode) {
   LOC_LOGD("LocApiRpc::startFix() called");
   return convertErr(
       loc_start_fix(client_handle)
       );
}

enum loc_api_adapter_err
LocApiRpc::stopFix() {
   LOC_LOGD("LocApiRpc::stopFix() called");
   return convertErr(
       loc_stop_fix(client_handle)
       );
}

enum loc_api_adapter_err
LocApiRpc::setPositionMode(const LocPosMode& posMode)
{
    rpc_loc_ioctl_data_u_type    ioctl_data;
    rpc_loc_fix_criteria_s_type *fix_criteria_ptr =
        &ioctl_data.rpc_loc_ioctl_data_u_type_u.fix_criteria;
    rpc_loc_ioctl_e_type         ioctl_type = RPC_LOC_IOCTL_SET_FIX_CRITERIA;
    rpc_loc_operation_mode_e_type op_mode;
    int                          ret_val;
    const LocPosMode* fixCriteria = &posMode;

    ALOGD ("loc_eng_set_position mode, client = %d, interval = %d, mode = %d\n",
          (int32) client_handle, fixCriteria->min_interval, fixCriteria->mode);

    switch (fixCriteria->mode)
    {
    case LOC_POSITION_MODE_MS_BASED:
        op_mode = RPC_LOC_OPER_MODE_MSB;
        break;
    case LOC_POSITION_MODE_MS_ASSISTED:
        op_mode = RPC_LOC_OPER_MODE_MSA;
        break;
    case LOC_POSITION_MODE_RESERVED_1:
        op_mode = RPC_LOC_OPER_MODE_SPEED_OPTIMAL;
        break;
    case LOC_POSITION_MODE_RESERVED_2:
        op_mode = RPC_LOC_OPER_MODE_ACCURACY_OPTIMAL;
        break;
    case LOC_POSITION_MODE_RESERVED_3:
        op_mode = RPC_LOC_OPER_MODE_DATA_OPTIMAL;
        break;
    case LOC_POSITION_MODE_RESERVED_4:
    case LOC_POSITION_MODE_RESERVED_5:
        op_mode = RPC_LOC_OPER_MODE_MSA;
        fix_criteria_ptr->preferred_response_time  = 0;
        break;
    default:
        op_mode = RPC_LOC_OPER_MODE_STANDALONE;
    }

    fix_criteria_ptr->valid_mask = RPC_LOC_FIX_CRIT_VALID_PREFERRED_OPERATION_MODE |
                                   RPC_LOC_FIX_CRIT_VALID_RECURRENCE_TYPE;
    fix_criteria_ptr->min_interval = fixCriteria->min_interval;
    fix_criteria_ptr->preferred_operation_mode = op_mode;

    fix_criteria_ptr->min_interval = fixCriteria->min_interval;
    fix_criteria_ptr->valid_mask |= RPC_LOC_FIX_CRIT_VALID_MIN_INTERVAL;

    if (fixCriteria->preferred_accuracy > 0) {
        fix_criteria_ptr->preferred_accuracy = fixCriteria->preferred_accuracy;
        fix_criteria_ptr->valid_mask |= RPC_LOC_FIX_CRIT_VALID_PREFERRED_ACCURACY;
    }
    if (fixCriteria->preferred_time > 0) {
        fix_criteria_ptr->preferred_response_time = fixCriteria->preferred_time;
        fix_criteria_ptr->valid_mask |= RPC_LOC_FIX_CRIT_VALID_PREFERRED_RESPONSE_TIME;
    }

    switch (fixCriteria->recurrence) {
    case GPS_POSITION_RECURRENCE_SINGLE:
        fix_criteria_ptr->recurrence_type = RPC_LOC_SINGLE_FIX;
        break;
    case GPS_POSITION_RECURRENCE_PERIODIC:
    default:
        fix_criteria_ptr->recurrence_type = RPC_LOC_PERIODIC_FIX;
        break;
    }
    ioctl_data.disc = ioctl_type;

    ret_val = loc_eng_ioctl (client_handle,
                             ioctl_type,
                             &ioctl_data,
                             LOC_IOCTL_DEFAULT_TIMEOUT,
                             NULL /* No output information is expected*/);

    return convertErr(ret_val);
}

enum loc_api_adapter_err
LocApiRpc::setTime(GpsUtcTime time, int64_t timeReference, int uncertainty)
{
    rpc_loc_ioctl_data_u_type        ioctl_data;
    rpc_loc_assist_data_time_s_type *time_info_ptr;
    rpc_loc_ioctl_e_type             ioctl_type = RPC_LOC_IOCTL_INJECT_UTC_TIME;
    int                              ret_val;

    LOC_LOGD ("loc_eng_inject_time, uncertainty = %d\n", uncertainty);

    time_info_ptr = &ioctl_data.rpc_loc_ioctl_data_u_type_u.assistance_data_time;
    time_info_ptr->time_utc = time;
    time_info_ptr->time_utc += (int64_t)(ELAPSED_MILLIS_SINCE_BOOT_PLATFORM_LIB_ABSTRACTION - timeReference);
    time_info_ptr->uncertainty = uncertainty; // Uncertainty in ms

    ioctl_data.disc = ioctl_type;

    ret_val = loc_eng_ioctl (client_handle,
                             ioctl_type,
                             &ioctl_data,
                             LOC_IOCTL_DEFAULT_TIMEOUT,
                             NULL /* No output information is expected*/);

    return convertErr(ret_val);
}

enum loc_api_adapter_err
LocApiRpc::injectPosition(double latitude, double longitude, float accuracy)
{
    /* IOCTL data */
    rpc_loc_ioctl_data_u_type ioctl_data;
    rpc_loc_assist_data_pos_s_type *assistance_data_position =
        &ioctl_data.rpc_loc_ioctl_data_u_type_u.assistance_data_position;
    int                          ret_val;

    /************************************************
     * Fill in latitude, longitude & accuracy
     ************************************************/

    /* This combo is required */
    assistance_data_position->valid_mask =
        RPC_LOC_ASSIST_POS_VALID_LATITUDE |
        RPC_LOC_ASSIST_POS_VALID_LONGITUDE |
        RPC_LOC_ASSIST_POS_VALID_HOR_UNC_CIRCULAR |
        RPC_LOC_ASSIST_POS_VALID_CONFIDENCE_HORIZONTAL;

    assistance_data_position->latitude = latitude;
    assistance_data_position->longitude = longitude;
    assistance_data_position->hor_unc_circular = accuracy; /* Meters assumed */
    assistance_data_position->confidence_horizontal = 63;  /* 63% (1 std dev) assumed */

    /* Log */
    LOC_LOGD("Inject coarse position Lat=%lf, Lon=%lf, Acc=%.2lf\n",
             (double) assistance_data_position->latitude,
             (double) assistance_data_position->longitude,
             (double) assistance_data_position->hor_unc_circular);

    ret_val = loc_eng_ioctl( client_handle,
                             RPC_LOC_IOCTL_INJECT_POSITION,
                             &ioctl_data,
                             LOC_IOCTL_DEFAULT_TIMEOUT,
                             NULL /* No output information is expected*/);
    return convertErr(ret_val);
}

enum loc_api_adapter_err
LocApiRpc::informNiResponse(GpsUserResponseType userResponse,
                                   const void* passThroughData)
{
    rpc_loc_ioctl_data_u_type data;
    rpc_loc_ioctl_callback_s_type callback_payload;

    memcpy(&data.rpc_loc_ioctl_data_u_type_u.user_verify_resp.ni_event_pass_back,
           passThroughData, sizeof (rpc_loc_ni_event_s_type));

    rpc_loc_ni_user_resp_e_type resp;
    switch (userResponse)
    {
    case GPS_NI_RESPONSE_ACCEPT:
        data.rpc_loc_ioctl_data_u_type_u.user_verify_resp.user_resp =
            RPC_LOC_NI_LCS_NOTIFY_VERIFY_ACCEPT;
        break;
    case GPS_NI_RESPONSE_DENY:
        data.rpc_loc_ioctl_data_u_type_u.user_verify_resp.user_resp =
            RPC_LOC_NI_LCS_NOTIFY_VERIFY_DENY;
        break;
    case GPS_NI_RESPONSE_NORESP:
    default:
        data.rpc_loc_ioctl_data_u_type_u.user_verify_resp.user_resp =
            RPC_LOC_NI_LCS_NOTIFY_VERIFY_NORESP;
        break;
    }

    return convertErr(
        loc_eng_ioctl(client_handle,
                      RPC_LOC_IOCTL_INFORM_NI_USER_RESPONSE,
                      &data,
                      LOC_IOCTL_DEFAULT_TIMEOUT,
                      &callback_payload)
        );
}

enum loc_api_adapter_err
LocApiRpc::setAPN(char* apn, int len, boolean force)
{
    enum loc_api_adapter_err rtv = LOC_API_ADAPTER_ERR_SUCCESS;
    int size = sizeof(apnLastSet);
    if (force || memcmp(apnLastSet, apn, size)) {
        if (len < size) {
            // size will be not larger than its original value
            size = len + 1;
        }
        memcpy(apnLastSet, apn, size);

        if (!isInSession()) {
            rpc_loc_ioctl_data_u_type ioctl_data = {RPC_LOC_IOCTL_SET_LBS_APN_PROFILE, {0}};
            ioctl_data.rpc_loc_ioctl_data_u_type_u.apn_profiles[0].srv_system_type = LOC_APN_PROFILE_SRV_SYS_MAX;
            ioctl_data.rpc_loc_ioctl_data_u_type_u.apn_profiles[0].pdp_type = LOC_APN_PROFILE_PDN_TYPE_IPV4;
            memcpy(&(ioctl_data.rpc_loc_ioctl_data_u_type_u.apn_profiles[0].apn_name), apn, size);

            rtv = convertErr(
                loc_eng_ioctl (client_handle,
                               RPC_LOC_IOCTL_SET_LBS_APN_PROFILE,
                               &ioctl_data,
                               LOC_IOCTL_DEFAULT_TIMEOUT,
                               NULL)
                );
        }
    }
    return rtv;
}

void LocApiRpc::setInSession(bool inSession)
{
    if (!inSession) {
        enableData(dataEnableLastSet, true);
        setAPN(apnLastSet, sizeof(apnLastSet)-1, true);
    }
}

enum loc_api_adapter_err
LocApiRpc::setServer(const char* url, int len)
{
    rpc_loc_ioctl_data_u_type         ioctl_data;
    rpc_loc_server_info_s_type       *server_info_ptr;
    rpc_loc_ioctl_e_type              ioctl_cmd;

    ioctl_cmd = RPC_LOC_IOCTL_SET_UMTS_SLP_SERVER_ADDR;
    ioctl_data.disc = ioctl_cmd;
    server_info_ptr = &ioctl_data.rpc_loc_ioctl_data_u_type_u.server_addr;
    server_info_ptr->addr_type = RPC_LOC_SERVER_ADDR_URL;
    server_info_ptr->addr_info.disc = server_info_ptr->addr_type;
    server_info_ptr->addr_info.rpc_loc_server_addr_u_type_u.url.length = len;
#if (AMSS_VERSION==3200)
    server_info_ptr->addr_info.rpc_loc_server_addr_u_type_u.url.addr.addr_val = (char*) url;
    server_info_ptr->addr_info.rpc_loc_server_addr_u_type_u.url.addr.addr_len= len;
#else
    strlcpy(server_info_ptr->addr_info.rpc_loc_server_addr_u_type_u.url.addr, url,
            sizeof server_info_ptr->addr_info.rpc_loc_server_addr_u_type_u.url.addr);
#endif /* #if (AMSS_VERSION==3200) */
    LOC_LOGD ("loc_eng_set_server, addr = %s\n", url);

    return convertErr(
        loc_eng_ioctl (client_handle,
                       ioctl_cmd,
                       &ioctl_data,
                       LOC_IOCTL_DEFAULT_TIMEOUT,
                       NULL /* No output information is expected*/)
        );
}

enum loc_api_adapter_err
LocApiRpc::setServer(unsigned int ip, int port, LocServerType type)
{
    rpc_loc_ioctl_data_u_type         ioctl_data;
    rpc_loc_server_info_s_type       *server_info_ptr;
    rpc_loc_ioctl_e_type              ioctl_cmd;

    switch (type) {
    case LOC_AGPS_MPC_SERVER:
        ioctl_cmd = RPC_LOC_IOCTL_SET_CDMA_MPC_SERVER_ADDR;
        break;
    case LOC_AGPS_CUSTOM_PDE_SERVER:
        ioctl_cmd = RPC_LOC_IOCTL_SET_CUSTOM_PDE_SERVER_ADDR;
        break;
    default:
        ioctl_cmd = RPC_LOC_IOCTL_SET_CDMA_PDE_SERVER_ADDR;
        break;
    }
    ioctl_data.disc = ioctl_cmd;
    server_info_ptr = &ioctl_data.rpc_loc_ioctl_data_u_type_u.server_addr;
    server_info_ptr->addr_type = RPC_LOC_SERVER_ADDR_IPV4;
    server_info_ptr->addr_info.disc = server_info_ptr->addr_type;
    server_info_ptr->addr_info.rpc_loc_server_addr_u_type_u.ipv4.addr = ip;
    server_info_ptr->addr_info.rpc_loc_server_addr_u_type_u.ipv4.port = port;
    LOC_LOGD ("setServer, addr = %X:%d\n", (unsigned int) ip, (unsigned int) port);

    return convertErr(
        loc_eng_ioctl (client_handle,
                       ioctl_cmd,
                       &ioctl_data,
                       LOC_IOCTL_DEFAULT_TIMEOUT,
                       NULL /* No output information is expected*/)
        );
}

enum loc_api_adapter_err
LocApiRpc::enableData(int enable, boolean force)
{
    enum loc_api_adapter_err rtv = LOC_API_ADAPTER_ERR_SUCCESS;
    if (force || dataEnableLastSet != enable) {
        dataEnableLastSet = enable;

        if (!isInSession()) {
            rpc_loc_ioctl_data_u_type ioctl_data = {RPC_LOC_IOCTL_SET_DATA_ENABLE, {0}};

            ioctl_data.rpc_loc_ioctl_data_u_type_u.data_enable = enable;
            rtv =  convertErr(
                loc_eng_ioctl (client_handle,
                               RPC_LOC_IOCTL_SET_DATA_ENABLE,
                               &ioctl_data,
                               LOC_IOCTL_DEFAULT_TIMEOUT,
                               NULL)
                );
        }
    }
    return rtv;
}

enum loc_api_adapter_err
LocApiRpc::deleteAidingData(GpsAidingData bits)
{
    rpc_loc_ioctl_data_u_type ioctl_data = {RPC_LOC_IOCTL_DELETE_ASSIST_DATA, {0}};
    ioctl_data.rpc_loc_ioctl_data_u_type_u.assist_data_delete.type = bits;

    return convertErr(
        loc_eng_ioctl (client_handle,
                       RPC_LOC_IOCTL_DELETE_ASSIST_DATA,
                       &ioctl_data,
                       LOC_IOCTL_DEFAULT_TIMEOUT,
                       NULL)
        );
}

void LocApiRpc::reportPosition(const rpc_loc_parsed_position_s_type *location_report_ptr)
{
    LocPosTechMask tech_Mask = LOC_POS_TECH_MASK_DEFAULT;

    UlpLocation location = {0};
    GpsLocationExtended locationExtended = {0};

    location.size = sizeof(location);
    locationExtended.size = sizeof(locationExtended);
    if (location_report_ptr->valid_mask & RPC_LOC_POS_VALID_SESSION_STATUS)
    {
        // Process the position from final and intermediate reports
        if (location_report_ptr->session_status == RPC_LOC_SESS_STATUS_SUCCESS ||
            location_report_ptr->session_status == RPC_LOC_SESS_STATUS_IN_PROGESS)
        {
            // Latitude & Longitude
            if ((location_report_ptr->valid_mask & RPC_LOC_POS_VALID_LATITUDE) &&
                (location_report_ptr->valid_mask & RPC_LOC_POS_VALID_LONGITUDE) &&
                (location_report_ptr->latitude != 0 ||
                 location_report_ptr->longitude != 0))
            {
                location.gpsLocation.flags    |= GPS_LOCATION_HAS_LAT_LONG;
                location.gpsLocation.latitude  = location_report_ptr->latitude;
                location.gpsLocation.longitude = location_report_ptr->longitude;

                // Time stamp (UTC)
                if (location_report_ptr->valid_mask & RPC_LOC_POS_VALID_TIMESTAMP_UTC)
                {
                    location.gpsLocation.timestamp = location_report_ptr->timestamp_utc;
                }

                // Altitude
                if (location_report_ptr->valid_mask &  RPC_LOC_POS_VALID_ALTITUDE_WRT_ELLIPSOID )
                {
                    location.gpsLocation.flags    |= GPS_LOCATION_HAS_ALTITUDE;
                    location.gpsLocation.altitude = location_report_ptr->altitude_wrt_ellipsoid;
                }

                // Speed
                if (location_report_ptr->valid_mask & RPC_LOC_POS_VALID_SPEED_HORIZONTAL)
                {
                    location.gpsLocation.flags    |= GPS_LOCATION_HAS_SPEED;
                    location.gpsLocation.speed = location_report_ptr->speed_horizontal;
                }

                // Heading
                if (location_report_ptr->valid_mask &  RPC_LOC_POS_VALID_HEADING)
                {
                    location.gpsLocation.flags    |= GPS_LOCATION_HAS_BEARING;
                    location.gpsLocation.bearing = location_report_ptr->heading;
                }

                // Uncertainty (circular)
                if ( (location_report_ptr->valid_mask & RPC_LOC_POS_VALID_HOR_UNC_CIRCULAR) )
                {
                    location.gpsLocation.flags    |= GPS_LOCATION_HAS_ACCURACY;
                    location.gpsLocation.accuracy = location_report_ptr->hor_unc_circular;
                }

                // Technology Mask

                tech_Mask  |= location_report_ptr->technology_mask;
                //Mark the location source as from GNSS
                location.gpsLocation.flags |= LOCATION_HAS_SOURCE_INFO;
                location.position_source = ULP_LOCATION_IS_FROM_GNSS;
                if (location_report_ptr->valid_mask & RPC_LOC_POS_VALID_ALTITUDE_WRT_MEAN_SEA_LEVEL)
                {
                    locationExtended.flags |= GPS_LOCATION_EXTENDED_HAS_ALTITUDE_MEAN_SEA_LEVEL;
                    locationExtended.altitudeMeanSeaLevel = location_report_ptr->altitude_wrt_mean_sea_level;
                }

                if (location_report_ptr->valid_mask &  RPC_LOC_POS_VALID_MAGNETIC_VARIATION )
                {
                    locationExtended.flags |= GPS_LOCATION_EXTENDED_HAS_MAG_DEV;
                    locationExtended.magneticDeviation = location_report_ptr->magnetic_deviation;
                }

                if (location_report_ptr->valid_mask & RPC_LOC_POS_VALID_VERTICAL_UNC)
                {
                   locationExtended.flags |= GPS_LOCATION_EXTENDED_HAS_VERT_UNC;
                   locationExtended.vert_unc = location_report_ptr->vert_unc;
                }

                if (location_report_ptr->valid_mask & RPC_LOC_POS_VALID_SPEED_UNC)
                {
                   locationExtended.flags |= GPS_LOCATION_EXTENDED_HAS_SPEED_UNC;
                   locationExtended.speed_unc = location_report_ptr->speed_unc;
                }

                LOC_LOGV("reportPosition: fire callback\n");
                enum loc_sess_status fixStatus =
                    (location_report_ptr->session_status
                     == RPC_LOC_SESS_STATUS_IN_PROGESS ?
                     LOC_SESS_INTERMEDIATE : LOC_SESS_SUCCESS);
                LocApiBase::reportPosition(location,
                                           locationExtended,
                                           (void*)location_report_ptr,
                                           fixStatus,
                                           tech_Mask);
            }
        }
        else
        {
            LocApiBase::reportPosition(location,
                                       locationExtended,
                                       NULL,
                                       LOC_SESS_FAILURE);
            LOC_LOGV("loc_eng_report_position: ignore position report "
                     "when session status = %d\n",
                     location_report_ptr->session_status);
        }
    }
    else
    {
        LOC_LOGV("loc_eng_report_position: ignore position report "
                 "when session status is not set\n");
    }
}

void LocApiRpc::reportSv(const rpc_loc_gnss_info_s_type *gnss_report_ptr)
{
    QtiGnssSvStatus     SvStatus = {0};
    GpsLocationExtended locationExtended = {0};
    locationExtended.size = sizeof(locationExtended);
    int             num_svs_max = 0;
    const rpc_loc_sv_info_s_type *sv_info_ptr;

    if (gnss_report_ptr->valid_mask & RPC_LOC_GNSS_INFO_VALID_SV_COUNT)
    {
        num_svs_max = gnss_report_ptr->sv_count;
        if (num_svs_max > GPS_MAX_SVS)
        {
            num_svs_max = GPS_MAX_SVS;
        }
    }

    if (gnss_report_ptr->valid_mask & RPC_LOC_GNSS_INFO_VALID_SV_LIST)
    {
        SvStatus.num_svs = 0;

        for (int i = 0; i < num_svs_max; i++)
        {
            sv_info_ptr = &(gnss_report_ptr->sv_list.sv_list_val[i]);
            if (sv_info_ptr->valid_mask & RPC_LOC_SV_INFO_VALID_SYSTEM)
            {
                if (sv_info_ptr->system == RPC_LOC_SV_SYSTEM_GPS)
                {
                    SvStatus.sv_list[SvStatus.num_svs].size = sizeof(GpsSvInfo);
                    SvStatus.sv_list[SvStatus.num_svs].prn = sv_info_ptr->prn;

                    // We only have the data field to report gps eph and alm mask
                    if ((sv_info_ptr->valid_mask & RPC_LOC_SV_INFO_VALID_HAS_EPH) &&
                        (sv_info_ptr->has_eph == 1))
                    {
                        SvStatus.ephemeris_mask |= (1 << (sv_info_ptr->prn-1));
                    }

                    if ((sv_info_ptr->valid_mask & RPC_LOC_SV_INFO_VALID_HAS_ALM) &&
                        (sv_info_ptr->has_alm == 1))
                    {
                        SvStatus.almanac_mask |= (1 << (sv_info_ptr->prn-1));
                    }

                    if ((sv_info_ptr->valid_mask & RPC_LOC_SV_INFO_VALID_PROCESS_STATUS) &&
                        (sv_info_ptr->process_status == RPC_LOC_SV_STATUS_TRACK))
                    {
                        SvStatus.gps_used_in_fix_mask |= (1 << (sv_info_ptr->prn-1));
                    }
                }
                // SBAS: GPS RPN: 120-151,
                // In exteneded measurement report, we follow nmea standard, which is from 33-64.
                else if (sv_info_ptr->system == RPC_LOC_SV_SYSTEM_SBAS)
                {
                    SvStatus.sv_list[SvStatus.num_svs].prn = sv_info_ptr->prn + 33 - 120;
                }
                // Gloness: Slot id: 1-32
                // In extended measurement report, we follow nmea standard, which is 65-96
                else if (sv_info_ptr->system == RPC_LOC_SV_SYSTEM_GLONASS)
                {
                    if ((sv_info_ptr->valid_mask & RPC_LOC_SV_INFO_VALID_PROCESS_STATUS) &&
                        (sv_info_ptr->process_status == RPC_LOC_SV_STATUS_TRACK))
                    {
                        SvStatus.glo_used_in_fix_mask |= (1 << (sv_info_ptr->prn-1));
                    }

                    SvStatus.sv_list[SvStatus.num_svs].prn = sv_info_ptr->prn + (65-1);
                }
                // Unsupported SV system
                else
                {
                    continue;
                }
            }

            if (sv_info_ptr->valid_mask & RPC_LOC_SV_INFO_VALID_SNR)
            {
                SvStatus.sv_list[SvStatus.num_svs].snr = sv_info_ptr->snr;
            }

            if (sv_info_ptr->valid_mask & RPC_LOC_SV_INFO_VALID_ELEVATION)
            {
                SvStatus.sv_list[SvStatus.num_svs].elevation = sv_info_ptr->elevation;
            }

            if (sv_info_ptr->valid_mask & RPC_LOC_SV_INFO_VALID_AZIMUTH)
            {
                SvStatus.sv_list[SvStatus.num_svs].azimuth = sv_info_ptr->azimuth;
            }

            SvStatus.num_svs++;
        }
    }

    if ((gnss_report_ptr->valid_mask & RPC_LOC_GNSS_INFO_VALID_POS_DOP) &&
        (gnss_report_ptr->valid_mask & RPC_LOC_GNSS_INFO_VALID_HOR_DOP) &&
        (gnss_report_ptr->valid_mask & RPC_LOC_GNSS_INFO_VALID_VERT_DOP))
    {
        locationExtended.flags |= GPS_LOCATION_EXTENDED_HAS_DOP;
        locationExtended.pdop = gnss_report_ptr->position_dop;
        locationExtended.hdop = gnss_report_ptr->horizontal_dop;
        locationExtended.vdop = gnss_report_ptr->vertical_dop;
    }

    if (SvStatus.num_svs >= 0)
    {
        LocApiBase::reportSv(SvStatus,
                             locationExtended,
                             (void*)gnss_report_ptr);
    }
}

void LocApiRpc::reportStatus(const rpc_loc_status_event_s_type *status_report_ptr)
{

    if (status_report_ptr->event == RPC_LOC_STATUS_EVENT_ENGINE_STATE) {
        if (status_report_ptr->payload.rpc_loc_status_event_payload_u_type_u.engine_state == RPC_LOC_ENGINE_STATE_ON)
        {
            LocApiBase::reportStatus(GPS_STATUS_ENGINE_ON);
            LocApiBase::reportStatus(GPS_STATUS_SESSION_BEGIN);
        }
        else if (status_report_ptr->payload.rpc_loc_status_event_payload_u_type_u.engine_state == RPC_LOC_ENGINE_STATE_OFF)
        {
            LocApiBase::reportStatus(GPS_STATUS_SESSION_END);
            LocApiBase::reportStatus(GPS_STATUS_ENGINE_OFF);
        }
        else
        {
            LocApiBase::reportStatus(GPS_STATUS_NONE);
        }
    }

}

void LocApiRpc::reportNmea(const rpc_loc_nmea_report_s_type *nmea_report_ptr)
{

#if (AMSS_VERSION==3200)
    LocApiBase::reportNmea(nmea_report_ptr->nmea_sentences.nmea_sentences_val,
                           nmea_report_ptr->nmea_sentences.nmea_sentences_len);
#else
    LocApiBase::reportNmea(nmea_report_ptr->nmea_sentences,
                           nmea_report_ptr->length);
    LOC_LOGD("loc_eng_report_nmea: $%c%c%c\n",
             nmea_report_ptr->nmea_sentences[3],
             nmea_report_ptr->nmea_sentences[4],
             nmea_report_ptr->nmea_sentences[5]);
#endif /* #if (AMSS_VERSION==3200) */
}

enum loc_api_adapter_err
LocApiRpc::setXtraData(char* data, int length)
{
    int     rpc_ret_val = RPC_LOC_API_GENERAL_FAILURE;
    int     total_parts;
    uint8   part;
    uint16  part_len;
    uint16  len_injected;
    rpc_loc_ioctl_data_u_type            ioctl_data;
    rpc_loc_ioctl_e_type                 ioctl_type = RPC_LOC_IOCTL_INJECT_PREDICTED_ORBITS_DATA;
    rpc_loc_predicted_orbits_data_s_type *predicted_orbits_data_ptr;

    LOC_LOGD("qct_loc_eng_inject_xtra_data, xtra size = %d, data ptr = 0x%lx\n", length, (long) data);

    predicted_orbits_data_ptr = &ioctl_data.rpc_loc_ioctl_data_u_type_u.predicted_orbits_data;
    predicted_orbits_data_ptr->format_type = RPC_LOC_PREDICTED_ORBITS_XTRA;
    predicted_orbits_data_ptr->total_size = length;
    total_parts = (length - 1) / XTRA_BLOCK_SIZE + 1;
    predicted_orbits_data_ptr->total_parts = total_parts;

    len_injected = 0; // O bytes injected
    ioctl_data.disc = ioctl_type;

    // XTRA injection starts with part 1
    for (part = 1; part <= total_parts; part++)
    {
        predicted_orbits_data_ptr->part = part;
        predicted_orbits_data_ptr->part_len = XTRA_BLOCK_SIZE;
        if (XTRA_BLOCK_SIZE > (length - len_injected))
        {
            predicted_orbits_data_ptr->part_len = length - len_injected;
        }
        predicted_orbits_data_ptr->data_ptr.data_ptr_len = predicted_orbits_data_ptr->part_len;
        predicted_orbits_data_ptr->data_ptr.data_ptr_val = data + len_injected;

        LOC_LOGD("qct_loc_eng_inject_xtra_data, part %d/%d, len = %d, total = %d\n",
                 predicted_orbits_data_ptr->part,
                 total_parts,
                 predicted_orbits_data_ptr->part_len,
                 len_injected);

        if (part < total_parts)
        {
            // No callback in this case
            rpc_ret_val = loc_ioctl (client_handle,
                                     ioctl_type,
                                     &ioctl_data);

            if (rpc_ret_val != RPC_LOC_API_SUCCESS)
            {
                LOC_LOGE("loc_ioctl for xtra error: %s\n", loc_get_ioctl_status_name(rpc_ret_val));
                break;
            }
            //Add a delay of 10 ms so that repeated RPC calls dont starve the modem processor
            usleep(10 * 1000);
        }
        else // part == total_parts
        {
            // Last part injection, will need to wait for callback
            if (!loc_eng_ioctl(client_handle,
                               ioctl_type,
                               &ioctl_data,
                               LOC_XTRA_INJECT_DEFAULT_TIMEOUT,
                               NULL))
            {
                rpc_ret_val = RPC_LOC_API_GENERAL_FAILURE;
            }
            break; // done with injection
        }

        len_injected += predicted_orbits_data_ptr->part_len;
        LOC_LOGD("loc_ioctl XTRA injected length: %d\n", len_injected);
    }

    return convertErr(rpc_ret_val);
}

/* Request the Xtra Server Url from the modem */
enum loc_api_adapter_err
LocApiRpc::requestXtraServer()
{
    loc_api_adapter_err           err;
    rpc_loc_ioctl_data_u_type     data;
    rpc_loc_ioctl_callback_s_type callback_data;

    err = convertErr(loc_eng_ioctl(client_handle,
                                   RPC_LOC_IOCTL_QUERY_PREDICTED_ORBITS_DATA_SOURCE,
                                   &data,
                                   LOC_IOCTL_DEFAULT_TIMEOUT,
                                   &callback_data));

    if (LOC_API_ADAPTER_ERR_SUCCESS != err)
    {
        LOC_LOGE("RPC_LOC_IOCTL_QUERY_PREDICTED_ORBITS_DATA_SOURCE failed!: err=%d\n", err);
        return err;
    }
    else if (RPC_LOC_SESS_STATUS_SUCCESS != callback_data.status)
    {
        LOC_LOGE("RPC_LOC_IOCTL_QUERY_PREDICTED_ORBITS_DATA_SOURCE failed!: status=%ld\n", callback_data.status);
        return LOC_API_ADAPTER_ERR_GENERAL_FAILURE;
    }
    else if (RPC_LOC_IOCTL_QUERY_PREDICTED_ORBITS_DATA_SOURCE != callback_data.type)
    {
        LOC_LOGE("RPC_LOC_IOCTL_QUERY_PREDICTED_ORBITS_DATA_SOURCE is not the type expected! type=%d\n", callback_data.type);
        return LOC_API_ADAPTER_ERR_GENERAL_FAILURE;
    }
    else if (RPC_LOC_IOCTL_QUERY_PREDICTED_ORBITS_DATA_SOURCE != callback_data.data.disc)
    {
        LOC_LOGE("RPC_LOC_IOCTL_QUERY_PREDICTED_ORBITS_DATA_SOURCE is not the disc expected! disc=%d\n", callback_data.data.disc);
        return LOC_API_ADAPTER_ERR_GENERAL_FAILURE;
    }

    reportXtraServer(callback_data.data.rpc_loc_ioctl_callback_data_u_type_u.
                     predicted_orbits_data_source.servers[0],
                     callback_data.data.rpc_loc_ioctl_callback_data_u_type_u.
                     predicted_orbits_data_source.servers[1],
                     callback_data.data.rpc_loc_ioctl_callback_data_u_type_u.
                     predicted_orbits_data_source.servers[2],
                     255);

    return LOC_API_ADAPTER_ERR_SUCCESS;
}

enum loc_api_adapter_err
LocApiRpc::atlOpenStatus(int handle, int is_succ, char* apn, AGpsBearerType bearer, AGpsType agpsType)
{
    rpc_loc_server_open_status_e_type open_status = is_succ ? RPC_LOC_SERVER_OPEN_SUCCESS : RPC_LOC_SERVER_OPEN_FAIL;
   rpc_loc_ioctl_data_u_type           ioctl_data;

    if (AGPS_TYPE_INVALID == agpsType) {
        rpc_loc_server_open_status_s_type  *conn_open_status_ptr =
            &ioctl_data.rpc_loc_ioctl_data_u_type_u.conn_open_status;

        // Fill in data
        ioctl_data.disc = RPC_LOC_IOCTL_INFORM_SERVER_OPEN_STATUS;
        conn_open_status_ptr->conn_handle = handle;
        conn_open_status_ptr->open_status = open_status;
#if (AMSS_VERSION==3200)
        conn_open_status_ptr->apn_name = apn; /* requires APN */
#else
        if (is_succ) {
            strlcpy(conn_open_status_ptr->apn_name, apn,
                    sizeof conn_open_status_ptr->apn_name);
        } else {
            conn_open_status_ptr->apn_name[0] = 0;
        }
#endif /* #if (AMSS_VERSION==3200) */

        LOC_LOGD("ATL RPC_LOC_IOCTL_INFORM_SERVER_OPEN_STATUS open %s, APN name = [%s]\n",
                 log_succ_fail_string(is_succ),
                 apn);
    } else {
        rpc_loc_server_multi_open_status_s_type  *conn_multi_open_status_ptr =
            &ioctl_data.rpc_loc_ioctl_data_u_type_u.multi_conn_open_status;

        // Fill in data
        ioctl_data.disc = RPC_LOC_IOCTL_INFORM_SERVER_MULTI_OPEN_STATUS;
        conn_multi_open_status_ptr->conn_handle = handle;
        conn_multi_open_status_ptr->open_status = open_status;
        if (is_succ) {
            strlcpy(conn_multi_open_status_ptr->apn_name, apn,
                    sizeof conn_multi_open_status_ptr->apn_name);
        } else {
            conn_multi_open_status_ptr->apn_name[0] = 0;
        }

        switch(bearer)
        {
        case AGPS_APN_BEARER_IPV4:
            conn_multi_open_status_ptr->pdp_type = RPC_LOC_SERVER_PDP_IP;
            break;
        case AGPS_APN_BEARER_IPV6:
            conn_multi_open_status_ptr->pdp_type = RPC_LOC_SERVER_PDP_IPV6;
            break;
        case AGPS_APN_BEARER_IPV4V6:
            conn_multi_open_status_ptr->pdp_type = RPC_LOC_SERVER_PDP_IPV4V6;
            break;
        default:
            conn_multi_open_status_ptr->pdp_type = RPC_LOC_SERVER_PDP_PPP;
        }

        LOC_LOGD("ATL RPC_LOC_IOCTL_INFORM_SERVER_MULTI_OPEN_STATUS open %s, APN name = [%s], pdp_type = %d\n",
                 log_succ_fail_string(is_succ),
                 apn,
                 conn_multi_open_status_ptr->pdp_type);
    }

    // Make the IOCTL call
    return convertErr(
        loc_eng_ioctl(client_handle,
                      ioctl_data.disc,
                      &ioctl_data,
                      LOC_IOCTL_DEFAULT_TIMEOUT,
                      NULL)
        );
}

enum loc_api_adapter_err
LocApiRpc::atlCloseStatus(int handle, int is_succ)
{
    rpc_loc_ioctl_data_u_type           ioctl_data;
    ioctl_data.disc = RPC_LOC_IOCTL_INFORM_SERVER_CLOSE_STATUS;

    rpc_loc_server_close_status_s_type *conn_close_status_ptr =
        &ioctl_data.rpc_loc_ioctl_data_u_type_u.conn_close_status;
    conn_close_status_ptr->conn_handle = handle;
    conn_close_status_ptr->close_status = is_succ ? RPC_LOC_SERVER_CLOSE_SUCCESS : RPC_LOC_SERVER_CLOSE_FAIL;

    // Make the IOCTL call
    return convertErr(
        loc_eng_ioctl(client_handle,
                      ioctl_data.disc,
                      &ioctl_data,
                      LOC_IOCTL_DEFAULT_TIMEOUT,
                      NULL)
        );
}

void LocApiRpc::ATLEvent(const rpc_loc_server_request_s_type *server_request_ptr)
{
    int connHandle;
    AGpsType agps_type;

    LOC_LOGV("RPC_LOC_EVENT_ASSISTANCE_DATA_REQUEST event %s)",
             loc_get_event_atl_open_name(server_request_ptr->event));
    switch (server_request_ptr->event)
    {
    case RPC_LOC_SERVER_REQUEST_MULTI_OPEN:
        connHandle = server_request_ptr->payload.rpc_loc_server_request_u_type_u.multi_open_req.conn_handle;
        if (server_request_ptr->payload.rpc_loc_server_request_u_type_u.multi_open_req.connection_type
            == RPC_LOC_SERVER_CONNECTION_LBS) {
            agps_type = AGPS_TYPE_SUPL;
            LOC_LOGV("ATLEvent: event - RPC_LOC_SERVER_REQUEST_MULTI_OPEN\n            type - AGPS_TYPE_SUPL\n            handle - %d", connHandle);
        } else {
            agps_type = AGPS_TYPE_WWAN_ANY;
            LOC_LOGV("ATLEvent: event - RPC_LOC_SERVER_REQUEST_MULTI_OPEN\n            type - AGPS_TYPE_WWAN_ANY\n            handle - %d", connHandle);
        }
        requestATL(connHandle, agps_type);
        break;
    case RPC_LOC_SERVER_REQUEST_OPEN:
        connHandle = server_request_ptr->payload.rpc_loc_server_request_u_type_u.open_req.conn_handle;
        LOC_LOGV("ATLEvent: event - RPC_LOC_SERVER_REQUEST_OPEN\n            handle - %d", connHandle);
        requestATL(connHandle, AGPS_TYPE_INVALID);
        break;
    case RPC_LOC_SERVER_REQUEST_CLOSE:
        connHandle = server_request_ptr->payload.rpc_loc_server_request_u_type_u.close_req.conn_handle;
        LOC_LOGV("ATLEvent: event - RPC_LOC_SERVER_REQUEST_CLOSE\n            handle - %d", connHandle);
        releaseATL(connHandle);
        break;
    default:
        LOC_LOGE("ATLEvent: event type %d invalid", server_request_ptr->event);
   }
}

void LocApiRpc::NIEvent(const rpc_loc_ni_event_s_type *ni_req)
{
    GpsNiNotification notif = {0};

    switch (ni_req->event)
    {
    case RPC_LOC_NI_EVENT_VX_NOTIFY_VERIFY_REQ:
    {
        const rpc_loc_ni_vx_notify_verify_req_s_type *vx_req =
            &ni_req->payload.rpc_loc_ni_event_payload_u_type_u.vx_req;
        LOC_LOGI("VX Notification");
        notif.ni_type = GPS_NI_TYPE_VOICE;
        // Requestor ID
        hexcode(notif.requestor_id, sizeof notif.requestor_id,
                vx_req->requester_id.requester_id,
                vx_req->requester_id.requester_id_length);
        notif.text_encoding = 0; // No text and no encoding
        notif.requestor_id_encoding = convertNiEncodingType(vx_req->encoding_scheme);
        NIEventFillVerfiyType(notif, vx_req->notification_priv_type);
    }
        break;

    case RPC_LOC_NI_EVENT_UMTS_CP_NOTIFY_VERIFY_REQ:
    {
        const rpc_loc_ni_umts_cp_notify_verify_req_s_type *umts_cp_req =
            &ni_req->payload.rpc_loc_ni_event_payload_u_type_u.umts_cp_req;
        LOC_LOGI("UMTS CP Notification\n");
        notif.ni_type= GPS_NI_TYPE_UMTS_CTRL_PLANE;         // Stores notification text
#if (AMSS_VERSION==3200)
        hexcode(notif.text, sizeof notif.text,
                umts_cp_req->notification_text.notification_text_val,
                umts_cp_req->notification_length);
        hexcode(notif.requestor_id, sizeof notif.requestor_id,
                umts_cp_req->requestor_id.requestor_id_string.requestor_id_string_val,
                umts_cp_req->requestor_id.string_len);
#else
        hexcode(notif.text, sizeof notif.text,
                umts_cp_req->notification_text,
                umts_cp_req->notification_length);
        hexcode(notif.requestor_id, sizeof notif.requestor_id,
                umts_cp_req->requestor_id.requestor_id_string,
                umts_cp_req->requestor_id.string_len);
#endif
        notif.text_encoding = convertNiEncodingType(umts_cp_req->datacoding_scheme);
        notif.requestor_id_encoding = notif.text_encoding;
        NIEventFillVerfiyType(notif, umts_cp_req->notification_priv_type);

        // LCS address (using extras field)
        if (umts_cp_req->ext_client_address_data.ext_client_address_len != 0)
        {
            // Copy LCS Address into notif.extras in the format: Address = 012345
            strlcat(notif.extras, LOC_NI_NOTIF_KEY_ADDRESS, sizeof notif.extras);
            strlcat(notif.extras, " = ", sizeof notif.extras);
            int addr_len = 0;
            const char *address_source = NULL;

#if (AMSS_VERSION==3200)
            address_source = umts_cp_req->ext_client_address_data.ext_client_address.ext_client_address_val;
#else
            address_source = umts_cp_req->ext_client_address_data.ext_client_address;
#endif /* #if (AMSS_VERSION==3200) */

            char lcs_addr[32]; // Decoded LCS address for UMTS CP NI
            addr_len = decodeAddress(lcs_addr, sizeof lcs_addr, address_source,
                                     umts_cp_req->ext_client_address_data.ext_client_address_len);

            // The address is ASCII string
            if (addr_len)
            {
                strlcat(notif.extras, lcs_addr, sizeof notif.extras);
            }
        }
    }
        break;

    case RPC_LOC_NI_EVENT_SUPL_NOTIFY_VERIFY_REQ:
    {
        const rpc_loc_ni_supl_notify_verify_req_s_type *supl_req =
            &ni_req->payload.rpc_loc_ni_event_payload_u_type_u.supl_req;
        LOC_LOGI("SUPL Notification\n");
        notif.ni_type = GPS_NI_TYPE_UMTS_SUPL;

        if (supl_req->flags & RPC_LOC_NI_CLIENT_NAME_PRESENT)
        {
#if (AMSS_VERSION==3200)
            hexcode(notif.text, sizeof notif.text,
                    supl_req->client_name.client_name_string.client_name_string_val,   /* buffer */
                    supl_req->client_name.string_len                                   /* length */
            );
#else
            hexcode(notif.text, sizeof notif.text,
                            supl_req->client_name.client_name_string,   /* buffer */
                            supl_req->client_name.string_len            /* length */
            );
#endif /* #if (AMSS_VERSION==3200) */
            LOC_LOGV("SUPL NI: client_name: %s len=%d", notif.text, supl_req->client_name.string_len);
        }
        else {
            LOC_LOGV("SUPL NI: client_name not present.");
        }

        // Requestor ID
        if (supl_req->flags & RPC_LOC_NI_REQUESTOR_ID_PRESENT)
        {
#if (AMSS_VERSION==3200)
            hexcode(notif.requestor_id, sizeof notif.requestor_id,
                    supl_req->requestor_id.requestor_id_string.requestor_id_string_val,  /* buffer */
                    supl_req->requestor_id.string_len                                    /* length */
                );
#else
            hexcode(notif.requestor_id, sizeof notif.requestor_id,
                    supl_req->requestor_id.requestor_id_string,  /* buffer */
                    supl_req->requestor_id.string_len            /* length */
                );
#endif /* #if (AMSS_VERSION==3200) */
            LOC_LOGV("SUPL NI: requestor_id: %s len=%d", notif.requestor_id, supl_req->requestor_id.string_len);
        }
        else {
            LOC_LOGV("SUPL NI: requestor_id not present.");
        }

        // Encoding type
        if (supl_req->flags & RPC_LOC_NI_ENCODING_TYPE_PRESENT)
        {
            notif.text_encoding = convertNiEncodingType(supl_req->datacoding_scheme);
            notif.requestor_id_encoding = notif.text_encoding;
        }
        else {
            notif.text_encoding = notif.requestor_id_encoding = GPS_ENC_UNKNOWN;
        }

        NIEventFillVerfiyType(notif, ni_req->payload.rpc_loc_ni_event_payload_u_type_u.supl_req.notification_priv_type);
    }
        break;

    default:
        LOC_LOGE("Unknown NI event: %x\n", (int) ni_req->event);
        return;
    }

    // this copy will get freed in loc_eng_ni when loc_ni_respond() is called
    rpc_loc_ni_event_s_type *copy = (rpc_loc_ni_event_s_type *)malloc(sizeof(*copy));
    memcpy(copy, ni_req, sizeof(*copy));
    requestNiNotify(notif, (const void*)copy);
}

int LocApiRpc::NIEventFillVerfiyType(GpsNiNotification &notif,
                                rpc_loc_ni_notify_verify_e_type notif_priv)
{
   switch (notif_priv)
   {
   case RPC_LOC_NI_USER_NO_NOTIFY_NO_VERIFY:
       notif.notify_flags = 0;
       notif.default_response = GPS_NI_RESPONSE_NORESP;
       return 1;
   case RPC_LOC_NI_USER_NOTIFY_ONLY:
       notif.notify_flags = GPS_NI_NEED_NOTIFY;
       notif.default_response = GPS_NI_RESPONSE_NORESP;
       return 1;
   case RPC_LOC_NI_USER_NOTIFY_VERIFY_ALLOW_NO_RESP:
       notif.notify_flags = GPS_NI_NEED_NOTIFY | GPS_NI_NEED_VERIFY;
       notif.default_response = GPS_NI_RESPONSE_ACCEPT;
       return 1;
   case RPC_LOC_NI_USER_NOTIFY_VERIFY_NOT_ALLOW_NO_RESP:
       notif.notify_flags = GPS_NI_NEED_NOTIFY | GPS_NI_NEED_VERIFY;
       notif.default_response = GPS_NI_RESPONSE_DENY;
       return 1;
   case RPC_LOC_NI_USER_PRIVACY_OVERRIDE:
       notif.notify_flags = GPS_NI_PRIVACY_OVERRIDE;
       notif.default_response = GPS_NI_RESPONSE_NORESP;
       return 1;
   default:
      return 0;
   }
}

enum loc_api_adapter_err
LocApiRpc::setSUPLVersion(uint32_t version)
{
   rpc_loc_ioctl_data_u_type ioctl_data = {RPC_LOC_IOCTL_SET_SUPL_VERSION, {0}};
   ioctl_data.rpc_loc_ioctl_data_u_type_u.supl_version = (int)version;
   return convertErr(
       loc_eng_ioctl (client_handle,
                      RPC_LOC_IOCTL_SET_SUPL_VERSION,
                      &ioctl_data,
                      LOC_IOCTL_DEFAULT_TIMEOUT,
                      NULL)
       );
}

GpsNiEncodingType LocApiRpc::convertNiEncodingType(int loc_encoding)
{
   switch (loc_encoding)
   {
   case RPC_LOC_NI_SUPL_UTF8:
       return GPS_ENC_SUPL_UTF8;
   case RPC_LOC_NI_SUPL_UCS2:
       return GPS_ENC_SUPL_UCS2;
   case RPC_LOC_NI_SUPL_GSM_DEFAULT:
      return GPS_ENC_SUPL_GSM_DEFAULT;
   case RPC_LOC_NI_SS_LANGUAGE_UNSPEC:
      return GPS_ENC_SUPL_GSM_DEFAULT; // SS_LANGUAGE_UNSPEC = GSM
   default:
       return GPS_ENC_UNKNOWN;
   }
}

LocApiBase* getLocApi(const MsgTask* msgTask,
                      LOC_API_ADAPTER_EVENT_MASK_T exMask,
                      ContextBase *context) {
    return new LocApiRpc(msgTask, exMask, context);
}

/*Values for lock
  1 = Do not lock any position sessions
  2 = Lock MI position sessions
  3 = Lock MT position sessions
  4 = Lock all position sessions
*/
int LocApiRpc::setGpsLock(LOC_GPS_LOCK_MASK lockMask)
{
    rpc_loc_ioctl_data_u_type    ioctl_data;
    boolean ret_val;
    LOC_LOGD("%s:%d]: lock: %x\n", __func__, __LINE__, lockMask);
    ioctl_data.rpc_loc_ioctl_data_u_type_u.engine_lock = convertGpsLockMask(lockMask);
    ioctl_data.disc = RPC_LOC_IOCTL_SET_ENGINE_LOCK;
    ret_val = loc_eng_ioctl (loc_eng_data.client_handle,
                            RPC_LOC_IOCTL_SET_ENGINE_LOCK,
                            &ioctl_data,
                            LOC_IOCTL_DEFAULT_TIMEOUT,
                            NULL /* No output information is expected*/);

    LOC_LOGD("%s:%d]: ret_val: %d\n", __func__, __LINE__, (int)ret_val);
    return (ret_val == TRUE ? 0 : -1);
}

/*
  Returns
  Current value of GPS lock on success
  -1 on failure
*/
int LocApiRpc :: getGpsLock()
{
    rpc_loc_ioctl_data_u_type    ioctl_data;
    rpc_loc_ioctl_callback_s_type callback_payload;
    boolean ret_val;
    int ret=0;
    LOC_LOGD("%s:%d]: Enter\n", __func__, __LINE__);
    ret_val = loc_eng_ioctl (loc_eng_data.client_handle,
                            RPC_LOC_IOCTL_GET_ENGINE_LOCK,
                            &ioctl_data,
                            LOC_IOCTL_DEFAULT_TIMEOUT,
                            &callback_payload);
    if(ret_val == TRUE) {
        ret = (int)callback_payload.data.engine_lock;
        LOC_LOGD("%s:%d]: Lock type: %d\n", __func__, __LINE__, ret);
    }
    else {
        LOC_LOGE("%s:%d]: Ioctl failed", __func__, __LINE__);
        ret = -1;
    }
    LOC_LOGD("%s:%d]: Exit\n", __func__, __LINE__);
    return ret;
}
