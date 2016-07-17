/* Copyright (c) 2011 The Linux Foundation. All rights reserved.
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

#define LOG_NDDEBUG 0
#define LOG_TAG "LocSvc_api_rpc_glue"

#include "loc_api_log.h"
#include "loc_log.h"
#include "log_util.h"
#include "platform_lib_includes.h"
#include "rpc/rpc.h"
#include "loc_api_fixup.h"

/* Event names */
loc_name_val_s_type loc_event_name[] =
   {
      NAME_VAL( RPC_LOC_EVENT_PARSED_POSITION_REPORT ),
      NAME_VAL( RPC_LOC_EVENT_SATELLITE_REPORT ),
      NAME_VAL( RPC_LOC_EVENT_NMEA_1HZ_REPORT ),
      NAME_VAL( RPC_LOC_EVENT_NMEA_POSITION_REPORT ),
      NAME_VAL( RPC_LOC_EVENT_NI_NOTIFY_VERIFY_REQUEST ),
      NAME_VAL( RPC_LOC_EVENT_ASSISTANCE_DATA_REQUEST ),
      NAME_VAL( RPC_LOC_EVENT_LOCATION_SERVER_REQUEST ),
      NAME_VAL( RPC_LOC_EVENT_IOCTL_REPORT ),
      NAME_VAL( RPC_LOC_EVENT_STATUS_REPORT ),
      NAME_VAL( RPC_LOC_EVENT_WPS_NEEDED_REQUEST ),
   };
int loc_event_num = sizeof loc_event_name / sizeof(loc_name_val_s_type);

/* Event names */
loc_name_val_s_type loc_event_atl_open_name[] =
   {
      NAME_VAL( RPC_LOC_SERVER_REQUEST_OPEN ),
      NAME_VAL( RPC_LOC_SERVER_REQUEST_CLOSE ),
      NAME_VAL( RPC_LOC_SERVER_REQUEST_MULTI_OPEN )
   };
int loc_event_atl_open_num = sizeof loc_event_atl_open_name / sizeof(loc_name_val_s_type);

/* Finds the first event found in the mask */
const char* loc_get_event_atl_open_name(rpc_loc_server_request_e_type loc_event_atl_open)
{
   return loc_get_name_from_val(loc_event_atl_open_name, loc_event_atl_open_num,
         (long) loc_event_atl_open);
}

/* IOCTL Type names */
loc_name_val_s_type loc_ioctl_type_name[] =
   {
      NAME_VAL( RPC_LOC_IOCTL_GET_API_VERSION ),
      NAME_VAL( RPC_LOC_IOCTL_SET_FIX_CRITERIA ),
      NAME_VAL( RPC_LOC_IOCTL_GET_FIX_CRITERIA ),
      NAME_VAL( RPC_LOC_IOCTL_INFORM_NI_USER_RESPONSE ),
      NAME_VAL( RPC_LOC_IOCTL_INJECT_PREDICTED_ORBITS_DATA ),
      NAME_VAL( RPC_LOC_IOCTL_QUERY_PREDICTED_ORBITS_DATA_VALIDITY ),
      NAME_VAL( RPC_LOC_IOCTL_QUERY_PREDICTED_ORBITS_DATA_SOURCE ),
      NAME_VAL( RPC_LOC_IOCTL_SET_PREDICTED_ORBITS_DATA_AUTO_DOWNLOAD ),
      NAME_VAL( RPC_LOC_IOCTL_INJECT_UTC_TIME ),
      NAME_VAL( RPC_LOC_IOCTL_INJECT_RTC_VALUE ),
      NAME_VAL( RPC_LOC_IOCTL_INJECT_POSITION ),
      NAME_VAL( RPC_LOC_IOCTL_QUERY_ENGINE_STATE ),
      NAME_VAL( RPC_LOC_IOCTL_ERROR_ESTIMATE_CONFIG),
      NAME_VAL( RPC_LOC_IOCTL_INFORM_SERVER_MULTI_OPEN_STATUS ),
      NAME_VAL( RPC_LOC_IOCTL_INFORM_SERVER_OPEN_STATUS ),
      NAME_VAL( RPC_LOC_IOCTL_INFORM_SERVER_CLOSE_STATUS ),
      NAME_VAL( RPC_LOC_IOCTL_SEND_WIPER_POSITION_REPORT ),
      NAME_VAL( RPC_LOC_IOCTL_NOTIFY_WIPER_STATUS ),
      NAME_VAL( RPC_LOC_IOCTL_SET_ENGINE_LOCK ),
      NAME_VAL( RPC_LOC_IOCTL_GET_ENGINE_LOCK ),
      NAME_VAL( RPC_LOC_IOCTL_SET_SBAS_CONFIG ),
      NAME_VAL( RPC_LOC_IOCTL_GET_SBAS_CONFIG ),
      NAME_VAL( RPC_LOC_IOCTL_SET_NMEA_TYPES ),
      NAME_VAL( RPC_LOC_IOCTL_GET_NMEA_TYPES ),
      NAME_VAL( RPC_LOC_IOCTL_SET_CDMA_PDE_SERVER_ADDR ),
      NAME_VAL( RPC_LOC_IOCTL_GET_CDMA_PDE_SERVER_ADDR ),
      NAME_VAL( RPC_LOC_IOCTL_SET_CDMA_MPC_SERVER_ADDR ),
      NAME_VAL( RPC_LOC_IOCTL_GET_CDMA_MPC_SERVER_ADDR ),
      NAME_VAL( RPC_LOC_IOCTL_SET_UMTS_SLP_SERVER_ADDR ),
      NAME_VAL( RPC_LOC_IOCTL_GET_UMTS_SLP_SERVER_ADDR ),
      NAME_VAL( RPC_LOC_IOCTL_SET_ON_DEMAND_LPM ),
      NAME_VAL( RPC_LOC_IOCTL_GET_ON_DEMAND_LPM ),
      NAME_VAL( RPC_LOC_IOCTL_SET_XTRA_T_SESSION_CONTROL ),
      NAME_VAL( RPC_LOC_IOCTL_GET_XTRA_T_SESSION_CONTROL ),
      NAME_VAL( RPC_LOC_IOCTL_SET_LBS_APN_PROFILE ),
      NAME_VAL( RPC_LOC_IOCTL_GET_LBS_APN_PROFILE ),
      NAME_VAL( RPC_LOC_IOCTL_SET_XTRA_APN_PROFILE ),
      NAME_VAL( RPC_LOC_IOCTL_GET_XTRA_APN_PROFILE ),
      NAME_VAL( RPC_LOC_IOCTL_SET_DATA_ENABLE ),
      NAME_VAL( RPC_LOC_IOCTL_SET_SUPL_VERSION ),
      NAME_VAL( RPC_LOC_IOCTL_GET_SUPL_VERSION ),
      NAME_VAL( RPC_LOC_IOCTL_DELETE_ASSIST_DATA ),
      NAME_VAL( RPC_LOC_IOCTL_SET_CUSTOM_PDE_SERVER_ADDR ),
      NAME_VAL( RPC_LOC_IOCTL_GET_CUSTOM_PDE_SERVER_ADDR ),
   };
int loc_ioctl_type_num = sizeof loc_ioctl_type_name / sizeof(loc_name_val_s_type);

/* IOCTL Status names */
loc_name_val_s_type loc_ioctl_status_name[] =
   {
      NAME_VAL( RPC_LOC_API_SUCCESS ),
      NAME_VAL( RPC_LOC_API_GENERAL_FAILURE ),
      NAME_VAL( RPC_LOC_API_UNSUPPORTED ),
      NAME_VAL( RPC_LOC_API_INVALID_HANDLE ),
      NAME_VAL( RPC_LOC_API_INVALID_PARAMETER ),
      NAME_VAL( RPC_LOC_API_ENGINE_BUSY ),
      NAME_VAL( RPC_LOC_API_PHONE_OFFLINE ),
      NAME_VAL( RPC_LOC_API_TIMEOUT ),
      NAME_VAL( RPC_LOC_API_RPC_FAILURE ),
      NAME_VAL( RPC_LOC_API_RPC_MODEM_RESTART )
   };
int loc_ioctl_status_num = sizeof loc_ioctl_status_name / sizeof(loc_name_val_s_type);

/* Fix session status names */
loc_name_val_s_type loc_sess_status_name[] =
   {
      NAME_VAL( RPC_LOC_SESS_STATUS_SUCCESS ),
      NAME_VAL( RPC_LOC_SESS_STATUS_IN_PROGESS ),
      NAME_VAL( RPC_LOC_SESS_STATUS_GENERAL_FAILURE ),
      NAME_VAL( RPC_LOC_SESS_STATUS_TIMEOUT ),
      NAME_VAL( RPC_LOC_SESS_STATUS_USER_END ),
      NAME_VAL( RPC_LOC_SESS_STATUS_BAD_PARAMETER ),
      NAME_VAL( RPC_LOC_SESS_STATUS_PHONE_OFFLINE ),
      NAME_VAL( RPC_LOC_SESS_STATUS_USER_END ),
      NAME_VAL( RPC_LOC_SESS_STATUS_ENGINE_LOCKED )
   };
int loc_sess_status_num = sizeof loc_sess_status_name / sizeof(loc_name_val_s_type);

/* Engine state names */
loc_name_val_s_type loc_engine_state_name[] =
   {
      NAME_VAL( RPC_LOC_ENGINE_STATE_ON ),
      NAME_VAL( RPC_LOC_ENGINE_STATE_OFF )
   };
int loc_engine_state_num = sizeof loc_engine_state_name / sizeof(loc_name_val_s_type);

/* Fix session state names */
loc_name_val_s_type loc_fix_session_state_name[] =
   {
      NAME_VAL( RPC_LOC_FIX_SESSION_STATE_BEGIN ),
      NAME_VAL( RPC_LOC_FIX_SESSION_STATE_END )
   };
int loc_fix_session_state_num = sizeof loc_fix_session_state_name / sizeof(loc_name_val_s_type);


static const char* log_final_interm_string(int is_final)
{
   return is_final ? "final" : "intermediate";
}

/* Logs parsed report */
static void log_parsed_report(const rpc_loc_parsed_position_s_type *parsed_report)
{
   rpc_loc_session_status_e_type status = parsed_report->session_status;
   LOC_LOGD("Session status: %s   Valid mask: 0x%X\n",
         loc_get_sess_status_name(status),
         (uint) parsed_report->valid_mask);
   LOC_LOGD("Latitude:  %.7f (%s)\n", parsed_report->latitude,
         log_final_interm_string(
               (parsed_report->valid_mask & RPC_LOC_POS_VALID_LATITUDE) &&
               parsed_report->session_status == RPC_LOC_SESS_STATUS_SUCCESS));
   LOC_LOGD("Longitude: %.7f\n", parsed_report->longitude);
   LOC_LOGD("Accuracy: %.7f\n", parsed_report->hor_unc_circular);
}

/* Logs status report */
static void log_status_report(const rpc_loc_status_event_s_type *status_event)
{
   rpc_loc_status_event_e_type event = status_event->event;
   switch (event) {
   case RPC_LOC_STATUS_EVENT_ENGINE_STATE:
      LOC_LOGD("Engine state: %s\n",
            loc_get_engine_state_name(
                  status_event->payload.rpc_loc_status_event_payload_u_type_u.engine_state));
      break;
   case RPC_LOC_STATUS_EVENT_FIX_SESSION_STATE:
      LOC_LOGD("Fix session state: %s\n",
            loc_get_fix_session_state_name(
                  status_event->payload.rpc_loc_status_event_payload_u_type_u.fix_session_state));
      break;
   default:
      break;
   }
}

/* Logs valid fields in the GNSS SV constellation report */
static void log_satellite_report(const rpc_loc_gnss_info_s_type *gnss)
{
   if (gnss->valid_mask & RPC_LOC_GNSS_INFO_VALID_POS_DOP)
   {
      LOC_LOGV("position dop: %.3f\n", (float) gnss->position_dop);
   }
   if (gnss->valid_mask & RPC_LOC_GNSS_INFO_VALID_HOR_DOP)
   {
      LOC_LOGV("horizontal dop: %.3f\n", (float) gnss->horizontal_dop);
   }
   if (gnss->valid_mask & RPC_LOC_GNSS_INFO_VALID_VERT_DOP)
   {
      LOC_LOGV("vertical dop: %.3f\n", (float) gnss->vertical_dop);
   }
   if (gnss->valid_mask & RPC_LOC_GNSS_INFO_VALID_ALTITUDE_ASSUMED)
   {
      LOC_LOGV("altitude assumed: %d\n", (int) gnss->altitude_assumed);
   }
   if (gnss->valid_mask & RPC_LOC_GNSS_INFO_VALID_SV_COUNT)
   {
      LOC_LOGD("sv count: %d\n", (int) gnss->sv_count);
   }
   if (gnss->valid_mask & RPC_LOC_GNSS_INFO_VALID_SV_LIST)
   {
      LOC_LOGV("sv list: ");

      if (gnss->sv_count)
      {
         LOC_LOGV("\n\tsys\tprn\thlth\tproc\teph\talm\telev\tazi\tsnr\n");
      }
      else {
         LOC_LOGV("empty\n");
      }

      int i;
      for (i = 0; i < gnss->sv_count; i++)
      {
         const rpc_loc_sv_info_s_type *sv = &gnss->sv_list.sv_list_val[i];
         rpc_loc_sv_info_valid_mask_type mask = sv->valid_mask;
         LOC_LOGV("  %d: \t%d\t%d\t%d\t%d\t%d\t%d\t%.3f\t%.3f\t%.3f\n", i,
               CHECK_MASK(int,   sv->system,         mask, RPC_LOC_SV_INFO_VALID_SYSTEM),
               CHECK_MASK(int,   sv->prn,            mask, RPC_LOC_SV_INFO_VALID_PRN),
               CHECK_MASK(int,   sv->health_status,  mask, RPC_LOC_SV_INFO_VALID_HEALTH_STATUS),
               CHECK_MASK(int,   sv->process_status, mask, RPC_LOC_SV_INFO_VALID_PROCESS_STATUS),
               CHECK_MASK(int,   sv->has_eph,        mask, RPC_LOC_SV_INFO_VALID_HAS_EPH),
               CHECK_MASK(int,   sv->has_alm,        mask, RPC_LOC_SV_INFO_VALID_HAS_ALM),
               CHECK_MASK(float, sv->elevation,      mask, RPC_LOC_SV_INFO_VALID_ELEVATION),
               CHECK_MASK(float, sv->azimuth,        mask, RPC_LOC_SV_INFO_VALID_AZIMUTH),
               CHECK_MASK(float, sv->snr,            mask, RPC_LOC_SV_INFO_VALID_SNR)
         );
      }
   }
}

/* Logs a callback event */
int loc_callback_log(
      rpc_loc_event_mask_type               loc_event,              /* event mask           */
      const rpc_loc_event_payload_u_type*   loc_event_payload       /* payload              */
)
{
   switch (loc_event)
   {
   case RPC_LOC_EVENT_SATELLITE_REPORT:
      log_satellite_report(&loc_event_payload->
            rpc_loc_event_payload_u_type_u.gnss_report);
      break;
   case RPC_LOC_EVENT_STATUS_REPORT:
      log_status_report(&loc_event_payload->
            rpc_loc_event_payload_u_type_u.status_report);
      break;
   case RPC_LOC_EVENT_PARSED_POSITION_REPORT:
      log_parsed_report(&loc_event_payload->
            rpc_loc_event_payload_u_type_u.parsed_location_report);
      break;
   default:
      break;
   }

   return 0;
}

/* Finds the first event found in the mask */
const char* loc_get_event_name(rpc_loc_event_mask_type loc_event_mask)
{
   return loc_get_name_from_mask(loc_event_name, loc_event_num,
         (long) loc_event_mask);
}

/* Finds IOCTL type name */
const char* loc_get_ioctl_type_name(rpc_loc_ioctl_e_type ioctl_type)
{
   return loc_get_name_from_val(loc_ioctl_type_name, loc_ioctl_type_num,
         (long) ioctl_type);
}

/* Finds IOCTL status name */
const char* loc_get_ioctl_status_name(uint32 status)
{
   return loc_get_name_from_val(loc_ioctl_status_name, loc_ioctl_status_num,
         (long) status);
}

/* Finds session status name */
const char* loc_get_sess_status_name(rpc_loc_session_status_e_type status)
{
   return loc_get_name_from_val(loc_sess_status_name, loc_sess_status_num,
         (long) status);
}

/* Find engine state name */
const char* loc_get_engine_state_name(rpc_loc_engine_state_e_type state)
{
   return loc_get_name_from_val(loc_engine_state_name, loc_engine_state_num,
         (long) state);
}

/* Find engine state name */
const char* loc_get_fix_session_state_name(rpc_loc_fix_session_state_e_type state)
{
   return loc_get_name_from_val(loc_fix_session_state_name, loc_fix_session_state_num,
         (long) state);
}

/* Event names */
loc_name_val_s_type rpc_reset_event_name[] =
{
    NAME_VAL( RPC_SUBSYSTEM_RESTART_BEGIN ),
    NAME_VAL( RPC_SUBSYSTEM_RESTART_END )
};
int rpc_reset_event_num = sizeof rpc_reset_event_name / sizeof(loc_name_val_s_type);

const char* loc_get_rpc_reset_event_name(enum rpc_reset_event event)
{
    return loc_get_name_from_val(rpc_reset_event_name, rpc_reset_event_num, event);
}
