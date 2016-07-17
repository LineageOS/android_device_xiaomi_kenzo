/* Copyright (c) 2011, The Linux Foundation. All rights reserved.
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
 */

#ifndef  LOC_API_FIXUP_H
#define LOC_API_FIXUP_H

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef NULLPROC
#define NULLPROC 0
#endif /* NULLPROC */

#ifdef ADD_XDR_FLOAT

extern bool_t xdr_float (XDR *__xdrs, float *__fp);
extern bool_t xdr_double (XDR *__xdrs, double *__dp);

#endif /* ADD_XDR_FLOAT */

#ifdef ADD_XDR_BOOL
extern bool_t xdr_bool(XDR *__xdrs, int *__bp);
#endif /* ADD_XDR_BOOL */

#define RPC_LOC_API_MAJOR_VERSION_NUMBER   1
#define RPC_LOC_API_MINOR_VERSION_NUMBER   0

// Return value for loc_open in case of failure.
#define RPC_LOC_CLIENT_HANDLE_INVALID     -1

// Return value of loc api calls for loc_close, loc_start_fix, loc_stop_fix and loc_ioctl
// These are also the status for the ioctl callback
#define RPC_LOC_API_SUCCESS           0
#define RPC_LOC_API_GENERAL_FAILURE   1
#define RPC_LOC_API_UNSUPPORTED       2
#define RPC_LOC_API_INVALID_HANDLE    4
#define RPC_LOC_API_INVALID_PARAMETER 5
#define RPC_LOC_API_ENGINE_BUSY       6
#define RPC_LOC_API_PHONE_OFFLINE     7
#define RPC_LOC_API_TIMEOUT           8

// Special return value for loc api calls in case of RPC failure
#define RPC_LOC_API_RPC_FAILURE       (-1234)
// Special return value for modem restart incurred RPC failure
#define RPC_LOC_API_RPC_MODEM_RESTART       (-1235)

#define RPC_LOC_API_MAX_SV_COUNT                      80
#define RPC_LOC_API_MAX_NMEA_STRING_LENGTH            1200

// Maximum server address that will be used in location API
#define RPC_LOC_API_MAX_SERVER_ADDR_LENGTH            256
#define RPC_LOC_API_MAX_NUM_PREDICTED_ORBITS_SERVERS  3
#define RPC_LOC_API_MAX_NUM_NTP_SERVERS               3

#define RPC_LOC_EVENT_PARSED_POSITION_REPORT             0x00000001 // Position report comes in loc_parsed_position_s_type
#define RPC_LOC_EVENT_SATELLITE_REPORT                   0x00000002 // Satellite in view report
#define RPC_LOC_EVENT_NMEA_1HZ_REPORT                    0x00000004 // NMEA report at 1HZ rate
#define RPC_LOC_EVENT_NMEA_POSITION_REPORT               0x00000008 // NMEA report at position report rate
#define RPC_LOC_EVENT_NI_NOTIFY_VERIFY_REQUEST           0x00000010 // NI notification/verification request
#define RPC_LOC_EVENT_ASSISTANCE_DATA_REQUEST            0x00000020 // Assistance data, eg: time, predicted orbits request
#define RPC_LOC_EVENT_LOCATION_SERVER_REQUEST            0x00000040 // Request for location server
#define RPC_LOC_EVENT_IOCTL_REPORT                       0x00000080 // Callback report for loc_ioctl
#define RPC_LOC_EVENT_STATUS_REPORT                      0x00000100 // Misc status report: eg, engine state

#define RPC_LOC_POS_VALID_SESSION_STATUS                 0x00000001
#define RPC_LOC_POS_VALID_TIMESTAMP_CALENDAR             0x00000002
#define RPC_LOC_POS_VALID_TIMESTAMP_UTC                  0x00000004
#define RPC_LOC_POS_VALID_LEAP_SECONDS                   0x00000008
#define RPC_LOC_POS_VALID_TIME_UNC                       0x00000010
#define RPC_LOC_POS_VALID_LATITUDE                       0x00000020
#define RPC_LOC_POS_VALID_LONGITUDE                      0x00000040
#define RPC_LOC_POS_VALID_ALTITUDE_WRT_ELLIPSOID         0x00000080
#define RPC_LOC_POS_VALID_ALTITUDE_WRT_MEAN_SEA_LEVEL    0x00000100
#define RPC_LOC_POS_VALID_SPEED_HORIZONTAL               0x00000200
#define RPC_LOC_POS_VALID_SPEED_VERTICAL                 0x00000400
#define RPC_LOC_POS_VALID_HEADING                        0x00000800
#define RPC_LOC_POS_VALID_HOR_UNC_CIRCULAR               0x00001000
#define RPC_LOC_POS_VALID_HOR_UNC_ELLI_SEMI_MAJ          0x00002000
#define RPC_LOC_POS_VALID_HOR_UNC_ELLI_SEMI_MIN          0x00004000
#define RPC_LOC_POS_VALID_HOR_UNC_ELLI_ORIENT_AZIMUTH    0x00008000
#define RPC_LOC_POS_VALID_VERTICAL_UNC                   0x00010000
#define RPC_LOC_POS_VALID_SPEED_UNC                      0x00020000
#define RPC_LOC_POS_VALID_HEADING_UNC                    0x00040000
#define RPC_LOC_POS_VALID_CONFIDENCE_HORIZONTAL          0x00080000
#define RPC_LOC_POS_VALID_CONFIDENCE_VERTICAL            0x00100000
#define RPC_LOC_POS_VALID_MAGNETIC_VARIATION             0x00200000
#define RPC_LOC_POS_VALID_TECHNOLOGY_MASK                0x00400000

#define RPC_LOC_POS_TECH_SATELLITE 0x00000001
#define RPC_LOC_POS_TECH_CELLID    0x00000002
#define RPC_LOC_POS_TECH_WIFI      0x00000004

#define RPC_LOC_SV_INFO_VALID_SYSTEM         0x00000001
#define RPC_LOC_SV_INFO_VALID_PRN            0x00000002
#define RPC_LOC_SV_INFO_VALID_HEALTH_STATUS  0x00000004
#define RPC_LOC_SV_INFO_VALID_PROCESS_STATUS 0x00000008
#define RPC_LOC_SV_INFO_VALID_HAS_EPH        0x00000010
#define RPC_LOC_SV_INFO_VALID_HAS_ALM        0x00000020
#define RPC_LOC_SV_INFO_VALID_ELEVATION      0x00000040
#define RPC_LOC_SV_INFO_VALID_AZIMUTH        0x00000080
#define RPC_LOC_SV_INFO_VALID_SNR            0x00000100

#define RPC_LOC_GNSS_INFO_VALID_POS_DOP           0x00000001
#define RPC_LOC_GNSS_INFO_VALID_HOR_DOP           0x00000002
#define RPC_LOC_GNSS_INFO_VALID_VERT_DOP          0x00000004
#define RPC_LOC_GNSS_INFO_VALID_ALTITUDE_ASSUMED  0x00000008
#define RPC_LOC_GNSS_INFO_VALID_SV_COUNT          0x00000010
#define RPC_LOC_GNSS_INFO_VALID_SV_LIST           0x00000020

#define RPC_LOC_NI_MAX_REQUESTOR_ID_LENGTH          200
#define RPC_LOC_NI_SUPL_HASH_LENGTH                 8
#define RPC_LOC_NI_SUPL_SLP_SESSION_ID_BYTE_LENGTH  4
#define RPC_LOC_NI_MAX_CLIENT_NAME_LENGTH           64
#define RPC_LOC_NI_MAX_EXT_CLIENT_ADDRESS           20
#define RPC_LOC_NI_CODEWORD_LENGTH                  20

#define RPC_LOC_NI_SUPL_QOP_VALID         0x01
#define RPC_LOC_NI_SUPL_QOP_VERACC_VALID  0x02
#define RPC_LOC_NI_SUPL_QOP_MAXAGE_VALID  0x04
#define RPC_LOC_NI_SUPL_QOP_DELAY_VALID   0x08

#define RPC_LOC_FIX_CRIT_VALID_RECURRENCE_TYPE                  0x00000001
#define RPC_LOC_FIX_CRIT_VALID_PREFERRED_OPERATION_MODE         0x00000002
#define RPC_LOC_FIX_CRIT_VALID_PREFERRED_ACCURACY               0x00000004
#define RPC_LOC_FIX_CRIT_VALID_PREFERRED_RESPONSE_TIME          0x00000008
#define RPC_LOC_FIX_CRIT_VALID_INTERMEDIATE_POS_REPORT_ENABLED  0x00000010
#define RPC_LOC_FIX_CRIT_VALID_NOTIFY_TYPE                      0x00000020
#define RPC_LOC_FIX_CRIT_VALID_MIN_INTERVAL                     0x00000040
#define RPC_LOC_FIX_CRIT_VALID_MIN_DISTANCE                     0x00000080
#define RPC_LOC_FIX_CRIT_VALID_MIN_DIST_SAMPLE_INTERVAL         0x00000100

#define RPC_LOC_ASSIST_POS_VALID_TIMESTAMP_UTC                0x00000001
#define RPC_LOC_ASSIST_POS_VALID_LATITUDE                     0x00000002
#define RPC_LOC_ASSIST_POS_VALID_LONGITUDE                    0x00000004
#define RPC_LOC_ASSIST_POS_VALID_ALTITUDE_WRT_ELLIPSOID       0x00000008
#define RPC_LOC_ASSIST_POS_VALID_ALTITUDE_WRT_MEAN_SEA_LEVEL  0x00000010
#define RPC_LOC_ASSIST_POS_VALID_HOR_UNC_CIRCULAR             0x00000020
#define RPC_LOC_ASSIST_POS_VALID_VERT_UNC                     0x00000040
#define RPC_LOC_ASSIST_POS_VALID_CONFIDENCE_HORIZONTAL        0x00000080
#define RPC_LOC_ASSIST_POS_VALID_CONFIDENCE_VERTICAL          0x00000100
#define RPC_LOC_ASSIST_POS_VALID_TIMESTAMP_AGE                0x00000200

#define RPC_LOC_ASSIST_DATA_ALL         0xFFFFFFFF

#define RPC_LOC_NMEA_MASK_ALL    0xffff
#define RPC_LOC_NMEA_MASK_GGA    0x0001
#define RPC_LOC_NMEA_MASK_RMC    0x0002
#define RPC_LOC_NMEA_MASK_GSV    0x0004
#define RPC_LOC_NMEA_MASK_GSA    0x0008
#define RPC_LOC_NMEA_MASK_VTG    0x0010

/* EFS data access */
#define RPC_LOC_EFS_MAX_PATH_LEN_BYTES  64     /* Max file name length in bytes that can be written*/
#define RPC_LOC_EFS_MAX_FILE_LEN_BYTES  2000   /* Max file size in bytes that can be written */

/* WIPER valid information flag in log report */
#define RPC_LOC_WIPER_LOG_TIME_VALID 0x01
#define RPC_LOC_WIPER_LOG_POS_VALID 0x02
#define RPC_LOC_WIPER_LOG_AP_SET_VALID 0x04

/* General WIPER defines */
#define RPC_LOC_WIPER_MAC_ADDR_LENGTH  6              // Do not change this number since it affects RPC and log packet sizes
#define RPC_LOC_WIPER_MAX_REPORTED_APS_PER_LOG_MSG 50 // Do not change this number since it affects RPC and log packet sizes

/* WIPER AP Qualifier */
#define RPC_LOC_WIPER_AP_QUALIFIER_BEING_USED 0x1  /* AP is being used by WPS */
#define RPC_LOC_WIPER_AP_QUALIFIER_HIDDEN_SSID 0x2 /* AP does not broadcast SSID */
#define RPC_LOC_WIPER_AP_QUALIFIER_PRIVATE 0x4     /* AP has encryption turned on */
#define RPC_LOC_WIPER_AP_QUALIFIER_INFRASTRUCTURE_MODE 0x8     /* AP is in infrastructure mode and not in ad-hoc/unknown mode */

/* flags for notification */
#define  RPC_LOC_NI_CLIENT_NAME_PRESENT             0x0001
#define  RPC_LOC_NI_CLIENT_EXTADDR_PRESENT          0x0002
#define  RPC_LOC_NI_DEF_LOCATION_TYPE_PRESENT       0x0010
#define  RPC_LOC_NI_REQUESTOR_ID_PRESENT            0x0020
#define  RPC_LOC_NI_CODEWORD_PRESENT                0x0040
#define  RPC_LOC_NI_SERVICE_TYPE_ID_PRESENT         0x0080
#define  RPC_LOC_NI_ENCODING_TYPE_PRESENT           0x0100

/* below are for RPC_LOC_IOCTL_SET_LBS_APN_PROFILE data */
/* values for apn_profiles[0].srv_system_type */
#define LOC_APN_PROFILE_SRV_SYS_CDMA  0x01
#define LOC_APN_PROFILE_SRV_SYS_HDR   0x02
#define LOC_APN_PROFILE_SRV_SYS_GSM   0x04
#define LOC_APN_PROFILE_SRV_SYS_WCDMA 0x08
#define LOC_APN_PROFILE_SRV_SYS_LTE   0x10
#define LOC_APN_PROFILE_SRV_SYS_MAX   0x1F
/* values for apn_profiles[0].pdp_type */
#define LOC_APN_PROFILE_PDN_TYPE_IPV4    0x01
#define LOC_APN_PROFILE_PDN_TYPE_IPV6    0x02
#define LOC_APN_PROFILE_PDN_TYPE_IPV4V6  0x03
#define LOC_APN_PROFILE_PDN_TYPE_PPP     0x04
#define LOC_APN_PROFILE_PDN_TYPE_MAX     0x04

#ifdef __cplusplus
}
#endif

#endif /* LOC_API_FIXUP_H */
