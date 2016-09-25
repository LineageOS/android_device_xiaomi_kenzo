/* Copyright (c) 2009-2014, The Linux Foundation. All rights reserved.
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

#ifndef LOC_ENG_H
#define LOC_ENG_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

// Uncomment to keep all LOG messages (LOGD, LOGI, LOGV, etc.)
#define MAX_NUM_ATL_CONNECTIONS  2

// Define boolean type to be used by libgps on loc api module
typedef unsigned char boolean;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#include <loc.h>
#include <loc_eng_xtra.h>
#include <loc_eng_ni.h>
#include <loc_eng_agps.h>
#include <loc_cfg.h>
#include <loc_log.h>
#include <log_util.h>
#include <loc_eng_agps.h>
#include <LocEngAdapter.h>

// The data connection minimal open time
#define DATA_OPEN_MIN_TIME        1  /* sec */

// The system sees GPS engine turns off after inactive for this period of time
#define GPS_AUTO_OFF_TIME         2  /* secs */
#define SUCCESS              TRUE
#define FAILURE                 FALSE
#define INVALID_ATL_CONNECTION_HANDLE -1

#define gps_conf ContextBase::mGps_conf
#define sap_conf ContextBase::mSap_conf

enum loc_nmea_provider_e_type {
    NMEA_PROVIDER_AP = 0, // Application Processor Provider of NMEA
    NMEA_PROVIDER_MP // Modem Processor Provider of NMEA
};

enum loc_mute_session_e_type {
   LOC_MUTE_SESS_NONE = 0,
   LOC_MUTE_SESS_WAIT,
   LOC_MUTE_SESS_IN_SESSION
};

// Module data
typedef struct loc_eng_data_s
{
    LocEngAdapter                  *adapter;
    loc_location_cb_ext            location_cb;
    gps_status_callback            status_cb;
    loc_sv_status_cb_ext           sv_status_cb;
    agps_status_extended           agps_status_cb;
    gps_nmea_callback              nmea_cb;
    loc_ni_notify_callback         ni_notify_cb;
    gps_set_capabilities           set_capabilities_cb;
    gps_acquire_wakelock           acquire_wakelock_cb;
    gps_release_wakelock           release_wakelock_cb;
    gps_request_utc_time           request_utc_time_cb;
    gnss_set_system_info           set_system_info_cb;
    gnss_sv_status_callback        gnss_sv_status_cb;
    gnss_measurement_callback      gnss_measurement_cb;
    boolean                        intermediateFix;
    AGpsStatusValue                agps_status;
    loc_eng_xtra_data_s_type       xtra_module_data;
    loc_eng_ni_data_s_type         loc_eng_ni_data;

    // AGPS state machines
    AgpsStateMachine*              agnss_nif;
    AgpsStateMachine*              internet_nif;
    AgpsStateMachine*              wifi_nif;
    //State machine for Data Services
    AgpsStateMachine*              ds_nif;

    // GPS engine status
    GpsStatusValue                 engine_status;
    GpsStatusValue                 fix_session_status;

    // Aiding data information to be deleted, aiding data can only be deleted when GPS engine is off
    GpsAidingData                  aiding_data_for_deletion;

    // For muting session broadcast
    loc_mute_session_e_type        mute_session_state;

    // For nmea generation
    boolean generateNmea;
    uint32_t gps_used_mask;
    uint32_t glo_used_mask;
    float hdop;
    float pdop;
    float vdop;

    // Address buffers, for addressing setting before init
    int    supl_host_set;
    char   supl_host_buf[101];
    int    supl_port_buf;
    int    c2k_host_set;
    char   c2k_host_buf[101];
    int    c2k_port_buf;
    int    mpc_host_set;
    char   mpc_host_buf[101];
    int    mpc_port_buf;

    loc_ext_parser location_ext_parser;
    loc_ext_parser sv_ext_parser;
} loc_eng_data_s_type;

//loc_eng functions
int  loc_eng_init(loc_eng_data_s_type &loc_eng_data,
                  LocCallbacks* callbacks,
                  LOC_API_ADAPTER_EVENT_MASK_T event,
                  ContextBase* context);
int  loc_eng_start(loc_eng_data_s_type &loc_eng_data);
int  loc_eng_stop(loc_eng_data_s_type &loc_eng_data);
void loc_eng_cleanup(loc_eng_data_s_type &loc_eng_data);
int  loc_eng_inject_time(loc_eng_data_s_type &loc_eng_data,
                         GpsUtcTime time, int64_t timeReference,
                         int uncertainty);
int  loc_eng_inject_location(loc_eng_data_s_type &loc_eng_data,
                             double latitude, double longitude,
                             float accuracy);
void loc_eng_delete_aiding_data(loc_eng_data_s_type &loc_eng_data,
                                GpsAidingData f);
int  loc_eng_set_position_mode(loc_eng_data_s_type &loc_eng_data,
                               LocPosMode &params);
const void* loc_eng_get_extension(loc_eng_data_s_type &loc_eng_data,
                                  const char* name);
int  loc_eng_set_server_proxy(loc_eng_data_s_type &loc_eng_data,
                              LocServerType type, const char *hostname, int port);
void loc_eng_mute_one_session(loc_eng_data_s_type &loc_eng_data);
int loc_eng_read_config(void);

//loc_eng_agps functions
void loc_eng_agps_init(loc_eng_data_s_type &loc_eng_data,
                       AGpsExtCallbacks* callbacks);
int  loc_eng_agps_open(loc_eng_data_s_type &loc_eng_data, AGpsExtType agpsType,
                      const char* apn, AGpsBearerType bearerType);
int  loc_eng_agps_closed(loc_eng_data_s_type &loc_eng_data, AGpsExtType agpsType);
int  loc_eng_agps_open_failed(loc_eng_data_s_type &loc_eng_data, AGpsExtType agpsType);
void loc_eng_agps_ril_update_network_availability(loc_eng_data_s_type &loc_eng_data,
                                                  int avaiable, const char* apn);
int loc_eng_agps_install_certificates(loc_eng_data_s_type &loc_eng_data,
                                      const DerEncodedCertificate* certificates,
                                      size_t length);

//loc_eng_xtra functions
int  loc_eng_xtra_init (loc_eng_data_s_type &loc_eng_data,
                       GpsXtraExtCallbacks* callbacks);
int  loc_eng_xtra_inject_data(loc_eng_data_s_type &loc_eng_data,
                             char* data, int length);
int  loc_eng_xtra_request_server(loc_eng_data_s_type &loc_eng_data);
void loc_eng_xtra_version_check(loc_eng_data_s_type &loc_eng_data, int check);

//loc_eng_ni functions
extern void loc_eng_ni_init(loc_eng_data_s_type &loc_eng_data,
                            GpsNiExtCallbacks *callbacks);
extern void loc_eng_ni_respond(loc_eng_data_s_type &loc_eng_data,
                               int notif_id, GpsUserResponseType user_response);
extern void loc_eng_ni_request_handler(loc_eng_data_s_type &loc_eng_data,
                                   const GpsNiNotification *notif,
                                   const void* passThrough);
extern void loc_eng_ni_reset_on_engine_restart(loc_eng_data_s_type &loc_eng_data);

void loc_eng_configuration_update (loc_eng_data_s_type &loc_eng_data,
                                   const char* config_data, int32_t length);
int loc_eng_gps_measurement_init(loc_eng_data_s_type &loc_eng_data,
                                 GpsMeasurementCallbacks* callbacks);
void loc_eng_gps_measurement_close(loc_eng_data_s_type &loc_eng_data);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // LOC_ENG_H
