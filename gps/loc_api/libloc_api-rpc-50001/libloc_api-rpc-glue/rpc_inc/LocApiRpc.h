/* Copyright (c) 2011,2014, The Linux Foundation. All rights reserved.
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
#ifndef LOC_API_RPC_H
#define LOC_API_RPC_H

#include <rpc/rpc.h>
#include <loc_api_rpcgen_common_rpc.h>
#include <loc_api_rpc_glue.h>
#include <LocApiBase.h>
#include <loc_log.h>

using namespace loc_core;

class LocApiRpc : public LocApiBase {
protected:
    // RPC communication establishment
    rpc_loc_client_handle_type client_handle;

private:
    int dataEnableLastSet;
    char apnLastSet[MAX_APN_LEN];

    static const LOC_API_ADAPTER_EVENT_MASK_T maskAll;
    static const rpc_loc_event_mask_type locBits[];
    static rpc_loc_event_mask_type convertMask(LOC_API_ADAPTER_EVENT_MASK_T mask);
    static rpc_loc_lock_e_type convertGpsLockMask(LOC_GPS_LOCK_MASK lockMask);
    static enum loc_api_adapter_err convertErr(int rpcErr);
    static GpsNiEncodingType convertNiEncodingType(int loc_encoding);
    static int NIEventFillVerfiyType(GpsNiNotification &notif,
                              rpc_loc_ni_notify_verify_e_type notif_priv);

    void reportPosition(const rpc_loc_parsed_position_s_type *location_report_ptr);
    void reportSv(const rpc_loc_gnss_info_s_type *gnss_report_ptr);
    void reportStatus(const rpc_loc_status_event_s_type *status_report_ptr);
    void reportNmea(const rpc_loc_nmea_report_s_type *nmea_report_ptr);
    void ATLEvent(const rpc_loc_server_request_s_type *server_request_ptr);
    void NIEvent(const rpc_loc_ni_event_s_type *ni_req_ptr);

protected:
    virtual enum loc_api_adapter_err
        open(LOC_API_ADAPTER_EVENT_MASK_T mask);
    virtual enum loc_api_adapter_err
        close();

public:
    LocApiRpc(const MsgTask* msgTask,
              LOC_API_ADAPTER_EVENT_MASK_T exMask);
    ~LocApiRpc();

    virtual int locEventCB(rpc_loc_client_handle_type client_handle,
                   rpc_loc_event_mask_type loc_event,
                   const rpc_loc_event_payload_u_type* loc_event_payload);

    void locRpcGlobalCB(CLIENT* clnt, enum rpc_reset_event event);

    // RPC adapter interface implementations
    virtual enum loc_api_adapter_err
        startFix(const LocPosMode& posMode);
    virtual enum loc_api_adapter_err
        stopFix();
    virtual enum loc_api_adapter_err
        setPositionMode(const LocPosMode& mode);
    inline virtual enum loc_api_adapter_err
        enableData(int enable) { return enableData(enable, false); }
    virtual enum loc_api_adapter_err
        enableData(int enable, boolean force);
    virtual enum loc_api_adapter_err
        setTime(GpsUtcTime time, int64_t timeReference, int uncertainty);
    virtual enum loc_api_adapter_err
        injectPosition(double latitude, double longitude, float accuracy);
    virtual enum loc_api_adapter_err
        deleteAidingData(GpsAidingData f);
    virtual enum loc_api_adapter_err
        informNiResponse(GpsUserResponseType userResponse, const void* passThroughData);
    inline virtual enum loc_api_adapter_err
        setAPN(char* apn, int len) { return setAPN(apn, len, false); }
    virtual enum loc_api_adapter_err
        setAPN(char* apn, int len, boolean force);
    virtual enum loc_api_adapter_err
        setServer(const char* url, int len);
    virtual enum loc_api_adapter_err
        setServer(unsigned int ip, int port, LocServerType type);
    virtual enum loc_api_adapter_err
        setXtraData(char* data, int length);
    virtual enum loc_api_adapter_err
        requestXtraServer();
    virtual enum loc_api_adapter_err
        atlOpenStatus(int handle, int is_succ, char* apn, AGpsBearerType bear, AGpsType agpsType);
    virtual enum loc_api_adapter_err
        atlCloseStatus(int handle, int is_succ);
    virtual enum loc_api_adapter_err
        setSUPLVersion(uint32_t version);
    virtual void setInSession(bool inSession);

    /*Values for lock
      1 = Do not lock any position sessions
      2 = Lock MI position sessions
      3 = Lock MT position sessions
      4 = Lock all position sessions
    */
    virtual int setGpsLock(LOC_GPS_LOCK_MASK lock);
    /*
     Returns
     Current value of GPS Lock on success
     -1 on failure
     */
    virtual int getGpsLock(void);
};

extern "C" LocApiBase* getLocApi(const MsgTask* msgTask,
                                 LOC_API_ADAPTER_EVENT_MASK_T exMask,
                                 ContextBase *context);

#endif //LOC_API_RPC_H
