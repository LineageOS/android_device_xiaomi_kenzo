/* Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
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
 */

#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <sys/time.h>
#include <string.h>
#include <pthread.h>

#include <rpc/rpc.h>
#include <loc_api_rpc_glue.h>
#include "loc_api_sync_call.h"

/* Logging */
#define LOG_TAG "LocSvc_api_rpc_glue"
// #define LOG_NDDEBUG 0
#ifndef USE_GLIB
#include <utils/Log.h>
#endif /* USE_GLIB */

/***************************************************************************
 *                 DATA FOR ASYNCHRONOUS RPC PROCESSING
 **************************************************************************/
loc_sync_call_slot_array_s_type loc_sync_data;

pthread_mutex_t loc_sync_call_mutex = PTHREAD_MUTEX_INITIALIZER;
boolean loc_sync_call_inited = 0;

/*===========================================================================

FUNCTION    loc_api_sync_call_init

DESCRIPTION
   Initialize this module

DEPENDENCIES
   N/A

RETURN VALUE
   none

SIDE EFFECTS
   N/A

===========================================================================*/
void loc_api_sync_call_init()
{
   pthread_mutex_lock(&loc_sync_call_mutex);
   if (loc_sync_call_inited == 1) {
       pthread_mutex_unlock(&loc_sync_call_mutex);
       return;
   }
   loc_sync_call_inited = 1;

   loc_sync_data.num_of_slots = LOC_SYNC_CALL_SLOTS_MAX;

   int i;
   for (i = 0; i < loc_sync_data.num_of_slots; i++)
   {
      loc_sync_call_slot_s_type *slot = &loc_sync_data.slots[i];

      pthread_mutex_init(&slot->lock, NULL);
      pthread_cond_init(&slot->loc_cb_arrived_cond, NULL);

      slot->not_available = 0;
      slot->in_use = 0;
      slot->loc_handle = -1;
      slot->loc_cb_wait_event_mask = 0;       /* event to wait   */
      slot->loc_cb_received_event_mask = 0;   /* received event   */
   }

   pthread_mutex_unlock(&loc_sync_call_mutex);
}

/*===========================================================================

FUNCTION    loc_api_sync_call_destroy

DESCRIPTION
   Initialize this module

DEPENDENCIES
   N/A

RETURN VALUE
   none

SIDE EFFECTS
   N/A

===========================================================================*/
void loc_api_sync_call_destroy()
{
   int i;

   pthread_mutex_lock(&loc_sync_call_mutex);
   if (loc_sync_call_inited == 0) {
       pthread_mutex_unlock(&loc_sync_call_mutex);
       return;
   }
   loc_sync_call_inited = 0;

   for (i = 0; i < loc_sync_data.num_of_slots; i++)
   {
      loc_sync_call_slot_s_type *slot = &loc_sync_data.slots[i];

      pthread_mutex_lock(&slot->lock);

      slot->not_available = 1;

      pthread_mutex_unlock(&slot->lock);

      pthread_cond_destroy(&slot->loc_cb_arrived_cond);
      pthread_mutex_destroy(&slot->lock);
   }

   pthread_mutex_unlock(&loc_sync_call_mutex);
}

/*===========================================================================

FUNCTION    loc_match_callback

DESCRIPTION
   Checks if an awaited event has arrived

RETURN VALUE
   TRUE                 arrived
   FALSE                not matching

===========================================================================*/
static boolean loc_match_callback(
      rpc_loc_event_mask_type             wait_mask,
      rpc_loc_ioctl_e_type                wait_ioctl,
      rpc_loc_event_mask_type             event_mask,
      const rpc_loc_event_payload_u_type  *callback_payload
)
{
   if ((event_mask & wait_mask) == 0) return FALSE;

   if (event_mask != RPC_LOC_EVENT_IOCTL_REPORT || wait_ioctl == 0 ||
        ( (callback_payload != NULL) &&
         callback_payload->rpc_loc_event_payload_u_type_u.ioctl_report.type == wait_ioctl) )
      return TRUE;

   return FALSE;
}

/*===========================================================================

FUNCTION    loc_api_callback_process_sync_call

DESCRIPTION
   Wakes up blocked API calls to check if the needed callback has arrived

DEPENDENCIES
   N/A

RETURN VALUE
   none

SIDE EFFECTS
   N/A

===========================================================================*/
void loc_api_callback_process_sync_call(
      rpc_loc_client_handle_type            loc_handle,             /* handle of the client */
      rpc_loc_event_mask_type               loc_event,              /* event mask           */
      const rpc_loc_event_payload_u_type*   loc_event_payload       /* payload              */
)
{
   int i;

   ALOGV("loc_handle = 0x%lx, loc_event = 0x%lx", loc_handle, loc_event);
   for (i = 0; i < loc_sync_data.num_of_slots; i++)
   {
      loc_sync_call_slot_s_type *slot = &loc_sync_data.slots[i];

      pthread_mutex_lock(&slot->lock);

      if (slot->in_use &&
          slot->signal_sent == 0 &&
          slot->loc_handle == loc_handle &&
          loc_match_callback(slot->loc_cb_wait_event_mask, slot->ioctl_type, loc_event, loc_event_payload))
      {
         memcpy(&slot->loc_cb_received_payload, loc_event_payload, sizeof (rpc_loc_event_payload_u_type));

         slot->loc_cb_received_event_mask = loc_event;

         ALOGV("signal slot %d in_use %d, loc_handle 0x%lx, event_mask 0x%1x, ioctl_type %d", i, slot->in_use, slot->loc_handle, (int) slot->loc_cb_wait_event_mask, (int) slot->ioctl_type);
         pthread_cond_signal(&slot->loc_cb_arrived_cond);
         slot->signal_sent = 1;

         pthread_mutex_unlock(&slot->lock);
         break;
      } else {
         /* do nothing */
      }

      pthread_mutex_unlock(&slot->lock);
   }
}

/*===========================================================================

FUNCTION    loc_lock_a_slot

DESCRIPTION
   Allocates a buffer slot for the synchronous API call

DEPENDENCIES
   N/A

RETURN VALUE
   Select ID (>=0)     : successful
   -1                  : buffer full

SIDE EFFECTS
   N/A

===========================================================================*/
static int loc_lock_a_slot()
{
   int i, select_id = -1; /* no free buffer */

   for (i = 0; i < loc_sync_data.num_of_slots; i++)
   {
      loc_sync_call_slot_s_type *slot = &loc_sync_data.slots[i];
      if (pthread_mutex_trylock(&slot->lock) == EBUSY)
      {
         ALOGV("trylock EBUSY : %d", i);
         continue;
      }

      if (!slot->in_use && !slot->not_available)
      {
         select_id = i;
         /* Return from here and leave the mutex locked.
          * will unlock it in loc_unlock_slot()
          */
         break;
      }
      /* ALOGV("slot %d in_use = %d, not_available = %d : %d", i, slot->in_use, slot->not_available, i); */
      pthread_mutex_unlock(&slot->lock);
   }

   return select_id;
}

/*===========================================================================

FUNCTION    loc_unlock_slot

DESCRIPTION
   Unlocks a buffer slot

DEPENDENCIES
   N/A

RETURN VALUE
   None

SIDE EFFECTS
   N/A

===========================================================================*/
static void loc_unlock_slot(int select_id)
{
   pthread_mutex_unlock(&loc_sync_data.slots[select_id].lock);
}

/*===========================================================================

FUNCTION    loc_lock_slot

DESCRIPTION
   Locks a specific slot that was previously locked from loc_lock_a_slot

DEPENDENCIES
   N/A

RETURN VALUE
   None

SIDE EFFECTS
   N/A

===========================================================================*/
static void loc_lock_slot(int select_id)
{
    pthread_mutex_lock(&loc_sync_data.slots[select_id].lock);
}

/*===========================================================================

FUNCTION    loc_set_slot_in_use

DESCRIPTION
   Sets the in_use flag of slot to true or false.
   Should be called only after the slot is locked

DEPENDENCIES
   N/A

RETURN VALUE
   None

SIDE EFFECTS
   N/A

===========================================================================*/
static void loc_set_slot_in_use(int select_id, boolean in_use)
{
    loc_sync_data.slots[select_id].in_use = in_use;
    if (in_use == 1)
        loc_sync_data.slots[select_id].signal_sent = 0;
}

/*===========================================================================

FUNCTION    loc_api_save_callback

DESCRIPTION
   Selects which callback or IOCTL event to wait for.

   The event_mask specifies the event(s). If it is RPC_LOC_EVENT_IOCTL_REPORT,
   then ioctl_type specifies the IOCTL event.

   If ioctl_type is non-zero, RPC_LOC_EVENT_IOCTL_REPORT is automatically added.

DEPENDENCIES
   N/A

RETURN VALUE
   Select ID (>=0)     : successful
   -1                  : out of buffer

SIDE EFFECTS
   N/A

===========================================================================*/
static void loc_api_save_callback(
      int                              select_id,            /* Selected slot */
      rpc_loc_client_handle_type       loc_handle,           /* Client handle */
      rpc_loc_event_mask_type          event_mask,           /* Event mask to wait for */
      rpc_loc_ioctl_e_type             ioctl_type            /* IOCTL type to wait for */
)
{
   loc_sync_call_slot_s_type *slot = &loc_sync_data.slots[select_id];

   slot->loc_handle = loc_handle;

   slot->loc_cb_wait_event_mask = event_mask;
   slot->ioctl_type = ioctl_type;
   if (ioctl_type) slot->loc_cb_wait_event_mask |= RPC_LOC_EVENT_IOCTL_REPORT;

   return;
}

/*===========================================================================

FUNCTION    loc_save_user_payload

DESCRIPTION
   Saves received payload into user data structures

RETURN VALUE
   None

===========================================================================*/
static void loc_save_user_payload(
      rpc_loc_event_payload_u_type  *user_cb_payload,
      rpc_loc_ioctl_callback_s_type *user_ioctl_buffer,
      const rpc_loc_event_payload_u_type  *received_cb_payload
)
{
   if (user_cb_payload)
   {
      memcpy(user_cb_payload, received_cb_payload,
            sizeof (rpc_loc_event_payload_u_type));
   }
   if (user_ioctl_buffer)
   {
      memcpy(user_ioctl_buffer,
            &received_cb_payload->rpc_loc_event_payload_u_type_u.ioctl_report,
            sizeof *user_ioctl_buffer);
   }
}

/*===========================================================================

FUNCTION    loc_api_wait_callback

DESCRIPTION
   Waits for a selected callback. The wait expires in timeout_seconds seconds.

   If the function is called before an existing wait has finished, it will
   immediately return EBUSY.

DEPENDENCIES
   N/A

RETURN VALUE
   RPC_LOC_API_SUCCESS              if successful (0)
   RPC_LOC_API_TIMEOUT              if timed out
   RPC_LOC_API_ENGINE_BUSY          if already in a wait
   RPC_LOC_API_INVALID_PARAMETER    if callback is not yet selected

SIDE EFFECTS
   N/A

===========================================================================*/
static int loc_api_wait_callback(
      int select_id,        /* ID from loc_select_callback() */
      int timeout_seconds,  /* Timeout in this number of seconds  */
      rpc_loc_event_payload_u_type     *callback_payload,    /* Pointer to callback payload buffer, can be NULL */
      rpc_loc_ioctl_callback_s_type    *ioctl_payload        /* Pointer to IOCTL payload, can be NULL */
)
{
   int ret_val = RPC_LOC_API_SUCCESS;  /* the return value of this function: 0 = no error */
   int rc = 0;                         /* return code from pthread calls */

   struct timespec expire_time;

   loc_sync_call_slot_s_type *slot = &loc_sync_data.slots[select_id];

   clock_gettime(CLOCK_REALTIME, &expire_time);
   expire_time.tv_sec += timeout_seconds;

   /* Waiting */
   while (slot->signal_sent == 0 && rc != ETIMEDOUT) {
       rc = pthread_cond_timedwait(&slot->loc_cb_arrived_cond,
             &slot->lock, &expire_time);
   }

   if (rc == ETIMEDOUT)
   {
      ret_val = RPC_LOC_API_TIMEOUT; /* Timed out */
      ALOGE("TIMEOUT: %d", select_id);
   }
   else {
      /* Obtained the first awaited callback */
      ret_val = RPC_LOC_API_SUCCESS;       /* Successful */
      loc_save_user_payload(callback_payload, ioctl_payload, &slot->loc_cb_received_payload);
   }

   return ret_val;
}

/*===========================================================================

FUNCTION    loc_api_sync_ioctl

DESCRIPTION
   Synchronous IOCTL call (reentrant version)

DEPENDENCIES
   N/A

RETURN VALUE
   Loc API error code (0 = success)

SIDE EFFECTS
   N/A

===========================================================================*/
int loc_api_sync_ioctl
(
      rpc_loc_client_handle_type           handle,
      rpc_loc_ioctl_e_type                 ioctl_type,
      rpc_loc_ioctl_data_u_type*           ioctl_data_ptr,
      uint32                               timeout_msec,
      rpc_loc_ioctl_callback_s_type       *cb_data_ptr
)
{
   int                              rc = -1;
   int                              select_id;
   rpc_loc_ioctl_callback_s_type    callback_data;

   select_id = loc_lock_a_slot();

   if (select_id < 0 || select_id >= loc_sync_data.num_of_slots)
   {
      ALOGE("slot not available ioctl_type = %s",
           loc_get_ioctl_type_name(ioctl_type));
      return rc;
   }

   loc_set_slot_in_use(select_id, 1); // set slot in use to true

   // Select the callback we are waiting for
   loc_api_save_callback(select_id, handle, 0, ioctl_type);

   loc_unlock_slot(select_id); // slot is unlocked, but in_use is still true

   // we want to avoid keeping the slot locked during the loc_ioctl because the rpc
   // framework will also lock a different mutex during this call, and typically
   // locking two different mutexes at the same time can lead to deadlock.
   rc =  loc_ioctl(handle, ioctl_type, ioctl_data_ptr);

   loc_lock_slot(select_id);

   if (rc != RPC_LOC_API_SUCCESS)
   {
      ALOGE("loc_ioctl failed select_id = %d, ioctl_type %s, returned %s",
           select_id, loc_get_ioctl_type_name(ioctl_type), loc_get_ioctl_status_name(rc));
   }
   else {
      ALOGV("select_id = %d, ioctl_type %d, returned RPC_LOC_API_SUCCESS",
          select_id, ioctl_type);
      // Wait for the callback of loc_ioctl
      if ((rc = loc_api_wait_callback(select_id, timeout_msec / 1000, NULL, &callback_data)) != 0)
      {
         // Callback waiting failed
         ALOGE("callback wait failed select_id = %d, ioctl_type %s, returned %s",
              select_id, loc_get_ioctl_type_name(ioctl_type), loc_get_ioctl_status_name(rc));
      }
      else
      {
         if (cb_data_ptr) memcpy(cb_data_ptr, &callback_data, sizeof *cb_data_ptr);
         if (callback_data.status != RPC_LOC_API_SUCCESS)
         {
            rc = callback_data.status;
            ALOGE("callback status failed select_id = %d, ioctl_type %s, returned %s",
                 select_id, loc_get_ioctl_type_name(ioctl_type), loc_get_ioctl_status_name(rc));
         } else {
            ALOGV("callback status success select_id = %d, ioctl_type %d, returned %d",
                select_id, ioctl_type, rc);
         }
      } /* wait callback */
   } /* loc_ioctl */

   loc_set_slot_in_use(select_id, 0); // set slot in use to false
   loc_unlock_slot(select_id);

   return rc;
}


