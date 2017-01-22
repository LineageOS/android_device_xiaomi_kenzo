/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 * Not a Contribution.
 *
 * Copyright (C) 2012 The Android Open Source Project
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

#ifndef __QCAMERATRACE_H__
#define __QCAMERATRACE_H__

#define ATRACE_TAG ATRACE_TAG_CAMERA
#include <utils/Trace.h>

#undef ATRACE_CALL
#undef ATRACE_NAME
#undef ATRACE_BEGIN
#undef ATRACE_INT
#undef ATRACE_END

#define KPI_ONLY 1
#define KPI_DBG 2

#define CAMERA_TRACE_BUF 32

//to enable only KPI logs
#define KPI_ATRACE_BEGIN(name) ({\
if (gKpiDebugLevel >= KPI_ONLY) { \
     atrace_begin(ATRACE_TAG, name); \
}\
})

#define KPI_ATRACE_END() ({\
if (gKpiDebugLevel >= KPI_ONLY) { \
     atrace_end(ATRACE_TAG); \
}\
})

#define KPI_ATRACE_INT(name,val) ({\
if (gKpiDebugLevel >= KPI_ONLY) { \
     atrace_int(ATRACE_TAG, name, val); \
}\
})


#define ATRACE_BEGIN_SNPRINTF(fmt_str, ...) \
 if (gKpiDebugLevel >= KPI_DBG) { \
   char trace_tag[CAMERA_TRACE_BUF]; \
   snprintf(trace_tag, CAMERA_TRACE_BUF, fmt_str, ##__VA_ARGS__); \
   ATRACE_BEGIN(trace_tag); \
}

#define ATRACE_BEGIN_DBG(name) ({\
if (gKpiDebugLevel >= KPI_DBG) { \
     atrace_begin(ATRACE_TAG, name); \
}\
})

#define ATRACE_END_DBG() ({\
if (gKpiDebugLevel >= KPI_DBG) { \
     atrace_end(ATRACE_TAG); \
}\
})

#define ATRACE_INT_DBG(name,val) ({\
if (gKpiDebugLevel >= KPI_DBG) { \
     atrace_int(ATRACE_TAG, name, val); \
}\
})

#define ATRACE_BEGIN ATRACE_BEGIN_DBG
#define ATRACE_INT ATRACE_INT_DBG
#define ATRACE_END ATRACE_END_DBG

#define KPI_ATRACE_NAME(name) qcamera::ScopedTraceKpi ___tracer(ATRACE_TAG, name)
#define ATRACE_NAME(name) qcamera::ScopedTraceDbg ___tracer(ATRACE_TAG, name)
#define KPI_ATRACE_CALL() KPI_ATRACE_NAME(__FUNCTION__)
#define ATRACE_CALL() ATRACE_NAME(__FUNCTION__)

namespace qcamera {
extern volatile uint32_t gKpiDebugLevel;
class ScopedTraceKpi {
public:
    inline ScopedTraceKpi(uint64_t tag, const char *name)
    : mTag(tag) {
        if (gKpiDebugLevel >= KPI_ONLY) {
            atrace_begin(mTag,name);
        }
    }

    inline ~ScopedTraceKpi() {
        if (gKpiDebugLevel >= KPI_ONLY) {
            atrace_end(mTag);
        }
    }

    private:
        uint64_t mTag;
};

class ScopedTraceDbg {
public:
    inline ScopedTraceDbg(uint64_t tag, const char *name)
    : mTag(tag) {
        if (gKpiDebugLevel >= KPI_DBG) {
            atrace_begin(mTag,name);
        }
    }

    inline ~ScopedTraceDbg() {
        if (gKpiDebugLevel >= KPI_DBG) {
            atrace_end(mTag);
        }
    }

    private:
        uint64_t mTag;
};
};

extern volatile uint32_t gKpiDebugLevel;

#endif /* __QCAMREATRACE_H__ */
