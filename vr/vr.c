/*
 * Copyright (C) 2016 The Android Open Source Project
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

#define LOG_TAG "VrHALImpl"

#include <cutils/log.h>

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <hardware/vr.h>
#include <hardware/hardware.h>


// Kenzo has two inflight numbers. By default, inflight=15 and inflight_low_latency=4.
// Inflight is only used when there is a single GL context, when there is more than one
// context, inflight_low_latency is used. Since we are only interested in affecting
// performance when there is context preemption, we only have to modify the low latency
// parameter.
static const int DEFAULT_GPU_INFLIGHT = 4;
static const int VR_MODE_GPU_INFLIGHT = 2;
static const char* GPU_INFLIGHT_PATH = "/sys/class/kgsl/kgsl-3d0/dispatch/inflight_low_latency";

/**
 * Write 'len' characters from 'input' character array into file at path 'outFile.'
 *
 * Return 0 on success, or a negative error code.
 */
static int write_string(const char* input, size_t len, const char* outFile) {
    int fd = -1;
    ssize_t err = 0;

    // Check input strings.
    if (input == NULL || outFile == NULL) {
        ALOGE("%s: Invalid input to write", __FUNCTION__);
        return -1;
    }

    // Open file, check for errors.
    fd = open(outFile, O_WRONLY);
    if (fd < 0) {
        ALOGE("%s: Failed to open file %s, error %s (%d)", __FUNCTION__, outFile, strerror(errno),
              -errno);
        return -errno;
    }

    // Write file, check for errors.
    err = write(fd, input, len);
    if (err < 0) {
        ALOGE("%s: Failed to write file %s, error %s (%d)", __FUNCTION__, outFile, strerror(errno),
              -errno);
        close(fd);
        return -errno;
    }

    // Close and return success.
    close(fd);
    return 0;
}

/**
 * Write integer 'input' formatted as a character string into the file at path 'outFile.'
 *
 * Return 0 on success, or a negative error code.
 */
static int write_int(int input, const char* outFile) {
    char buffer[128] = {0,};
    int bytes = snprintf(buffer, sizeof(buffer), "%d", input);

    if (bytes < 0 || (size_t) bytes >= sizeof(buffer)) {
        ALOGE("%s: Failed to format integer %d", __FUNCTION__, input);
        return -EINVAL;
    }

    return write_string(buffer, (size_t) bytes, outFile);
}

// Set global display/GPU/scheduler configuration to used for VR apps.
static void set_vr_performance_configuration() {
    int err = 0;

    // Set in-flight buffers to 2.
    err = write_int(VR_MODE_GPU_INFLIGHT, GPU_INFLIGHT_PATH);

    if (err < 0) {
        ALOGW("%s: Error while setting configuration for VR mode.", __FUNCTION__);
    }
}

// Reset to default global display/GPU/scheduler configuration.
static void unset_vr_performance_configuration() {
    int err = 0;

    // Set in-flight buffers back to default (15).
    err = write_int(DEFAULT_GPU_INFLIGHT, GPU_INFLIGHT_PATH);

    if (err < 0) {
        ALOGW("%s: Error while setting configuration for VR mode.", __FUNCTION__);
    }
}

static void vr_init(struct vr_module *module) {
    // NOOP
}

static void vr_set_vr_mode(struct vr_module *module, bool enabled) {
    if (enabled) {
        set_vr_performance_configuration();
    } else {
        unset_vr_performance_configuration();
    }
}

static struct hw_module_methods_t vr_module_methods = {
    .open = NULL, // There are no devices for this HAL interface.
};


vr_module_t HAL_MODULE_INFO_SYM = {
    .common = {
        .tag                = HARDWARE_MODULE_TAG,
        .module_api_version = VR_MODULE_API_VERSION_1_0,
        .hal_api_version    = HARDWARE_HAL_API_VERSION,
        .id                 = VR_HARDWARE_MODULE_ID,
        .name               = "Kenzo VR HAL",
        .author             = "The Android Open Source Project",
        .methods            = &vr_module_methods,
    },

    .init = vr_init,
    .set_vr_mode = vr_set_vr_mode,
};
