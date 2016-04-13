/*
 * Copyright (C) 2016 The CyanogenMod Project
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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>
#include <utils/Log.h>
#include <hardware/power.h>
#include <hardware/hardware.h>

//various funcs we'll need to call, in their mangled form

    //android::String16::String16(char const*)
    extern void _ZN7android8String16C1EPKc(void **str16P, const char *str);

    //android::String16::~String16()
    extern void _ZN7android8String16D1Ev(void **str16P);

    //android::SensorManager::SensorManager(android::String16 const&)
    extern void _ZN7android13SensorManagerC1ERKNS_8String16E(void *sensorMgr, void **str16P);

    extern int _ZN7android5Fence4waitEi(int);

//code exports we provide

    //android::SensorManager::SensorManager(void)
    void _ZN7android13SensorManagerC1Ev(void *sensorMgr);

    int _ZN7android5Fence4waitEj(unsigned int timeout);

/*
 * FUNCTION: android::SensorManager::SensorManager(void)
 * USE:      INTERPOSE: construct a sensor manager object
 * NOTES:    This constructor no longer exists in M, instead now one must pass
 *           in a package name as a "string16" to the consrtuctor. Since this
 *           lib only services camera library, it is easy for us to just do that
 *           and this provide the constructor that the camera library wants.
 *           The package name we use if "camera.msm8952". Why not?
 */
void _ZN7android13SensorManagerC1Ev(void *sensorMgr)
{
    void *string;

    _ZN7android8String16C1EPKc(&string, "camera.msm8952");
    _ZN7android13SensorManagerC1ERKNS_8String16E(sensorMgr, &string);
    _ZN7android8String16D1Ev(&string);
}

int _ZN7android5Fence4waitEj(unsigned int timeout) {
    return _ZN7android5Fence4waitEi(timeout);
}
