#
# Copyright (C) 2016 The CyanogenMod Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import os

TARGET_DIR = os.getenv('OUT')

def FullOTA_InstallEnd(self):
  self.output_zip.write(os.path.join(TARGET_DIR, "firmware-update/adspso.bin"), "firmware-update/adspso.bin")
  self.output_zip.write(os.path.join(TARGET_DIR, "firmware-update/cmnlib.mbn"), "firmware-update/cmnlib.mbn")
  self.output_zip.write(os.path.join(TARGET_DIR, "firmware-update/devinfo.img"), "firmware-update/devinfo.img")
  self.output_zip.write(os.path.join(TARGET_DIR, "firmware-update/emmc_appsboot.mbn"), "firmware-update/emmc_appsboot.mbn")
  self.output_zip.write(os.path.join(TARGET_DIR, "firmware-update/hyp.mbn"), "firmware-update/hyp.mbn")
  self.output_zip.write(os.path.join(TARGET_DIR, "firmware-update/keymaster.mbn"), "firmware-update/keymaster.mbn")
  self.output_zip.write(os.path.join(TARGET_DIR, "firmware-update/mdtp.img"), "firmware-update/mdtp.img")
  self.output_zip.write(os.path.join(TARGET_DIR, "firmware-update/NON-HLOS.bin"), "firmware-update/NON-HLOS.bin")
  self.output_zip.write(os.path.join(TARGET_DIR, "firmware-update/rpm.mbn"), "firmware-update/rpm.mbn")
  self.output_zip.write(os.path.join(TARGET_DIR, "firmware-update/sbl1.mbn"), "firmware-update/sbl1.mbn")
  self.output_zip.write(os.path.join(TARGET_DIR, "firmware-update/tz.mbn"), "firmware-update/tz.mbn")

  self.script.AppendExtra('ui_print("update image keymaster.mbn...");')
  self.script.AppendExtra('package_extract_file("firmware-update/keymaster.mbn", "/dev/block/bootdevice/by-name/keymaster");')
  self.script.AppendExtra('ui_print("update image cmnlib.mbn...");')
  self.script.AppendExtra('package_extract_file("firmware-update/cmnlib.mbn", "/dev/block/bootdevice/by-name/cmnlib");')
  self.script.AppendExtra('ui_print("update image adspso.bin...");')
  self.script.AppendExtra('package_extract_file("firmware-update/adspso.bin", "/dev/block/bootdevice/by-name/dsp");')
  self.script.AppendExtra('ui_print("update image devinfo.img...");')
  self.script.AppendExtra('package_extract_file("firmware-update/devinfo.img", "/dev/block/bootdevice/by-name/devinfo");')
  self.script.AppendExtra('ui_print("update image NON-HLOS.bin...");')
  self.script.AppendExtra('package_extract_file("firmware-update/NON-HLOS.bin", "/dev/block/bootdevice/by-name/modem");')
  self.script.AppendExtra('ui_print("update image rpm.mbn...");')
  self.script.AppendExtra('package_extract_file("firmware-update/rpm.mbn", "/dev/block/bootdevice/by-name/rpm");')
  self.script.AppendExtra('ui_print("update image mdtp.img...");')
  self.script.AppendExtra('package_extract_file("firmware-update/mdtp.img", "/dev/block/bootdevice/by-name/mdtp");')
  self.script.AppendExtra('ui_print("update image emmc_appsboot.mbn...");')
  self.script.AppendExtra('package_extract_file("firmware-update/emmc_appsboot.mbn", "/dev/block/bootdevice/by-name/aboot");')
  self.script.AppendExtra('ui_print("update image hyp.mbn...");')
  self.script.AppendExtra('package_extract_file("firmware-update/hyp.mbn", "/dev/block/bootdevice/by-name/hyp");')
  self.script.AppendExtra('ui_print("update image tz.mbn...");')
  self.script.AppendExtra('package_extract_file("firmware-update/tz.mbn", "/dev/block/bootdevice/by-name/tz");')
  self.script.AppendExtra('ui_print("update image sbl1.mbn...");')
  self.script.AppendExtra('package_extract_file("firmware-update/sbl1.mbn", "/dev/block/bootdevice/by-name/sbl1")')
