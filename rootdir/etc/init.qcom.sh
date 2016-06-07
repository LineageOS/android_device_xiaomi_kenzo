#!/system/bin/sh
# Copyright (c) 2009-2015, The Linux Foundation. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of The Linux Foundation nor
#       the names of its contributors may be used to endorse or promote
#       products derived from this software without specific prior written
#       permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

export PATH=/system/bin

# Set platform variables
target=`getprop ro.board.platform`
if [ -f /sys/devices/soc0/soc_id ]; then
    platformid=`cat /sys/devices/soc0/soc_id`
else
    platformid=`cat /sys/devices/system/soc/soc0/id`
fi

#
# Function to start sensors for DSPS enabled platforms
#
start_sensors()
{
    if [ -c /dev/msm_dsps -o -c /dev/sensors ]; then
        chmod -h 775 /persist/sensors
        chmod -h 664 /persist/sensors/sensors_settings
        chown -h system.root /persist/sensors/sensors_settings

        mkdir -p /data/misc/sensors
        chmod -h 775 /data/misc/sensors

        start sensors
    fi
}

start_msm_irqbalance_8952()
{
	if [ -f /system/bin/msm_irqbalance ]; then
		case "$platformid" in
		    "239" | "241" | "263" | "264" | "268" | "269" | "270" | "271")
			start msm_irqbalance;;
		esac
		case "$platformid" in
			"266" | "274" | "277" | "278")
			start msm_irqbal_lb;;
		esac
	fi
}

start_copying_prebuilt_qcril_db()
{
    if [ -f /system/vendor/qcril.db -a ! -f /data/misc/radio/qcril.db ]; then
        cp /system/vendor/qcril.db /data/misc/radio/qcril.db
        chown -h radio.radio /data/misc/radio/qcril.db
    fi
}

baseband=`getprop ro.baseband`
echo 1 > /proc/sys/net/ipv6/conf/default/accept_ra_defrtr

start_sensors
start_copying_prebuilt_qcril_db
start_msm_irqbalance_8952

if [ -f /sys/class/graphics/fb0/modes ]; then
	panel_res=`cat /sys/class/graphics/fb0/modes`
	if [ "${panel_res:5:1}" == "x" ]; then
		panel_xres=${panel_res:2:3}
	else
		panel_xres=${panel_res:2:4}
	fi
fi

bootmode=`getprop ro.bootmode`
emmc_boot=`getprop ro.boot.emmc`
case "$emmc_boot"
    in "true")
        if [ "$bootmode" != "charger" ]; then # start rmt_storage and rfs_access
            start rmt_storage
            start rfs_access
        fi
    ;;
esac

#
# Make modem config folder and copy firmware config to that folder
#
rm -rf /data/misc/radio/modem_config
mkdir /data/misc/radio/modem_config
chmod 770 /data/misc/radio/modem_config
cp -r /firmware/image/modem_pr/mcfg/configs/* /data/misc/radio/modem_config 
chown -hR radio.radio /data/misc/radio/modem_config
echo 1 > /data/misc/radio/copy_complete
chown radio:radio /data/misc/radio/copy_complete
