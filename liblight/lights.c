/*
 * Copyright (C) 2014-2016 The CyanogenMod Project
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

//#define LOG_NDEBUG 0
#define LOG_TAG "lights"

#include <cutils/log.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>

#include <sys/types.h>

#include <hardware/lights.h>

#define UNUSED __attribute__((unused))

/******************************************************************************/

static pthread_once_t g_init = PTHREAD_ONCE_INIT;
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;

static struct light_state_t g_attention;
static struct light_state_t g_notification;
static struct light_state_t g_battery;
static struct light_state_t g_buttons;
static uint32_t g_battery_color = 0;

const char *const LCD_FILE
        = "/sys/class/leds/lcd-backlight/brightness";

const char *const BUTTONS_FILE
        = "/sys/class/leds/button-backlight/brightness";

const char *const RED_LED_FILE
        = "/sys/class/leds/red/brightness";

const char *const GREEN_LED_FILE
        = "/sys/class/leds/green/brightness";

const char *const BLUE_LED_FILE
        = "/sys/class/leds/blue/brightness";

const char *const RED_BLINK_FILE
        = "/sys/class/leds/red/blink";

const char *const GREEN_BLINK_FILE
        = "/sys/class/leds/green/blink";

const char *const BLUE_BLINK_FILE
        = "/sys/class/leds/blue/blink";

const char *const RED_BREATH_FILE
        = "/sys/class/leds/red/led_time";

const char *const GREEN_BREATH_FILE
        = "/sys/class/leds/green/led_time";

const char *const BLUE_BREATH_FILE
        = "/sys/class/leds/blue/led_time";

/**
 * device methods
 */

void init_globals(void)
{
    // init the mutex
    pthread_mutex_init(&g_lock, NULL);
}

static int
write_string(const char *path, const char *buffer)
{
    int fd;
    static int already_warned = 0;

    fd = open(path, O_RDWR);
    if (fd >= 0) {
        int bytes = strlen(buffer);
        int amt = write(fd, buffer, bytes);
        close(fd);
        return amt == -1 ? -errno : 0;
    } else {
        if (already_warned == 0) {
            ALOGE("write_string failed to open %s (%s)\n", path, strerror(errno));
            already_warned = 1;
        }
        return -errno;
    }
}

static int
write_int(const char *path, int value)
{
    char buffer[20];
    sprintf(buffer, "%d\n", value);
    return write_string(path, buffer);
}

static int
rgb_to_brightness(const struct light_state_t *state)
{
    int color = state->color & 0x00ffffff;
    return ((77 * ((color >> 16) & 0xff))
            + (150 * ((color >> 8) & 0xff))
            + (29 * (color & 0xff))) >> 8;
}

static int
is_lit(const struct light_state_t *state)
{
    return (state->color & 0x00ffffff);
}

static int
set_speaker_light_locked(struct light_device_t *dev,
        struct light_state_t const *state)
{
    int red, green, blue;
    int blink;
    int onMS, offMS;
    unsigned int colorRGB;
    char breath_pattern[64] = { 0, };

    if (!dev) {
        return -1;
    }

    if (state == NULL) {
        write_int(RED_LED_FILE, 0);
        write_int(GREEN_LED_FILE, 0);
        write_int(BLUE_LED_FILE, 0);
        write_int(RED_BLINK_FILE, 0);
        write_int(GREEN_BLINK_FILE, 0);
        write_int(BLUE_BLINK_FILE, 0);
        return 0;
    }

    switch (state->flashMode) {
        case LIGHT_FLASH_TIMED:
            onMS = state->flashOnMS;
            offMS = state->flashOffMS;
            break;
        case LIGHT_FLASH_NONE:
        default:
            onMS = 0;
            offMS = 0;
            break;
    }

    colorRGB = state->color;

#if 0
    ALOGD("set_speaker_light_locked mode %d, colorRGB=%08X, onMS=%d, offMS=%d\n",
            state->flashMode, colorRGB, onMS, offMS);
#endif

    red = (colorRGB >> 16) & 0xFF;
    green = (colorRGB >> 8) & 0xFF;
    blue = colorRGB & 0xFF;

    if (onMS > 0 && offMS > 0 && !(
          (red == green && green == blue) ||
          (red == green && blue == 0) ||
          (red == blue && green == 0) ||
          (green == blue && red == 0) ||
          (blue == 0 && red == 0) ||
          (green == 0 && red == 0) ||
          (green == 0 && blue == 0)
       )) {
       // Blinking only works if all active component colors have
       // the same brightness value
       offMS = 0;
    }

    if (onMS > 0 && offMS > 0) {
        blink = 1;
        // Make sure the values are at least 1 second. That's the smallest
        // we take
        if (onMS && onMS < 1000) {
            onMS = 1000;
        }
        if (offMS && offMS < 1000) {
            offMS = 1000;
        }
        // ramp up, lit, ramp down, unlit. in seconds.
        sprintf(breath_pattern,"1 %d 1 %d",
                (int)(onMS / 1000), (int)(offMS / 1000));
    } else {
        blink = 0;
        sprintf(breath_pattern,"1 2 1 2");
    }

    write_string(RED_BREATH_FILE, breath_pattern);
    write_int(RED_BLINK_FILE, blink);
    write_string(GREEN_BREATH_FILE, breath_pattern);
    write_int(GREEN_BLINK_FILE, blink);
    write_string(BLUE_BREATH_FILE, breath_pattern);
    write_int(BLUE_BLINK_FILE, blink);

    write_int(RED_LED_FILE, red);
    write_int(GREEN_LED_FILE, green);
    write_int(BLUE_LED_FILE, blue);

    return 0;
}

static void
handle_speaker_light_locked(struct light_device_t *dev)
{
    if (is_lit(&g_buttons)) {
        set_speaker_light_locked(dev, &g_buttons);
    } else if (is_lit(&g_attention)) {
        set_speaker_light_locked(dev, &g_attention);
    } else if (is_lit(&g_notification)) {
        set_speaker_light_locked(dev, &g_notification);
    } else if (is_lit(&g_battery)) {
        set_speaker_light_locked(dev, &g_battery);
    } else {
        set_speaker_light_locked(dev, NULL);
    }
}

static int
set_light_backlight(UNUSED struct light_device_t *dev,
        const struct light_state_t *state)
{
    int err = 0;
    int brightness = rgb_to_brightness(state);
    bool is_display_on = (brightness > 0) ? true : false;

    pthread_mutex_lock(&g_lock);
    err = write_int(LCD_FILE, brightness);
    g_battery.color = is_display_on ? 0 : g_battery_color;
    handle_speaker_light_locked(dev);
    pthread_mutex_unlock(&g_lock);

    return err;
}

static int
set_light_buttons(UNUSED struct light_device_t *dev,
        const struct light_state_t *state)
{
    int err = 0;
    int brightness = rgb_to_brightness(state);
    pthread_mutex_lock(&g_lock);
    g_buttons = *state;
    g_buttons.color = brightness ? 0x00ffffff : 0;
    err = write_int(BUTTONS_FILE, brightness);
    handle_speaker_light_locked(dev);
    pthread_mutex_unlock(&g_lock);

    return err;
}

static int
set_light_notifications(UNUSED struct light_device_t *dev,
        const struct light_state_t *state)
{
    pthread_mutex_lock(&g_lock);
    g_notification = *state;
    handle_speaker_light_locked(dev);
    pthread_mutex_unlock(&g_lock);

    return 0;
}

static int
set_light_attention(UNUSED struct light_device_t *dev,
        const struct light_state_t *state)
{
    pthread_mutex_lock(&g_lock);
    g_attention = *state;
    handle_speaker_light_locked(dev);
    pthread_mutex_unlock(&g_lock);

    return 0;
}

static int
set_light_battery(UNUSED struct light_device_t *dev,
        const struct light_state_t *state)
{
    pthread_mutex_lock(&g_lock);
    g_battery = *state;
    g_battery_color = g_battery.color;
    handle_speaker_light_locked(dev);
    pthread_mutex_unlock(&g_lock);

    return 0;
}

/** Close the lights device */
static int
close_lights(struct light_device_t *dev)
{
    if (dev) {
        free(dev);
    }
    return 0;
}

/******************************************************************************/

/**
 * module methods
 */

/** Open a new instance of a lights device using name */
static int open_lights(const struct hw_module_t *module, const char *name,
        struct hw_device_t **device)
{
    int (*set_light)(struct light_device_t *dev,
            const struct light_state_t *state);

    if (0 == strcmp(LIGHT_ID_BACKLIGHT, name))
        set_light = set_light_backlight;
    else if (0 == strcmp(LIGHT_ID_BUTTONS, name))
        set_light = set_light_buttons;
    else if (0 == strcmp(LIGHT_ID_NOTIFICATIONS, name))
        set_light = set_light_notifications;
    else if (0 == strcmp(LIGHT_ID_ATTENTION, name))
        set_light = set_light_attention;
    else if (0 == strcmp(LIGHT_ID_BATTERY, name))
        set_light = set_light_battery;
    else
        return -EINVAL;

    pthread_once(&g_init, init_globals);

    struct light_device_t *dev = malloc(sizeof(struct light_device_t));
    memset(dev, 0, sizeof(*dev));

    dev->common.tag = HARDWARE_DEVICE_TAG;
    dev->common.version = 0;
    dev->common.module = (struct hw_module_t*)module;
    dev->common.close = (int (*)(struct hw_device_t*))close_lights;
    dev->set_light = set_light;

    *device = (struct hw_device_t*)dev;
    return 0;
}

static struct hw_module_methods_t lights_module_methods = {
    .open =  open_lights,
};

/*
 * The lights Module
 */
struct hw_module_t HAL_MODULE_INFO_SYM = {
    .tag = HARDWARE_MODULE_TAG,
    .version_major = 1,
    .version_minor = 0,
    .id = LIGHTS_HARDWARE_MODULE_ID,
    .name = "HM Note 3 Lights Module",
    .author = "The CyanogenMod Project",
    .methods = &lights_module_methods,
};
