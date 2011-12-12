/* audio_hw.c
 **
 ** Copyright 2009 Texas Instruments
 **
 ** Licensed under the Apache License, Version 2.0 (the "License");
 ** you may not use this file except in compliance with the License.
 ** You may obtain a copy of the License at
 **
 **     http://www.apache.org/licenses/LICENSE-2.0
 **
 ** Unless required by applicable law or agreed to in writing, software
 ** distributed under the License is distributed on an "AS IS" BASIS,
 ** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 ** See the License for the specific language governing permissions and
 ** limitations under the License.
 */

/* audio_hw.c for encore
 *
 * Work done by Gerad Munsch <gmunsch@unforgivendevelopment.com>
 *
 * Work based off of:
 * device/bn/encore/audio_hal/alsa_omap3.cpp    [gingerbread]
 * device/samsung/tuna/audio/audio_hw.c         [ics]
 */

// define log tag
#define LOG_TAG "alsa_omap3_ics"

// includes
#include <errno.h>
#include <pthread.h>
#include <stdint.h>
#include <sys/time.h>
#include <stdlib.h>

#include <cutils/log.h>
#include <cutils/str_parms.h>
#include <cutils/properties.h>

#include <hardware/hardware.h>
#include <system/audio.h>
#include <hardware/audio.h>

#include <tinyalsa/asoundlib.h>
#include <audio_utils/resampler.h>
#include <audio_utils/echo_reference.h>
#include <hardware/audio_effect.h>
#include <audio_effects/effect_aec.h>

// defines from original
#define BLUETOOTH_SCO_DEVICE "hw:0,2"
#define FM_TRANSMIT_DEVICE "hw:0,3"
#define EXT_USB_DEVICE "hw:1,0"

#ifndef ALSA_DEFAULT_SAMPLE_RATE
#define ALSA_DEFAULT_SAMPLE_RATE 44100 // in Hz
#endif

#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))

/* defines for "audio_hw.c" / tinyalsa config */
#define DEFAULT_OUT_SAMPLING_RATE 44100
#define SPK_FULL_POWER_SAMPLING_RATE 48000

// alsa cards
#define CARD_OMAP3_MAIN 0
#define CARD_ENCORE_DEFAULT CARD_OMAP3_MAIN

// alsa devices
#define PORT_SPK 0
#define PORT_IN 1
#define PORT_PCM 2
#define PORT_FM 3

// misc
/* constraint imposed by ABE: all period sizes must be multiples of 24 */
#define ABE_BASE_FRAME_COUNT 24
/* number of base blocks in a short period (low latency) */
#define SHORT_PERIOD_MULTIPLIER 44  /* 22 ms */
/* number of frames per short period (low latency) */
#define SHORT_PERIOD_SIZE (ABE_BASE_FRAME_COUNT * SHORT_PERIOD_MULTIPLIER)
/* number of short periods in a long period (low power) */
#define LONG_PERIOD_MULTIPLIER 14  /* 308 ms */
/* number of frames per long period (low power) */
#define LONG_PERIOD_SIZE (SHORT_PERIOD_SIZE * LONG_PERIOD_MULTIPLIER)
/* number of periods for low power playback */
#define PLAYBACK_LONG_PERIOD_COUNT 2
/* number of pseudo periods for low latency playback */
#define PLAYBACK_SHORT_PERIOD_COUNT 4
/* number of periods for capture */
#define CAPTURE_PERIOD_COUNT 2
/* minimum sleep time in out_write() when write threshold is not reached */
#define MIN_WRITE_SLEEP_US 5000

#define RESAMPLER_BUFFER_FRAMES (SHORT_PERIOD_SIZE * 2)
#define RESAMPLER_BUFFER_SIZE (4 * RESAMPLER_BUFFER_FRAMES)



static int adev_open(const hw_module_t*, const char*, hw_device_t**);
static int adev_close(hw_device_t*);

static struct hw_module_methods_t hal_module_methods = {
    .open = adev_open,
};

struct audio_module HAL_MODULE_INFO_SYM = {
    .common = {
        .tag            = HARDWARE_MODULE_TAG,
        .version_major  = 1,
        .version_minor  = 0,
        .id             = AUDIO_HARDWARE_MODULE_ID,
        .name           = "Omap3 ICS ALSA module",
        .author         = "Texas Instruments, Gerad Munsch",
        .methods        = &hal_module_methods, //&s_module_methods
    },
};

struct omap3_audio_device {
    struct audio_hw_device hw_device;

    pthread_mutex_t lock;   // see note below on mutex acquisition order

    struct mixer *mixer;
    //struct mixer_ctls mixer_ctls;
    int mode;
    int devices;

    struct omap3_stream_out *active_output;
};

struct omap3_stream_out {
    struct audio_stream_out stream;

    pthread_mutex_t lock;       // see note below on mutex acquisition order
    struct pcm_config config;
    struct pcm *pcm;
    struct resampler_itfe *resampler;
    char *buffer;
    int standby;
    struct omap3_audio_device *dev;
    int write_threshold;
    bool low_power;
};

struct pcm_config pcm_config_mm = {
    .channels       = 2,
    .rate           = SPK_FULL_POWER_SAMPLING_RATE,
    .format         = PCM_FORMAT_S16_LE,
};

//struct route_setting
//{
//    char *ctl_name;
//    int intval;
//    char *strval;
//};

static int adev_open(const hw_module_t* module, const char* name,
                     hw_device_t** device)
{
    struct omap3_audio_device *adev; // check this out
    int ret;

    if (strcmp(name, AUDIO_HARDWARE_INTERFACE) != 0)
        return -EINVAL;

    adev = calloc(1, sizeof(struct omap3_audio_device));
    if (!adev)
        return -ENOMEM;

    adev->hw_device.common.tag = HARDWARE_DEVICE_TAG;
    adev->hw_device.common.version = 0;
    adev->hw_device.common.module = (struct hw_module_t *) module;
    adev->hw_device.common.close = adev_close;

    adev->mixer = mixer_open(0);
    if (!adev->mixer) {
        free(adev);
        LOGE("Unable to open the mixer, aborting.");
        return -EINVAL;
    }

    return 0;
}

static int adev_close(hw_device_t *device)
{
    free(device);
    return 0;
}

// ----------------------------------------------------------------------------
// - [START] REWRITE FOR TINYALSA BY GERAD MUNSCH
// ----------------------------------------------------------------------------

static uint32_t out_get_sample_rate(const struct audio_stream *stream)
{
    return DEFAULT_OUT_SAMPLING_RATE;
}

static uint32_t out_get_channels(const struct audio_stream *stream)
{
    return AUDIO_CHANNEL_OUT_STEREO;
}

static int out_get_format(const struct audio_stream *stream)
{
    return AUDIO_FORMAT_PCM_16_BIT;
}

static int out_set_format(struct audio_stream *stream, int format)
{
    return 0;
}

static int start_output_stream(struct omap3_stream_out *out)
{
    struct omap3_audio_device *adev = out->dev;
    unsigned int card = CARD_ENCORE_DEFAULT;
    unsigned int port = PORT_SPK;

    adev->active_output = out;

    out->config.rate = SPK_FULL_POWER_SAMPLING_RATE;
    out->write_threshold = PLAYBACK_LONG_PERIOD_COUNT * LONG_PERIOD_SIZE;
    out->config.start_threshold = SHORT_PERIOD_SIZE * 2;
    out->config.avail_min = LONG_PERIOD_SIZE;
    out->low_power = 1;

    out->pcm = pcm_open(card, port, PCM_OUT | PCM_MMAP | PCM_NOIRQ, &out->config);

    if (!pcm_is_ready(out->pcm)) {
        LOGE("cannot open pcm_out driver: %s", pcm_get_error(out->pcm));
        pcm_close(out->pcm);
        adev->active_output = NULL;
        return -ENOMEM;
    }

    out->resampler->reset(out->resampler);

    return 0;
}


// ----------------------------------------------------------------------------
// - [END] REWRITE FOR TINYALSA BY GERAD MUNSCH
// ----------------------------------------------------------------------------
