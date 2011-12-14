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
//* mixer control names
// volume
#define MIXER_HEADSET_PLAYBACK_VOLUME           "HP Analog Volume"
#define MIXER_HANDSFREE_PLAYBACK_VOLUME         "SPKR Analog Volume"

// headset
#define MIXER_HS_LEFT_PLAYBACK                  "HPL Output Mixer"
#define MIXER_HS_RIGHT_PLAYBACK                 "HPR Output Mixer"
// speaker
#define MIXER_HF_LEFT_PLAYBACK                  "LOL Output Mixer"
#define MIXER_HF_RIGHT_PLAYBACK                 "LOR Output Mixer"
// headphone enable switch
//#define MIXER_EARPHONE_ENABLE_SWITCH            

//* mixer contol gain and route values
// headset
#define MIXER_PLAYBACK_HS_LEFT_DAC              "Left DAC"
#define MIXER_PLAYBACK_HS_RIGHT_DAC             "Right DAC"
// speaker
#define MIXER_PLAYBACK_HF_LEFT_DAC              "Left DAC"
#define MIXER_PLAYBACK_HF_RIGHT_DAC             "Right DAC"

// mixer stuff FIXME: COMBINE INTO ABOVE SECTION
#define MIXER_ABE_GAIN_0DB                      120

// sampling rates
#define DEFAULT_OUT_SAMPLING_RATE 44100
#define MM_FULL_POWER_SAMPLING_RATE 48000

// alsa cards
#define CARD_OMAP3_MAIN 0
#define CARD_ENCORE_DEFAULT CARD_OMAP3_MAIN

// alsa devices
#define PORT_SPK 0
#define PORT_IN 1
#define PORT_PCM 2
#define PORT_FM 3

// dB conversions
#define DB_FROM_SPEAKER_VOLUME(x) ((x) * 2 - 52)
#define DB_TO_HEADSET_VOLUME(x) (((x) + 30) / 2)
#define DB_TO_SPEAKER_VOLUME(x) (((x) + 52) / 2)

// volume levels
#define NORMAL_SPEAKER_VOLUME_OMAP3 4
#define NORMAL_HEADSET_VOLUME_OMAP3 -12
#define NORMAL_HEADPHONE_VOLUME_OMAP3 -6 // allow louder output for headphones

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
#define CAPTURE_PERIOD_COUNT 2
/* minimum sleep time in out_write() when write threshold is not reached */
#define MIN_WRITE_SLEEP_US 5000

#define RESAMPLER_BUFFER_FRAMES (SHORT_PERIOD_SIZE * 2)
#define RESAMPLER_BUFFER_SIZE (4 * RESAMPLER_BUFFER_FRAMES)



//static int adev_open(const hw_module_t*, const char*, hw_device_t**);
//static int adev_close(hw_device_t*);

struct pcm_config pcm_config_mm = {
    .channels = 2,
    .rate = MM_FULL_POWER_SAMPLING_RATE,
    .period_size = LONG_PERIOD_SIZE,
    .period_count = PLAYBACK_LONG_PERIOD_COUNT,
    .format = PCM_FORMAT_S16_LE,
};

struct route_setting
{
    char *ctl_name;
    int intval;
    char *strval;
};

/* 
 * ROUTE SETTINGS
 * XXX: THESE VALUES SHOULD NOT CHANGE
 */

struct route_setting defaults[] = {
    {
        .ctl_name = NULL,
    },
};

/* hands-free (speaker) output */
struct route_setting hf_output[] = {
    /*{
        .ctl_name = "Left Playback",
        .strval = "Left DAC",
    },
    {
        .ctl_name = "Right Playback",
        .strval = "Right DAC",
    },*/
    {
        .ctl_name = "SPKR Analog",
        .strval = "LO DAC",
    },
    {
        .ctl_name = "LO DAC Playback Switch",
        .intval = 1,
    },
    {
        .ctl_name = NULL,
    },
};

/* headphone/headset output */
struct route_setting hs_output[] = {
    /*{
        .ctl_name = "HandsfreeR Mux",
        .strval = "AudioR2",
    },
    {
        .ctl_name = "HandsfreeL Mux",
        .strval = "AudioL2",
    },
    {
        .ctl_name = "HP DAC Playback Switch",
        .intval = 1,
    },*/
    {
        .ctl_name = NULL,
    },
};

struct mixer_ctls
{
    struct mixer_ctl *headset_volume;
    struct mixer_ctl *speaker_volume;
};

struct omap3_audio_device {
    struct audio_hw_device hw_device;

    pthread_mutex_t lock;   // see note below on mutex acquisition order

    struct mixer *mixer;
    struct mixer_ctls mixer_ctls;
    int mode;
    int devices;

    struct omap3_stream_out *active_output;
    bool mic_mute;

    bool bluetooth_nrec;

    bool low_power;
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

static void select_output_device(struct omap3_audio_device *adev);

/* The enable flag when 0 makes the assumption that enums are disabled by
 * "Off" and integers/booleans by 0 */
static int set_route_by_array(struct mixer *mixer, struct route_setting *route,
                              int enable)
{
    LOGD("DEBUG: Entered set_route_by_array()");
    struct mixer_ctl *ctl;
    unsigned int i, j;

    /* Go through the route array and set each value */
    i = 0;
    while (route[i].ctl_name) {
        ctl = mixer_get_ctl_by_name(mixer, route[i].ctl_name);
        if (!ctl)
            LOGE("DEBUG: ERROR in set_route_by_array() -- ctl is invalid");
            LOGE("DEBUG: route[%s].ctl_name = '%s'", i, route[i].ctl_name);
            return -EINVAL;

        if (route[i].strval) {
            if (enable)
                mixer_ctl_set_enum_by_string(ctl, route[i].strval);
            else
                mixer_ctl_set_enum_by_string(ctl, "Off");
        } else {
            /* This ensures multiple (i.e. stereo) values are set jointly */
            for (j = 0; j < mixer_ctl_get_num_values(ctl); j++) {
                if (enable)
                    mixer_ctl_set_value(ctl, j, route[i].intval);
                else
                    mixer_ctl_set_value(ctl, j, 0);
            }
        }
        LOGD("DEBUG: OK! route[%s].ctl_name = '%s'", i, route[i].ctl_name);
        i++;
    }

    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: set_route_by_array() complete!");

    return 0;
}


// ----------------------------------------------------------------------------
// - [START] REWRITE FOR TINYALSA BY GERAD MUNSCH
// ----------------------------------------------------------------------------

static void set_output_volumes(struct omap3_audio_device *adev)
{
    // FIXME: THIS SHIT NEEDS FIXED
    unsigned int channel;
    int speaker_volume;
    int headset_volume;
    int headphone_on = adev->devices & AUDIO_DEVICE_OUT_WIRED_HEADPHONE;
    int speaker_on = adev->devices & AUDIO_DEVICE_OUT_SPEAKER;
    int speaker_volume_overrange = MIXER_ABE_GAIN_0DB;
    int speaker_max_db =
        DB_FROM_SPEAKER_VOLUME(mixer_ctl_get_range_max(adev->mixer_ctls.speaker_volume));
    //struct mixer_ctl *mixer_ctl_overrange = adev->mixer_ctls.mm_dl2_volume;

    // since we're not a phone, I'm just going to set up media volumes here for now...
    speaker_volume = NORMAL_SPEAKER_VOLUME_OMAP3;
    if (headphone_on)
        headset_volume = NORMAL_HEADPHONE_VOLUME_OMAP3;
    else
        headset_volume = NORMAL_HEADSET_VOLUME_OMAP3;

    //if (adev->mode == AUDIO_MODE_RINGTONE)
        //headset_volume += RINGTONE_HEADSET_VOLUME_OFFSET;

    /* If we have run out of range in the codec (analog) speaker volume,
       we have to apply the remainder of the dB increase to the DL2
       media/voice mixer volume, which is a digital gain */
    if (speaker_volume > speaker_max_db) {
        speaker_volume_overrange += (speaker_volume - speaker_max_db);
        speaker_volume = speaker_max_db;
    }

    for (channel = 0; channel < 2; channel++) {
        mixer_ctl_set_value(adev->mixer_ctls.speaker_volume, channel,
            DB_TO_SPEAKER_VOLUME(speaker_volume));
        mixer_ctl_set_value(adev->mixer_ctls.headset_volume, channel,
            DB_TO_HEADSET_VOLUME(headset_volume));
    }

    //if (speaker_on)
        //mixer_ctl_set_value(mixer_ctl_overrange, 0, speaker_volume_overrange);
    //else
        //mixer_ctl_set_value(mixer_ctl_overrange, 0, MIXER_ABE_GAIN_0DB);

    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: set_output_volumes() complete!");

}

static int start_output_stream(struct omap3_stream_out *out)
{
    struct omap3_audio_device *adev = out->dev;
    unsigned int card = CARD_ENCORE_DEFAULT;
    unsigned int port = PORT_SPK;

    adev->active_output = out;

    out->config.rate = MM_FULL_POWER_SAMPLING_RATE;
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

    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: start_output_stream() complete!");

    return 0;
}

static void select_mode(struct omap3_audio_device *adev)
{
    // XXX: I STUBBED THIS BITCH OUT!

    /*if (adev->mode == AUDIO_MODE_IN_CALL) {
        LOGE("Entering IN_CALL state, in_call=%d", adev->in_call);
        if (!adev->in_call) {
            force_all_standby(adev);*/
            /* force earpiece route for in call state if speaker is the
            only currently selected route. This prevents having to tear
            down the modem PCMs to change route from speaker to earpiece
            after the ringtone is played, but doesn't cause a route
            change if a headset or bt device is already connected. If
            speaker is not the only thing active, just remove it from
            the route. We'll assume it'll never be used initally during
            a call. This works because we're sure that the audio policy
            manager will update the output device after the audio mode
            change, even if the device selection did not change. */
            /*if ((adev->devices & AUDIO_DEVICE_OUT_ALL) == AUDIO_DEVICE_OUT_SPEAKER)
                adev->devices = AUDIO_DEVICE_OUT_EARPIECE |
                                AUDIO_DEVICE_IN_BUILTIN_MIC;
            else
                adev->devices &= ~AUDIO_DEVICE_OUT_SPEAKER;
            select_output_device(adev);
            start_call(adev);
            ril_set_call_clock_sync(&adev->ril, SOUND_CLOCK_START);
            adev_set_voice_volume(&adev->hw_device, adev->voice_volume);
            adev->in_call = 1;
        }
    } else {
        LOGE("Leaving IN_CALL state, in_call=%d, mode=%d",
             adev->in_call, adev->mode);
        if (adev->in_call) {
            adev->in_call = 0;
            end_call(adev);
            force_all_standby(adev);
            select_output_device(adev);
            select_input_device(adev);
        }
    }*/

    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: select_mode() (STUB) complete!");

}

static void select_output_device(struct omap3_audio_device *adev)
{
    int headset_on;
    int headphone_on;
    int speaker_on;
    int bt_on;
    unsigned int channel;

    headset_on = adev->devices & AUDIO_DEVICE_OUT_WIRED_HEADSET;
    headphone_on = adev->devices & AUDIO_DEVICE_OUT_WIRED_HEADPHONE;
    speaker_on = adev->devices & AUDIO_DEVICE_OUT_SPEAKER;
    bt_on = adev->devices & AUDIO_DEVICE_OUT_ALL_SCO;

    // mixer_ctl_set_value stuff

    // set_route_by_array stuff
    set_route_by_array(adev->mixer, hs_output, headset_on | headphone_on);
    set_route_by_array(adev->mixer, hf_output, speaker_on);

    //set_eq_filter(adev);
    set_output_volumes(adev);

    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: select_output_device() complete!");

}

static int adev_set_master_volume(struct audio_hw_device *dev, float volume)
{
    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: adev_set_master_volume() complete!");

    return -ENOSYS;
}

// XXX: MAY NOT BE NEEDED
static int adev_set_mode(struct audio_hw_device *dev, int mode)
{
    struct omap3_audio_device *adev = (struct omap3_audio_device *)dev;

    pthread_mutex_lock(&adev->lock);
    if (adev->mode != mode) {
        adev->mode = mode;
        select_mode(adev);
    }
    pthread_mutex_unlock(&adev->lock);

    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: adev_set_mode() complete!");

    return 0;
}

static int adev_set_mic_mute(struct audio_hw_device *dev, bool state)
{
    struct omap3_audio_device *adev = (struct omap3_audio_device *)dev;

    adev->mic_mute = state;

    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: adev_set_mic_mute() complete!");

    return 0;
}

static int adev_get_mic_mute(const struct audio_hw_device *dev, bool *state)
{
    LOGD("DEBUG: adev_get_mic_mute() begin!");

    struct omap3_audio_device *adev = (struct omap3_audio_device *)dev;

    //*state = adev->mic_mute;
    *state = false;

    /* XXX | DEBUG | XXX */
    LOGI("DEBUG: adev_get_mic_mute() OVERRIDE -- FALSE");
    LOGD("DEBUG: adev_get_mic_mute() complete!");

    return 0;
}

static uint32_t out_get_sample_rate(const struct audio_stream *stream)
{
    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: out_get_sample_rate() complete!");

    return DEFAULT_OUT_SAMPLING_RATE;
}

static int out_set_sample_rate(struct audio_stream *stream, uint32_t rate)
{
    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: out_set_sample_rate() complete!");

    return 0;
}

static size_t out_get_buffer_size(const struct audio_stream *stream)
{
    struct omap3_stream_out *out = (struct omap3_stream_out *)stream;

    size_t size = (SHORT_PERIOD_SIZE * DEFAULT_OUT_SAMPLING_RATE) / out->config.rate;
    size = ((size + 15) / 16) * 16;

    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: out_get_buffer_size() complete!");

    return size * audio_stream_frame_size((struct audio_stream *)stream);
}

static uint32_t out_get_channels(const struct audio_stream *stream)
{

    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: out_get_channels() complete!");

    return AUDIO_CHANNEL_OUT_STEREO;
}

static int out_get_format(const struct audio_stream *stream)
{

    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: out_get_format() complete!");

    return AUDIO_FORMAT_PCM_16_BIT;
}

static int out_set_format(struct audio_stream *stream, int format)
{

    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: out_set_format() complete!");

    return 0;
}

/* must be called with hw device and output stream mutexes locked */
static int do_output_standby(struct omap3_stream_out *out)
{
    struct omap3_audio_device *adev = out->dev;

    if (!out->standby) {
        pcm_close(out->pcm);
        out->pcm = NULL;

        adev->active_output = 0;

        // XXX: STUB THIS SHIT OUT FOR NAO
        /* if in call, don't turn off the output stage. This will
        be done when the call is ended */
        //if (adev->mode != AUDIO_MODE_IN_CALL) {
            /* FIXME: only works if only one output can be active at a time */
            //set_route_by_array(adev->mixer, hs_output, 0);
            //set_route_by_array(adev->mixer, hf_output, 0);
        //}

        /* stop writing to echo reference */
        //if (out->echo_reference != NULL) {
            //out->echo_reference->write(out->echo_reference, NULL);
            //out->echo_reference = NULL;
        //}

        // turn off outputs
        set_route_by_array(adev->mixer, hs_output, 0);
        set_route_by_array(adev->mixer, hf_output, 0);

        out->standby = 1;
    }

    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: do_output_standby() complete!");

    return 0;
}

static int out_standby(struct audio_stream *stream)
{
    struct omap3_stream_out *out = (struct omap3_stream_out *)stream;
    int status;

    pthread_mutex_lock(&out->dev->lock);
    pthread_mutex_lock(&out->lock);
    status = do_output_standby(out);
    pthread_mutex_unlock(&out->lock);
    pthread_mutex_unlock(&out->dev->lock);

    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: out_standby() complete!");

    return status;
}

static int out_dump(const struct audio_stream *stream, int fd)
{

    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: out_dump() complete!");

    return 0;
}

static int out_set_parameters(struct audio_stream *stream, const char *kvpairs)
{
    struct omap3_stream_out *out = (struct omap3_stream_out *)stream;
    struct omap3_audio_device *adev = out->dev;
    //struct omap3_stream_in *in;
    struct str_parms *parms;
    char *str;
    char value[32];
    int ret, val = 0;
    bool force_input_standby = false;

    parms = str_parms_create_str(kvpairs);

    ret = str_parms_get_str(parms, AUDIO_PARAMETER_STREAM_ROUTING, value, sizeof(value));
    if (ret >= 0) {
        val = atoi(value);
        pthread_mutex_lock(&adev->lock);
        pthread_mutex_lock(&out->lock);
        if (((adev->devices & AUDIO_DEVICE_OUT_ALL) != val) && (val != 0)) {
            if (out == adev->active_output) {
                /* a change in output device may change the microphone selection */
                /*if (adev->active_input &&
                        adev->active_input->source == AUDIO_SOURCE_VOICE_COMMUNICATION) {
                    force_input_standby = true;
                }*/
                /* force standby if moving to/from HDMI */
                if (((val & AUDIO_DEVICE_OUT_AUX_DIGITAL) ^
                        (adev->devices & AUDIO_DEVICE_OUT_AUX_DIGITAL)) ||
                        ((val & AUDIO_DEVICE_OUT_DGTL_DOCK_HEADSET) ^
                        (adev->devices & AUDIO_DEVICE_OUT_DGTL_DOCK_HEADSET)))
                    do_output_standby(out);
            }
            adev->devices &= ~AUDIO_DEVICE_OUT_ALL;
            adev->devices |= val;
            select_output_device(adev);
        }
        pthread_mutex_unlock(&out->lock);
        /*if (force_input_standby) {
            in = adev->active_input;
            pthread_mutex_lock(&in->lock);
            do_input_standby(in);
            pthread_mutex_unlock(&in->lock);
        }*/
        pthread_mutex_unlock(&adev->lock);
    }

    str_parms_destroy(parms);

    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: out_set_parameters() complete!");

    return ret;
}

static char * out_get_parameters(const struct audio_stream *stream, const char *keys)
{

    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: out_get_parameters() complete!");

    return strdup("");
}

static uint32_t out_get_latency(const struct audio_stream_out *stream)
{
    struct omap3_stream_out *out = (struct omap3_stream_out *)stream;

    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: out_get_latency() complete!");

    return (SHORT_PERIOD_SIZE * PLAYBACK_SHORT_PERIOD_COUNT * 1000) / out->config.rate;
}

static int out_set_volume(struct audio_stream_out *stream, float left,
                          float right)
{

    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: out_set_volume() complete!");

    return -ENOSYS;
}

static ssize_t out_write(struct audio_stream_out *stream, const void* buffer,
                         size_t bytes)
{
    int ret;
    struct omap3_stream_out *out = (struct omap3_stream_out *)stream;
    struct omap3_audio_device *adev = out->dev;
    size_t frame_size = audio_stream_frame_size(&out->stream.common);
    size_t in_frames = bytes / frame_size;
    size_t out_frames = RESAMPLER_BUFFER_SIZE / frame_size;
    bool force_input_standby = false;
    //struct tuna_stream_in *in;
    bool low_power;
    int kernel_frames;
    void *buf;

    /* acquiring hw device mutex systematically is useful if a low priority thread is waiting
     * on the output stream mutex - e.g. executing select_mode() while holding the hw device
     * mutex
     */
    pthread_mutex_lock(&adev->lock);
    pthread_mutex_lock(&out->lock);
    if (out->standby) {
        ret = start_output_stream(out);
        if (ret != 0) {
            pthread_mutex_unlock(&adev->lock);
            goto exit;
        }
        out->standby = 0;
        /* a change in output device may change the microphone selection */
        //if (adev->active_input &&
                //adev->active_input->source == AUDIO_SOURCE_VOICE_COMMUNICATION)
            //force_input_standby = true;
    }
    low_power = adev->low_power;// && !adev->active_input;
    pthread_mutex_unlock(&adev->lock);

    if (low_power != out->low_power) {
        if (low_power) {
            out->write_threshold = LONG_PERIOD_SIZE * PLAYBACK_LONG_PERIOD_COUNT;
            out->config.avail_min = LONG_PERIOD_SIZE;
        } else {
            out->write_threshold = SHORT_PERIOD_SIZE * PLAYBACK_SHORT_PERIOD_COUNT;
            out->config.avail_min = SHORT_PERIOD_SIZE;
        }
        pcm_set_avail_min(out->pcm, out->config.avail_min);
        out->low_power = low_power;
    }

    /* only use resampler if required */
    if (out->config.rate != DEFAULT_OUT_SAMPLING_RATE) {
        out->resampler->resample_from_input(out->resampler,
                                            (int16_t *)buffer,
                                            &in_frames,
                                            (int16_t *)out->buffer,
                                            &out_frames);
        buf = out->buffer;
    } else {
        out_frames = in_frames;
        buf = (void *)buffer;
    }
    /*if (out->echo_reference != NULL) {
        struct echo_reference_buffer b;
        b.raw = (void *)buffer;
        b.frame_count = in_frames;

        get_playback_delay(out, out_frames, &b);
        out->echo_reference->write(out->echo_reference, &b);
    }*/

    /* do not allow more than out->write_threshold frames in kernel pcm driver buffer */
    do {
        struct timespec time_stamp;

        if (pcm_get_htimestamp(out->pcm, (unsigned int *)&kernel_frames, &time_stamp) < 0)
            break;
        kernel_frames = pcm_get_buffer_size(out->pcm) - kernel_frames;

        if (kernel_frames > out->write_threshold) {
            unsigned long time = (unsigned long)
                    (((int64_t)(kernel_frames - out->write_threshold) * 1000000) /
                            MM_FULL_POWER_SAMPLING_RATE);
            if (time < MIN_WRITE_SLEEP_US)
                time = MIN_WRITE_SLEEP_US;
            usleep(time);
        }
    } while (kernel_frames > out->write_threshold);

    ret = pcm_mmap_write(out->pcm, (void *)buf, out_frames * frame_size);

exit:
    pthread_mutex_unlock(&out->lock);

    if (ret != 0) {
        usleep(bytes * 1000000 / audio_stream_frame_size(&stream->common) /
               out_get_sample_rate(&stream->common));
    }

    /*if (force_input_standby) {
        pthread_mutex_lock(&adev->lock);
        if (adev->active_input) {
            in = adev->active_input;
            pthread_mutex_lock(&in->lock);
            do_input_standby(in);
            pthread_mutex_unlock(&in->lock);
        }
        pthread_mutex_unlock(&adev->lock);
    }*/

    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: out_write() complete!");

    return bytes;
}

static int adev_open_output_stream(struct audio_hw_device *dev,
                                   uint32_t devices, int *format,
                                   uint32_t *channels, uint32_t *sample_rate,
                                   struct audio_stream_out **stream_out)
{
    struct tuna_audio_device *ladev = (struct omap3_audio_device *)dev;
    struct omap3_stream_out *out;
    int ret;

    out = (struct omap3_stream_out *)calloc(1, sizeof(struct omap3_stream_out));
    if (!out)
        return -ENOMEM;

    ret = create_resampler(DEFAULT_OUT_SAMPLING_RATE,
                           MM_FULL_POWER_SAMPLING_RATE,
                           2,
                           RESAMPLER_QUALITY_DEFAULT,
                           NULL,
                           &out->resampler);

    if (ret != 0)
        goto err_open;
    out->buffer = malloc(RESAMPLER_BUFFER_SIZE);

    out->stream.common.get_sample_rate = out_get_sample_rate;
    out->stream.common.set_sample_rate = out_set_sample_rate;
    out->stream.common.get_buffer_size = out_get_buffer_size;
    out->stream.common.get_channels = out_get_channels;
    out->stream.common.get_format = out_get_format;
    out->stream.common.set_format = out_set_format;
    out->stream.common.standby = out_standby;
    out->stream.common.dump = out_dump;
    out->stream.common.set_parameters = out_set_parameters;
    out->stream.common.get_parameters = out_get_parameters;
    //out->stream.common.add_audio_effect = out_add_audio_effect;
    //out->stream.common.remove_audio_effect = out_remove_audio_effect;
    out->stream.get_latency = out_get_latency;
    out->stream.set_volume = out_set_volume;
    out->stream.write = out_write;
    //out->stream.get_render_position = out_get_render_position;

    out->config = pcm_config_mm;

    out->dev = ladev;
    out->standby = 1;

    /* FIXME: when we support multiple output devices, we will want to
     * do the following:
     * adev->devices &= ~AUDIO_DEVICE_OUT_ALL;
     * adev->devices |= out->device;
     * select_output_device(adev);
     * This is because out_set_parameters() with a route is not
     * guaranteed to be called after an output stream is opened. */

    *format = out_get_format(&out->stream.common);
    *channels = out_get_channels(&out->stream.common);
    *sample_rate = out_get_sample_rate(&out->stream.common);

    *stream_out = &out->stream;

    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: adev_open_output_stream() complete!");

    return 0;

err_open:
    free(out);
    *stream_out = NULL;
    return ret;
}

static void adev_close_output_stream(struct audio_hw_device *dev,
                                     struct audio_stream_out *stream)
{
    struct omap3_stream_out *out = (struct omap3_stream_out *)stream;

    out_standby(&stream->common);
    if (out->buffer)
        free(out->buffer);
    if (out->resampler)
        release_resampler(out->resampler);
    free(stream);

    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: adev_close_output_stream() complete!");

}

static int adev_set_parameters(struct audio_hw_device *dev, const char *kvpairs)
{
    struct omap3_audio_device *adev = (struct omap3_audio_device *)dev;
    struct str_parms *parms;
    char *str;
    char value[32];
    int ret;

    parms = str_parms_create_str(kvpairs);
    /*ret = str_parms_get_str(parms, AUDIO_PARAMETER_KEY_TTY_MODE, value, sizeof(value));
    if (ret >= 0) {
        int tty_mode;

        if (strcmp(value, AUDIO_PARAMETER_VALUE_TTY_OFF) == 0)
            tty_mode = TTY_MODE_OFF;
        else if (strcmp(value, AUDIO_PARAMETER_VALUE_TTY_VCO) == 0)
            tty_mode = TTY_MODE_VCO;
        else if (strcmp(value, AUDIO_PARAMETER_VALUE_TTY_HCO) == 0)
            tty_mode = TTY_MODE_HCO;
        else if (strcmp(value, AUDIO_PARAMETER_VALUE_TTY_FULL) == 0)
            tty_mode = TTY_MODE_FULL;
        else
            return -EINVAL;

        pthread_mutex_lock(&adev->lock);
        if (tty_mode != adev->tty_mode) {
            adev->tty_mode = tty_mode;
            if (adev->mode == AUDIO_MODE_IN_CALL)
                select_output_device(adev);
        }
        pthread_mutex_unlock(&adev->lock);
    }*/

    ret = str_parms_get_str(parms, AUDIO_PARAMETER_KEY_BT_NREC, value, sizeof(value));
    if (ret >= 0) {
        if (strcmp(value, AUDIO_PARAMETER_VALUE_ON) == 0)
            adev->bluetooth_nrec = true;
        else
            adev->bluetooth_nrec = false;
    }

    ret = str_parms_get_str(parms, "screen_state", value, sizeof(value));
    if (ret >= 0) {
        if (strcmp(value, AUDIO_PARAMETER_VALUE_ON) == 0)
            adev->low_power = false;
        else
            adev->low_power = true;
    }

    str_parms_destroy(parms);

    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: adev_set_parameters() complete!");

    return ret;
}

static char * adev_get_parameters(const struct audio_hw_device *dev,
                                  const char *keys)
{

    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: adev_get_parameters() complete!");

    return strdup("");
}

static int adev_init_check(const struct audio_hw_device *dev)
{

    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: adev_init_check() complete!");

    return 0;
}

static int adev_dump(const audio_hw_device_t *device, int fd)
{

    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: adev_dump() complete!");

    return 0;
}

/*****************************************************************************
 * ALSA device open and close functions                                      *
 *****************************************************************************/

static int adev_close(hw_device_t *device)
{
    free(device);

    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: adev_close() complete!");

    return 0;
}

static uint32_t adev_get_supported_devices(const struct audio_hw_device *dev)
{

    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: adev_get_supported_devices() complete!");

    return (/* OUT */
            AUDIO_DEVICE_OUT_EARPIECE |
            AUDIO_DEVICE_OUT_SPEAKER |
            AUDIO_DEVICE_OUT_WIRED_HEADSET |
            AUDIO_DEVICE_OUT_WIRED_HEADPHONE |
            AUDIO_DEVICE_OUT_AUX_DIGITAL |
            AUDIO_DEVICE_OUT_ANLG_DOCK_HEADSET |
            AUDIO_DEVICE_OUT_DGTL_DOCK_HEADSET |
            AUDIO_DEVICE_OUT_ALL_SCO |
            AUDIO_DEVICE_OUT_DEFAULT |
            /* IN */
            AUDIO_DEVICE_IN_COMMUNICATION |
            AUDIO_DEVICE_IN_AMBIENT |
            AUDIO_DEVICE_IN_BUILTIN_MIC |
            AUDIO_DEVICE_IN_WIRED_HEADSET |
            AUDIO_DEVICE_IN_AUX_DIGITAL |
            AUDIO_DEVICE_IN_BACK_MIC |
            AUDIO_DEVICE_IN_ALL_SCO |
            AUDIO_DEVICE_IN_DEFAULT);
}

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

    adev->hw_device.get_supported_devices = adev_get_supported_devices;
    adev->hw_device.init_check = adev_init_check;
    //adev->hw_device.set_voice_volume = adev_set_voice_volume;
    adev->hw_device.set_master_volume = adev_set_master_volume;
    adev->hw_device.set_mode = adev_set_mode;
    adev->hw_device.set_mic_mute = adev_set_mic_mute;
    adev->hw_device.get_mic_mute = adev_get_mic_mute;
    adev->hw_device.set_parameters = adev_set_parameters;
    adev->hw_device.get_parameters = adev_get_parameters;
    //adev->hw_device.get_input_buffer_size = adev_get_input_buffer_size;
    adev->hw_device.open_output_stream = adev_open_output_stream;
    adev->hw_device.close_output_stream = adev_close_output_stream;
    //adev->hw_device.open_input_stream = adev_open_input_stream;
    //adev->hw_device.close_input_stream = adev_close_input_stream;
    adev->hw_device.dump = adev_dump;

    adev->mixer = mixer_open(0);
    if (!adev->mixer) {
        free(adev);
        LOGE("Unable to open the mixer, aborting.");
        return -EINVAL;
    }

    adev->mixer_ctls.headset_volume = mixer_get_ctl_by_name(adev->mixer,
                                           MIXER_HEADSET_PLAYBACK_VOLUME);
    adev->mixer_ctls.speaker_volume = mixer_get_ctl_by_name(adev->mixer,
                                           MIXER_HANDSFREE_PLAYBACK_VOLUME);

    if (!adev->mixer_ctls.headset_volume || !adev->mixer_ctls.speaker_volume)
    {
        mixer_close(adev->mixer);
        free(adev);
        LOGE("Unable to locate all mixer controls, aborting.");
        return -EINVAL;
    }

    /* Set the default route before the PCM stream is opened */
    pthread_mutex_lock(&adev->lock);
    set_route_by_array(adev->mixer, defaults, 1); // may need to change something here...
    adev->mode = AUDIO_MODE_NORMAL;
    adev->devices = AUDIO_DEVICE_OUT_SPEAKER | AUDIO_DEVICE_IN_BUILTIN_MIC;
    select_output_device(adev);

    adev->bluetooth_nrec = true;

    *device = &adev->hw_device.common;

    /* XXX | DEBUG | XXX */
    LOGD("DEBUG: adev_open() complete!");

    return 0;
}

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

// ----------------------------------------------------------------------------
// - [END] REWRITE FOR TINYALSA BY GERAD MUNSCH
// ----------------------------------------------------------------------------
