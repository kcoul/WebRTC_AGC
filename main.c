#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
//SOURCE: https://github.com/mackron/dr_libs/blob/master/dr_wav.h
#define DR_WAV_IMPLEMENTATION

#include "dr_wav.h"
#include "agc.h"

#ifndef nullptr
#define nullptr 0
#endif

#ifndef MIN
#define  MIN(A, B)        ((A) < (B) ? (A) : (B))
#endif

#include <stdint.h>

#if   defined(__APPLE__)
# include <mach/mach_time.h>
#elif defined(_WIN32)
# define WIN32_LEAN_AND_MEAN

# include <windows.h>

#else // __linux
# include <time.h>
# ifndef  CLOCK_MONOTONIC //_RAW
#  define CLOCK_MONOTONIC CLOCK_REALTIME
# endif
#endif

static
uint64_t nanotimer() {
    static int ever = 0;
#if defined(__APPLE__)
    static mach_timebase_info_data_t frequency;
    if (!ever) {
        if (mach_timebase_info(&frequency) != KERN_SUCCESS) {
            return 0;
        }
        ever = 1;
    }
    return 0;
#elif defined(_WIN32)
    static LARGE_INTEGER frequency;
    if (!ever) {
        QueryPerformanceFrequency(&frequency);
        ever = 1;
    }
    LARGE_INTEGER t;
    QueryPerformanceCounter(&t);
    return (t.QuadPart * (uint64_t) 1e9) / frequency.QuadPart;
#else // __linux
    struct timespec t;
    if (!ever) {
        if (clock_gettime(CLOCK_MONOTONIC, &t) != 0) {
            return 0;
        }
        ever = 1;
    }
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (t.tv_sec * (uint64_t)1e9) + t.tv_nsec;
#endif
}


static double now() {
    static uint64_t epoch = 0;
    if (!epoch) {
        epoch = nanotimer();
    }
    return (nanotimer() - epoch) / 1e9;
};

double calcElapsed(double start, double end) {
    double took = -start;
    return took + end;
}

//wav
void wavWrite_int16(char *filename, int16_t *buffer, size_t sampleRate, size_t totalSampleCount, unsigned int channels) {
    drwav_data_format format = {};
    format.container = drwav_container_riff;     // <-- drwav_container_riff = normal WAV files, drwav_container_w64 = Sony Wave64.
    format.format = DR_WAVE_FORMAT_PCM;          // <-- Any of the DR_WAVE_FORMAT_* codes.
    format.channels = channels;
    format.sampleRate = (drwav_uint32) sampleRate;
    format.bitsPerSample = 16;
    drwav *pWav = drwav_open_file_write(filename, &format);
    if (pWav) {
        drwav_uint64 samplesWritten = drwav_write(pWav, totalSampleCount, buffer);
        drwav_uninit(pWav);
        if (samplesWritten != totalSampleCount) {
            fprintf(stderr, "ERROR\n");
            exit(1);
        }
    }
}

//wav
int16_t *wavRead_int16(char *filename, uint32_t *sampleRate, uint64_t *totalSampleCount, unsigned int* channels) {
    int16_t *buffer = drwav_open_and_read_file_s16(filename, channels, sampleRate, totalSampleCount);
    if (buffer == nullptr) {
        printf("Couldn't load wav file.");
    }
    return buffer;
}

//
void splitpath(const char *path, char *drv, char *dir, char *name, char *ext) {
    const char *end;
    const char *p;
    const char *s;
    if (path[0] && path[1] == ':') {
        if (drv) {
            *drv++ = *path++;
            *drv++ = *path++;
            *drv = '\0';
        }
    } else if (drv)
        *drv = '\0';
    for (end = path; *end && *end != ':';)
        end++;
    for (p = end; p > path && *--p != '\\' && *p != '/';)
        if (*p == '.') {
            end = p;
            break;
        }
    if (ext)
        for (s = end; (*ext = *s++);)
            ext++;
    for (p = end; p > path;)
        if (*--p == '\\' || *p == '/') {
            p++;
            break;
        }
    if (name) {
        for (s = p; s < end;)
            *name++ = *s++;
        *name = '\0';
    }
    if (dir) {
        for (s = path; s < p;)
            *dir++ = *s++;
        *dir = '\0';
    }
}


int agcProcess(int16_t *buffer, uint32_t sampleRate, size_t samplesCount, int16_t agcMode) {
    if (buffer == nullptr) return -1;
    if (samplesCount == 0) return -1;
    WebRtcAgcConfig agcConfig;
    agcConfig.compressionGaindB = 9; // default 9 dB
    agcConfig.limiterEnable = 1; // default kAgcTrue (on)
    agcConfig.targetLevelDbfs = 3; // default 3 (-3 dBOv)
    int minLevel = 0;
    int maxLevel = 255;
    size_t samples = MIN(160, sampleRate / 100);
    if (samples == 0) return -1;
    const int maxSamples = 320;
    int16_t *input = buffer;
    size_t nTotal = (samplesCount / samples);
    void *agcInst = WebRtcAgc_Create();
    if (agcInst == NULL) return -1;
    int status = WebRtcAgc_Init(agcInst, minLevel, maxLevel, agcMode, sampleRate);
    if (status != 0) {
        printf("WebRtcAgc_Init fail\n");
        WebRtcAgc_Free(agcInst);
        return -1;
    }
    status = WebRtcAgc_set_config(agcInst, agcConfig);
    if (status != 0) {
        printf("WebRtcAgc_set_config fail\n");
        WebRtcAgc_Free(agcInst);
        return -1;
    }
    size_t num_bands = 1;
    int inMicLevel, outMicLevel = -1;
    int16_t out_buffer[maxSamples];
    int16_t *out16 = out_buffer;
    uint8_t saturationWarning = 1;
    int16_t echo = 0;
    for (int i = 0; i < nTotal; i++) {
        inMicLevel = 0;
        int nAgcRet = WebRtcAgc_Process(agcInst, (const int16_t *const *) &input, num_bands, samples,
                                        (int16_t *const *) &out16, inMicLevel, &outMicLevel, echo,
                                        &saturationWarning);

        if (nAgcRet != 0) {
            printf("failed in WebRtcAgc_Process\n");
            WebRtcAgc_Free(agcInst);
            return -1;
        }
        memcpy(input, out_buffer, samples * sizeof(int16_t));
        input += samples;
    }
    //Final chunk of audio that is less than 1 frame in size, if any
    const size_t remainedSamples = samplesCount - nTotal * samples;
    if (remainedSamples > 0) {
        if (nTotal > 0) {
            input = input - samples + remainedSamples;
        }

        inMicLevel = 0;
        int nAgcRet = WebRtcAgc_Process(agcInst, (const int16_t *const *) &input, num_bands, samples,
                                        (int16_t *const *) &out16, inMicLevel, &outMicLevel, echo,
                                        &saturationWarning);

        if (nAgcRet != 0) {
            printf("failed in WebRtcAgc_Process during filtering the last chunk\n");
            WebRtcAgc_Free(agcInst);
            return -1;
        }
        memcpy(&input[samples-remainedSamples], &out_buffer[samples-remainedSamples], remainedSamples * sizeof(int16_t));
        input += samples;
    }

    WebRtcAgc_Free(agcInst);
    return 1;
}

void auto_gain(char *in_file, char *out_file) {
    uint32_t sampleRate = 0;
    uint64_t inSampleCount = 0;
    unsigned int channels = 0;
    int16_t *inBuffer = wavRead_int16(in_file, &sampleRate, &inSampleCount, &channels);
    if (inBuffer != nullptr) {
        // Three available modes:
        //  kAgcModeAdaptiveAnalog
        //  kAgcModeAdaptiveDigital
        //  kAgcModeFixedDigital
        double startTime = now();

        agcProcess(inBuffer, sampleRate, inSampleCount, kAgcModeAdaptiveDigital);

        double elapsed_time = calcElapsed(startTime, now());

        printf("time: %d ms\n ", (int) (elapsed_time * 1000));
        wavWrite_int16(out_file, inBuffer, sampleRate, inSampleCount, channels);
        free(inBuffer);
    }
 }

int main(int argc, char *argv[]) {
    printf("WebRTC Automatic Gain Control\n");
    if (argc < 2)
        return -1;
    char *in_file = argv[1];
    char drive[3];
    char dir[256];
    char fname[256];
    char ext[256];
    char out_file[1024];
    splitpath(in_file, drive, dir, fname, ext);
    sprintf(out_file, "%s%s%s_out%s", drive, dir, fname, ext);
    auto_gain(in_file, out_file);

    printf("Finished rendering output file. \n");
    //getchar();
    return 0;
}
