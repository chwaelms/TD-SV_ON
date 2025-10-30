#ifndef MELSPEC_H
#define MELSPEC_H

#include <math.h>
#include <float.h>
#include <string.h>
#include <stdint.h>
#include "esp_log.h"
#include "esp_dsp.h"

#include "mfcc_filters.h"

//FFT가 기존 mfcc.c/h는 1024를 기준으로 생성된 것인데, 여기서는 512 사용하기 때문에 수정 필요
#define SAMPLE_RATE 16000
#define FRAME_LEN 512
#define FRAME_STEP 160
#define FFT_LEN 512
#define FRAME_LEN_PADDED FFT_LEN
#define NUM_FBANK_BINS 80
#define NUM_SPECTROGRAM_BINS (FFT_LEN / 2 + 1)  // 257

#define AUDIO_BUFFER_LEN (SAMPLE_RATE * 3)  // 48000
#define NUM_FRAMES ((AUDIO_BUFFER_LEN - FRAME_LEN) / FRAME_STEP + 1)  // 299
#define NUM_MEL_BINS NUM_FBANK_BINS
#define MELSPEC_OUTPUT_SIZE (NUM_FRAMES * NUM_MEL_BINS)
#ifdef __cplusplus
extern "C" {
#endif
int melspec_init(void);
void melspec_deinit(void);
void melspec_compute(const int16_t *audio_data, float *melspec_out);
void normalize(float *melSpectrogram, int num_frames);
#ifdef __cplusplus
}
#endif

#endif