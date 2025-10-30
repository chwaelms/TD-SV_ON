#include "melspec.h"
#include "melspec_filters.h"
#include "esp_log.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"


#define EPSILON 1e-12
const float window_func[FRAME_LEN] = WINDOW_FUNC;
const int32_t fbank_filter_first[NUM_FBANK_BINS] = FBANK_FILTER_FIRST;
const int32_t fbank_filter_last[NUM_FBANK_BINS] = FBANK_FILTER_LAST;
const float mel_fbank[NUM_FBANK_BINS][14] = MEL_FBANK;
const float twiddleCoef_rfft_512[FFT_LEN] = TWIDDLECOEF_RFFT_512;

float frame[FRAME_LEN_PADDED] __attribute__((section(".ext_ram.bss")));
float s_buffer[FRAME_LEN_PADDED] __attribute__((section(".ext_ram.bss")));
float mel_energies[NUM_FBANK_BINS] __attribute__((section(".ext_ram.bss")));

static inline float InverseMelScale(float mel_freq) {
    return 700.0f * (expf(mel_freq / 1127.0f) - 1.0f);
}

static inline float MelScale(float freq) {
    return 1127.0f * logf(1.0f + freq / 700.0f);
}

static void stage_rfft_f32(float *p, float *pOut)
{
    uint32_t k;
    float twR, twI;
    float *pCoeff = (float *)twiddleCoef_rfft_512; // (수정)
    float *pA = p;
    float *pB = p;
    float xAR, xAI, xBR, xBI;
    float t1a, t1b;
    float p0, p1, p2, p3;

    k = (FFT_LEN / 2) - 1; // (수정)

    xBR = pB[0];
    xBI = pB[1];
    xAR = pA[0];
    xAI = pA[1];

    twR = *pCoeff++;
    twI = *pCoeff++;

    t1a = xBR + xAR;
    t1b = xBI + xAI;

    *pOut++ = 0.5f * (t1a + t1b);
    *pOut++ = 0.5f * (t1a - t1b);

    pB = p + 2 * k;
    pA += 2;

    do {
        xBI = pB[1];
        xBR = pB[0];
        xAR = pA[0];
        xAI = pA[1];

        twR = *pCoeff++;
        twI = *pCoeff++;

        t1a = xBR - xAR;
        t1b = xBI + xAI;

        p0 = twR * t1a;
        p1 = twI * t1a;
        p2 = twR * t1b;
        p3 = twI * t1b;

        *pOut++ = 0.5f * (xAR + xBR + p0 + p3);
        *pOut++ = 0.5f * (xAI - xBI + p1 - p2);

        pA += 2;
        pB -= 2;
        k--;
    } while (k > 0u);
}

int melspec_init(void) {  // 수정
    esp_err_t ret;
    
    ret = dsps_fft2r_init_fc32(NULL, FFT_LEN);  // 수정
    if (ret != ESP_OK) {
        ESP_LOGE("melspec", "FFT initialization failed. Error = %d", ret);
        return -1;
    }
    
    ESP_LOGI("melspec", "melspec initialized: FRAME_LEN=%d, FFT_LEN=%d, NUM_FBANK_BINS=%d", 
             FRAME_LEN, FFT_LEN, NUM_FBANK_BINS);
    return 0;
}

void melspec_deinit(void) {
    dsps_fft2r_deinit_fc32();
}

// Global normalization
void normalize(float *melSpectrogram, int num_frames) {
    float sum = 0.0f;
    float sum_squares = 0.0f;
    int total_elements = num_frames * NUM_FBANK_BINS;
    
    // 전체 평균과 분산 계산
    for (int i = 0; i < total_elements; i++) {
        sum += melSpectrogram[i];
        sum_squares += melSpectrogram[i] * melSpectrogram[i];
    }
    
    float mean = sum / total_elements;
    float variance = (sum_squares / total_elements) - (mean * mean);
    float stddev = sqrtf(variance + EPSILON);
    
    // 정규화
    for (int i = 0; i < total_elements; i++) {
        melSpectrogram[i] = (melSpectrogram[i] - mean) / stddev;
    }
}

void melspec_compute(const int16_t *audio_data, float *freq_data) {
    int32_t i, j, bin;

    // 1. Int16 → Float 변환
    for (i = 0; i < FRAME_LEN; i++) {
        frame[i] = (float)audio_data[i] / 32768.0f;  // 수정 (1 << 15) 대신 명시적 값
    }
    
    // 2. Zero padding
    memset(&frame[FRAME_LEN], 0, sizeof(float) * (FRAME_LEN_PADDED - FRAME_LEN));

    // 3. Hann window 적용
    for (i = 0; i < FRAME_LEN; i++) {
        frame[i] *= window_func[i];
    }
    
    // 4. FFT
    dsps_fft2r_fc32(frame, FRAME_LEN_PADDED / 2);
    dsps_bit_rev_fc32(frame, FRAME_LEN_PADDED / 2);
    stage_rfft_f32(frame, s_buffer);

    // 5. Power spectrum
    int32_t half_dim = FRAME_LEN_PADDED / 2;
    float first_energy = s_buffer[0] * s_buffer[0];
    float last_energy = s_buffer[1] * s_buffer[1];
    
    for (i = 1; i < half_dim+1; i++) {
        float real = s_buffer[i * 2];
        float im = s_buffer[i * 2 + 1];
        s_buffer[i] = real * real + im * im;
    }
    s_buffer[0] = first_energy;
    s_buffer[half_dim] = last_energy;
    
    // 6. Magnitude spectrum
    for (i = 0; i <= half_dim + 1; i++) {  
        s_buffer[i] = sqrtf(s_buffer[i]);
    }

    // 7. Mel filterbank 적용
    for (bin = 0; bin < NUM_FBANK_BINS; bin++) {
        float mel_energy = 0.0f;
        int32_t first_index = fbank_filter_first[bin];
        int32_t last_index = fbank_filter_last[bin];
        
        j = 0;
        for (i = first_index; i <= last_index; i++) {
            mel_energy += s_buffer[i] * mel_fbank[bin][j++];
        }
        mel_energies[bin] = mel_energy;
    }

    // 8. Log 적용
    for (bin = 0; bin < NUM_FBANK_BINS; bin++) {
        freq_data[bin] = logf(mel_energies[bin] + EPSILON);  // ✅ FLT_MIN 대신 EPSILON
    }
}