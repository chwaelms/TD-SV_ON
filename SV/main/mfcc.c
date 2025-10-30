#include "mfcc.h"
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
const float window_func[FRAME_LEN] = WINDOW_FUNC ;
const int32_t fbank_filter_first[NUM_FBANK_BINS] = FBANK_FILTER_FIRST;
const int32_t fbank_filter_last[NUM_FBANK_BINS] = FBANK_FILTER_LAST;
const float mel_fbank[NUM_FBANK_BINS][26] = MEL_FBANK;
const float twiddleCoef_rfft_1024[FRAME_LEN_PADDED] = TWIDDLECOEF_RFFT_1024;

float frame[FRAME_LEN_PADDED] __attribute__((section(".ext_ram.bss")));
float s_buffer[FRAME_LEN_PADDED] __attribute__((section(".ext_ram.bss")));
float mel_energies[NUM_FBANK_BINS] __attribute__((section(".ext_ram.bss")));

static inline float InverseMelScale(float mel_freq) {
	return 700.0f * (expf (mel_freq / 1127.0f) - 1.0f);
}

static inline float MelScale(float freq) {
	return 1127.0f * logf (1.0f + freq / 700.0f);
}

static void stage_rfft_f32(float * p, float * pOut)
{
   uint32_t  k;
   float twR, twI;
   float * pCoeff = twiddleCoef_rfft_1024;
   float * pA = p;
   float * pB = p;
   float xAR, xAI, xBR, xBI;
   float t1a, t1b;
   float p0, p1, p2, p3;

   k = 512 - 1;

   xBR = pB[0];
   xBI = pB[1];
   xAR = pA[0];
   xAI = pA[1];

   twR = *pCoeff++ ;
   twI = *pCoeff++ ;

   t1a = xBR + xAR  ;

   t1b = xBI + xAI  ;

   *pOut++ = 0.5f * ( t1a + t1b );
   *pOut++ = 0.5f * ( t1a - t1b );

   pB  = p + 2*k;
   pA += 2;

   do
   {
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

		*pOut++ = 0.5f * (xAR + xBR + p0 + p3 );
		*pOut++ = 0.5f * (xAI - xBI + p1 - p2 );

		pA += 2;
		pB -= 2;
		k--;
   } while(k > 0u);
}

void mfcc_init() {
	esp_err_t ret;
	
	ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (ret != ESP_OK)
    {
        ESP_LOGE("esp-eye", "Not possible to initialize FFT. Error = %i", ret);
        return;
    }
}

void normalize(float* melSpectrogram) {
    for (int i = 0; i < 49; ++i) {
        float sum = 0;
        float sumSquares = 0;
        for (int j = 0; j < 80; ++j) {
            sum += melSpectrogram[i*80+j];
            sumSquares += melSpectrogram[i*80+j] * melSpectrogram[i*80+j];
        }
        float mean = sum / 80;
        float variance = sumSquares / 80 - mean * mean;
        float stddev = sqrt(variance + EPSILON);

        for (int j = 0; j < 80; ++j) {
            melSpectrogram[i*80+j] = (melSpectrogram[i*80+j] - mean) / (stddev + EPSILON);
        }
    }
}

void mfcc_compute(const int16_t * audio_data, float * freq_data) {
	int32_t i, j, bin;

	for (i = 0; i < FRAME_LEN; i++) {
		frame[i] = (float)audio_data[i] / (1 << 15); 
	}
	
	memset(&frame[FRAME_LEN], 0, sizeof(float) * (FRAME_LEN_PADDED - FRAME_LEN));

	for (i = 0; i < FRAME_LEN; i++) {
		frame[i] *= window_func[i];
	}
	
	dsps_fft2r_fc32(frame, FRAME_LEN_PADDED / 2);
	dsps_bit_rev_fc32(frame, FRAME_LEN_PADDED / 2);
	stage_rfft_f32(frame, s_buffer);

	int32_t half_dim = FRAME_LEN_PADDED / 2;
	float first_energy = s_buffer[0] * s_buffer[0];
	float last_energy =  s_buffer[1] * s_buffer[1];
	for (i = 1; i < half_dim; i++) {
		float real = s_buffer[i * 2];
		float im = s_buffer[i * 2 + 1];
		s_buffer[i] = real * real + im * im;
	}
	s_buffer[0] = first_energy;
	s_buffer[half_dim] = last_energy;
	
  // printf("[");
	for (i = 0; i < half_dim + 1; i++) {
		s_buffer[i] = sqrtf(s_buffer[i]);
		// printf("%.6f,", s_buffer[i]);
	}
	
  // printf("],");

	// printf("[");
	float sqrt_data;
	for (bin = 0; bin < NUM_FBANK_BINS; bin++) {
		j = 0;
		float mel_energy = 0;
		int32_t first_index = fbank_filter_first[bin];
		int32_t last_index = fbank_filter_last[bin];
		for (i = first_index; i <= last_index; i++) {
			//sqrt_data = sqrtf(buffer[i]);
			mel_energy += (s_buffer[i]) * mel_fbank[bin][j++];
		}
		mel_energies[bin] = mel_energy;

		//if (mel_energy == 0.0) mel_energies[bin] = FLT_MIN;

        // printf("%.6f,", mel_energies[bin]);
	}
    // printf("],");

    // printf("[");
	for (bin = 0; bin < NUM_FBANK_BINS; bin++){
		freq_data[bin] = logf(mel_energies[bin]+FLT_MIN);
       //  printf("%.6f,", freq_data[bin]);
    }
    // printf("],");
}


