#include "mfcc.h"

const float window_func[FRAME_LEN] = WINDOW_FUNC;
const int32_t fbank_filter_first[NUM_FBANK_BINS] = FBANK_FILTER_FIRST;
const int32_t fbank_filter_last[NUM_FBANK_BINS] = FBANK_FILTER_LAST;
const float mel_fbank[NUM_FBANK_BINS][26] = MEL_FBANK;
const float dct_matrix[NUM_FBANK_BINS * NUM_MFCC_COEFFS] = DCT_MATRIX;
const float twiddleCoef_rfft_1024[FRAME_LEN_PADDED] = TWIDDLECOEF_RFFT_1024;

float frame[FRAME_LEN_PADDED];
float buffer[FRAME_LEN_PADDED];
float mel_energies[NUM_FBANK_BINS];

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

void mfcc_compute(const int16_t * audio_data, int16_t * mfcc_out) {
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
	stage_rfft_f32(frame, buffer);

	int32_t half_dim = FRAME_LEN_PADDED / 2;
	float first_energy = buffer[0] * buffer[0];
	float last_energy =  buffer[1] * buffer[1];
	for (i = 1; i < half_dim; i++) {
		float real = buffer[i * 2];
		float im = buffer[i * 2 + 1];
		buffer[i] = real * real + im * im;
	}
	buffer[0] = first_energy;
	buffer[half_dim] = last_energy;  

	float sqrt_data;
	for (bin = 0; bin < NUM_FBANK_BINS; bin++) {
		j = 0;
		float mel_energy = 0;
		int32_t first_index = fbank_filter_first[bin];
		int32_t last_index = fbank_filter_last[bin];
		for (i = first_index; i <= last_index; i++) {
			sqrt_data = sqrtf(buffer[i]);
			mel_energy += (sqrt_data) * mel_fbank[bin][j++];
		}
		mel_energies[bin] = mel_energy;

		if (mel_energy == 0.0)
			mel_energies[bin] = FLT_MIN;
	}

	for (bin = 0; bin < NUM_FBANK_BINS; bin++)
		mel_energies[bin] = logf(mel_energies[bin]);

	for (i = 0; i < NUM_MFCC_COEFFS; i++) {
		float sum = 0.0;
		for (j = 0; j < NUM_FBANK_BINS; j++) {
			sum += dct_matrix[i * NUM_FBANK_BINS + j] * mel_energies[j];
		}
		
		sum *= (0x1 << 9);
		sum = round(sum); 
		if(sum >= 32767)
			mfcc_out[i] = 32767;
		else if(sum <= -32768)
			mfcc_out[i] = -32768;
		else
			mfcc_out[i] = sum; 
	}
}
