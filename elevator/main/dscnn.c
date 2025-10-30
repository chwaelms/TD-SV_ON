#include "dscnn.h"
#include "esp_heap_caps.h"

const int16_t conv1_wt[] = CONV_1_1;
const int16_t conv1_bias[] = CONV_1_2;
const int16_t conv2_ds_wt[] = CONV_2_DS_1;
const int16_t conv2_ds_bias[] = CONV_2_DS_2;
const int16_t conv2_pw_wt[] = CONV_2_PW_1;
const int16_t conv2_pw_bias[] = CONV_2_PW_2;
const int16_t conv3_ds_wt[] = CONV_3_DS_1;
const int16_t conv3_ds_bias[] = CONV_3_DS_2;
const int16_t conv3_pw_wt[] = CONV_3_PW_1;
const int16_t conv3_pw_bias[] = CONV_3_PW_2;
const int16_t conv4_ds_wt[] = CONV_4_DS_1;
const int16_t conv4_ds_bias[] = CONV_4_DS_2;
const int16_t conv4_pw_wt[] = CONV_4_PW_1;
const int16_t conv4_pw_bias[] = CONV_4_PW_2;
const int16_t conv5_ds_wt[] = CONV_5_DS_1;
const int16_t conv5_ds_bias[] = CONV_5_DS_2;
const int16_t conv5_pw_wt[] = CONV_5_PW_1;
const int16_t conv5_pw_bias[] = CONV_5_PW_2;
const int16_t final_fc_wt[] = FC_1;
const int16_t final_fc_bias[] = FC_2;

static dl_matrix3dq_t *input_matrix3dq = NULL;
static dl_matrix3dq_t *output_matrix3dq = NULL;
static dl_matrix3dq_t *conv1_wt_matrix3dq = NULL;
static dl_matrix3dq_t *conv1_bias_matrix3dq = NULL;
static dl_matrix3dq_t *conv2_ds_wt_matrix3dq = NULL;
static dl_matrix3dq_t *conv2_ds_bias_matrix3dq = NULL;
static dl_matrix3dq_t *conv2_pw_wt_matrix3dq = NULL;
static dl_matrix3dq_t *conv2_pw_bias_matrix3dq = NULL;
static dl_matrix3dq_t *conv3_ds_wt_matrix3dq = NULL;
static dl_matrix3dq_t *conv3_ds_bias_matrix3dq = NULL;
static dl_matrix3dq_t *conv3_pw_wt_matrix3dq = NULL;
static dl_matrix3dq_t *conv3_pw_bias_matrix3dq = NULL;
static dl_matrix3dq_t *conv4_ds_wt_matrix3dq = NULL;
static dl_matrix3dq_t *conv4_ds_bias_matrix3dq = NULL;
static dl_matrix3dq_t *conv4_pw_wt_matrix3dq = NULL;
static dl_matrix3dq_t *conv4_pw_bias_matrix3dq = NULL;
static dl_matrix3dq_t *conv5_ds_wt_matrix3dq = NULL;
static dl_matrix3dq_t *conv5_ds_bias_matrix3dq = NULL;
static dl_matrix3dq_t *conv5_pw_wt_matrix3dq = NULL;
static dl_matrix3dq_t *conv5_pw_bias_matrix3dq = NULL;
static dl_matrix3dq_t *final_fc_wt_matrix3dq = NULL;
static dl_matrix3dq_t *final_fc_bias_matrix3dq = NULL;

void dscnn_init()
{
	dl_matrix3dq_t *buffer_matrix3dq = NULL;
	
    // conv1
	conv1_wt_matrix3dq = dl_matrix3dq_alloc(64, 3, 3, 1, -16);
	memcpy(conv1_wt_matrix3dq->item, &conv1_wt, sizeof(conv1_wt));
		
	buffer_matrix3dq = dl_matrix3dq_alloc(1, 1, 1, 64, -15);
	conv1_bias_matrix3dq = dl_matrix3dq_alloc(1, 1, 1, 64, -9);
	memcpy(buffer_matrix3dq->item, &conv1_bias, sizeof(conv1_bias));
	dl_matrix3dq_shift_exponent(conv1_bias_matrix3dq, buffer_matrix3dq, -9);
	dl_matrix3dq_free(buffer_matrix3dq);

	// conv2_ds
	conv2_ds_wt_matrix3dq = dl_matrix3dq_alloc(1, 3, 3, 64, -12);
	memcpy(conv2_ds_wt_matrix3dq->item, &conv2_ds_wt, sizeof(conv2_ds_wt));
	
	buffer_matrix3dq = dl_matrix3dq_alloc(1, 1, 1, 64, -14);
	conv2_ds_bias_matrix3dq = dl_matrix3dq_alloc(1, 1, 1, 64, -9);
	memcpy(buffer_matrix3dq->item, &conv2_ds_bias, sizeof(conv2_ds_bias));
	dl_matrix3dq_shift_exponent(conv2_ds_bias_matrix3dq, buffer_matrix3dq, -9);
	dl_matrix3dq_free(buffer_matrix3dq);
	
	// conv2_pw
	conv2_pw_wt_matrix3dq = dl_matrix3dq_alloc(64, 1, 1, 64, -14);
	memcpy(conv2_pw_wt_matrix3dq->item, &conv2_pw_wt, sizeof(conv2_pw_wt));
	
	buffer_matrix3dq = dl_matrix3dq_alloc(1, 1, 1, 64, -13);
	conv2_pw_bias_matrix3dq = dl_matrix3dq_alloc(1, 1, 1, 64, -9);
	memcpy(buffer_matrix3dq->item, &conv2_pw_bias, sizeof(conv2_pw_bias));
	dl_matrix3dq_shift_exponent(conv2_pw_bias_matrix3dq, buffer_matrix3dq, -9);
	dl_matrix3dq_free(buffer_matrix3dq);
	
	// conv3_ds
	conv3_ds_wt_matrix3dq = dl_matrix3dq_alloc(1, 3, 3, 64, -13);
	memcpy(conv3_ds_wt_matrix3dq->item, &conv3_ds_wt, sizeof(conv3_ds_wt));
	
	buffer_matrix3dq = dl_matrix3dq_alloc(1, 1, 1, 64, -14);
	conv3_ds_bias_matrix3dq = dl_matrix3dq_alloc(1, 1, 1, 64, -9);
	memcpy(buffer_matrix3dq->item, &conv3_ds_bias, sizeof(conv3_ds_bias));
	dl_matrix3dq_shift_exponent(conv3_ds_bias_matrix3dq, buffer_matrix3dq, -9);
	dl_matrix3dq_free(buffer_matrix3dq);
	
	// conv3_pw
	conv3_pw_wt_matrix3dq = dl_matrix3dq_alloc(64, 1, 1, 64, -14);
	memcpy(conv3_pw_wt_matrix3dq->item, &conv3_pw_wt, sizeof(conv3_pw_wt));
	
	buffer_matrix3dq = dl_matrix3dq_alloc(1, 1, 1, 64, -13);
	conv3_pw_bias_matrix3dq = dl_matrix3dq_alloc(1, 1, 1, 64, -9);
	memcpy(buffer_matrix3dq->item, &conv3_pw_bias, sizeof(conv3_pw_bias));
	dl_matrix3dq_shift_exponent(conv3_pw_bias_matrix3dq, buffer_matrix3dq, -9);
	dl_matrix3dq_free(buffer_matrix3dq);
	
	// conv4_ds
	conv4_ds_wt_matrix3dq = dl_matrix3dq_alloc(1, 3, 3, 64, -13);
	memcpy(conv4_ds_wt_matrix3dq->item, &conv4_ds_wt, sizeof(conv4_ds_wt));
	
	buffer_matrix3dq = dl_matrix3dq_alloc(1, 1, 1, 64, -13);
	conv4_ds_bias_matrix3dq = dl_matrix3dq_alloc(1, 1, 1, 64, -9);
	memcpy(buffer_matrix3dq->item, &conv4_ds_bias, sizeof(conv4_ds_bias));
	dl_matrix3dq_shift_exponent(conv4_ds_bias_matrix3dq, buffer_matrix3dq, -9);
	dl_matrix3dq_free(buffer_matrix3dq);
	
	// conv4_pw
	conv4_pw_wt_matrix3dq = dl_matrix3dq_alloc(64, 1, 1, 64, -15);
	memcpy(conv4_pw_wt_matrix3dq->item, &conv4_pw_wt, sizeof(conv4_pw_wt));
	
	buffer_matrix3dq = dl_matrix3dq_alloc(1, 1, 1, 64, -13);
	conv4_pw_bias_matrix3dq = dl_matrix3dq_alloc(1, 1, 1, 64, -9);
	memcpy(buffer_matrix3dq->item, &conv4_pw_bias, sizeof(conv4_pw_bias));
	dl_matrix3dq_shift_exponent(conv4_pw_bias_matrix3dq, buffer_matrix3dq, -9);
	dl_matrix3dq_free(buffer_matrix3dq);
	
	// conv5_ds
	conv5_ds_wt_matrix3dq = dl_matrix3dq_alloc(1, 3, 3, 64, -12);
	memcpy(conv5_ds_wt_matrix3dq->item, &conv5_ds_wt, sizeof(conv5_ds_wt));
	
	buffer_matrix3dq = dl_matrix3dq_alloc(1, 1, 1, 64, -14);
	conv5_ds_bias_matrix3dq = dl_matrix3dq_alloc(1, 1, 1, 64, -9);
	memcpy(buffer_matrix3dq->item, &conv5_ds_bias, sizeof(conv5_ds_bias));
	dl_matrix3dq_shift_exponent(conv5_ds_bias_matrix3dq, buffer_matrix3dq, -9);
	dl_matrix3dq_free(buffer_matrix3dq);
	
	// conv5_pw
	conv5_pw_wt_matrix3dq = dl_matrix3dq_alloc(64, 1, 1, 64, -14);
	memcpy(conv5_pw_wt_matrix3dq->item, &conv5_pw_wt, sizeof(conv5_pw_wt));
	
	buffer_matrix3dq = dl_matrix3dq_alloc(1, 1, 1, 64, -13);
	conv5_pw_bias_matrix3dq = dl_matrix3dq_alloc(1, 1, 1, 64, -9);
	memcpy(buffer_matrix3dq->item, &conv5_pw_bias, sizeof(conv5_pw_bias));
	dl_matrix3dq_shift_exponent(conv5_pw_bias_matrix3dq, buffer_matrix3dq, -9);
	dl_matrix3dq_free(buffer_matrix3dq);
	
	// fc
	final_fc_wt_matrix3dq = dl_matrix3dq_alloc(1, 64, 16, 1, -14);
	memcpy(final_fc_wt_matrix3dq->item, &final_fc_wt, sizeof(final_fc_wt));
	
	buffer_matrix3dq = dl_matrix3dq_alloc(1, 1, 1, 16, -15);
	final_fc_bias_matrix3dq = dl_matrix3dq_alloc(1, 1, 1, 16, -9);
	memcpy(buffer_matrix3dq->item, &final_fc_bias, sizeof(final_fc_bias));
	dl_matrix3dq_shift_exponent(final_fc_bias_matrix3dq, buffer_matrix3dq, -9);
	dl_matrix3dq_free(buffer_matrix3dq);
}


int dscnn_run(int16_t * in_data)
{
	int p = 0;
	int i, out = 0;
	float max = 0;
	
	// conv1
	input_matrix3dq = dl_matrix3dq_alloc(1, 11, 51, 1, -9);
	memset(input_matrix3dq->item, 0, 2 * 1 * 11 * 51 * 1);
	for (i = 0; i < 49; i++) {
		memcpy(input_matrix3dq->item + ((i + 1) * 11 * 1), in_data + (i * 10 * 1), 2 * 10 * 1);
	}

	output_matrix3dq = dl_matrix3dqq_conv_3x3_with_bias(input_matrix3dq, conv1_wt_matrix3dq, conv1_bias_matrix3dq, 2, 2, 0, -9, 1);
	dl_matrix3dq_free(input_matrix3dq);

	// conv2_ds
	input_matrix3dq = dl_matrix3dq_alloc(1, 7, 27, 64, -9);
	memset(input_matrix3dq->item, 0, 2 * 1 * 7 * 27 * 64);
	for (i = 0; i < 25; i++) {
		memcpy(input_matrix3dq->item + ((i + 1) * 7 * 64 + 1 * 64), output_matrix3dq->item + (i * 5 * 64), 2 * 5 * 64);
	}
	dl_matrix3dq_free(output_matrix3dq);
	output_matrix3dq = dl_matrix3dqq_depthwise_conv_3x3_with_bias(input_matrix3dq, conv2_ds_wt_matrix3dq, conv2_ds_bias_matrix3dq, 1, 1, 0, -9, 1);
	dl_matrix3dq_free(input_matrix3dq);
	
	// conv2_pw
	input_matrix3dq = output_matrix3dq;
	output_matrix3dq = dl_matrix3dq_alloc(1, 5, 25, 64, -9);
	
	dl_matrix3dqq_conv_1x1_with_bias_relu(output_matrix3dq, input_matrix3dq, conv2_pw_wt_matrix3dq, conv2_pw_bias_matrix3dq, 1);
	dl_matrix3dq_free(input_matrix3dq);
	
	// conv3_ds
	input_matrix3dq = dl_matrix3dq_alloc(1, 7, 27, 64, -9);
	memset(input_matrix3dq->item, 0, 2 * 1 * 7 * 27 * 64);
	for (i = 0; i < 25; i++) {
		memcpy(input_matrix3dq->item + ((i + 1) * 7 * 64 + 1 * 64), output_matrix3dq->item + (i * 5 * 64), 2 * 5 * 64);
	}
	dl_matrix3dq_free(output_matrix3dq);
	
	output_matrix3dq = dl_matrix3dqq_depthwise_conv_3x3_with_bias(input_matrix3dq, conv3_ds_wt_matrix3dq, conv3_ds_bias_matrix3dq, 1, 1, 0, -9, 1);
	dl_matrix3dq_free(input_matrix3dq);
	
	// conv3_pw
	input_matrix3dq = output_matrix3dq;
	output_matrix3dq = dl_matrix3dq_alloc(1, 5, 25, 64, -9);
		
	dl_matrix3dqq_conv_1x1_with_bias_relu(output_matrix3dq, input_matrix3dq, conv3_pw_wt_matrix3dq, conv3_pw_bias_matrix3dq, 1);
	dl_matrix3dq_free(input_matrix3dq);
	
	// conv4_ds
	input_matrix3dq = dl_matrix3dq_alloc(1, 7, 27, 64, -9);
	memset(input_matrix3dq->item, 0, 2 * 1 * 7 * 27 * 64);
	for (i = 0; i < 25; i++) {
		memcpy(input_matrix3dq->item + ((i + 1) * 7 * 64 + 1 * 64), output_matrix3dq->item + (i * 5 * 64), 2 * 5 * 64);
	}
	dl_matrix3dq_free(output_matrix3dq);
	
	output_matrix3dq = dl_matrix3dqq_depthwise_conv_3x3_with_bias(input_matrix3dq, conv4_ds_wt_matrix3dq, conv4_ds_bias_matrix3dq, 1, 1, 0, -9, 1);
	dl_matrix3dq_free(input_matrix3dq);
	
	// conv4_pw
	input_matrix3dq = output_matrix3dq;
	output_matrix3dq = dl_matrix3dq_alloc(1, 5, 25, 64, -9);
		
	dl_matrix3dqq_conv_1x1_with_bias_relu(output_matrix3dq, input_matrix3dq, conv4_pw_wt_matrix3dq, conv4_pw_bias_matrix3dq, 1);
	dl_matrix3dq_free(input_matrix3dq);
	
	// conv5_ds
	input_matrix3dq = dl_matrix3dq_alloc(1, 7, 27, 64, -9);
	memset(input_matrix3dq->item, 0, 2 * 1 * 7 * 27 * 64);
	for (i = 0; i < 25; i++) {
		memcpy(input_matrix3dq->item + ((i + 1) * 7 * 64 + 1 * 64), output_matrix3dq->item + (i * 5 * 64), 2 * 5 * 64);
	}
	dl_matrix3dq_free(output_matrix3dq);
	
	output_matrix3dq = dl_matrix3dqq_depthwise_conv_3x3_with_bias(input_matrix3dq, conv5_ds_wt_matrix3dq, conv5_ds_bias_matrix3dq, 1, 1, 0, -9, 1);
	dl_matrix3dq_free(input_matrix3dq);
	
	// conv5_pw
	input_matrix3dq = output_matrix3dq;
	output_matrix3dq = dl_matrix3dq_alloc(1, 5, 25, 64, -9);
		
	dl_matrix3dqq_conv_1x1_with_bias_relu(output_matrix3dq, input_matrix3dq, conv5_pw_wt_matrix3dq, conv5_pw_bias_matrix3dq, 1);
	dl_matrix3dq_free(input_matrix3dq);
	
	// global pool and fc
	input_matrix3dq = dl_matrix3dq_global_pool(output_matrix3dq);
	dl_matrix3dq_free(output_matrix3dq);
	output_matrix3dq = dl_matrix3dq_alloc(1, 1, 1, 16, -9);
	
	dl_matrix3dqq_fc_with_bias(output_matrix3dq, input_matrix3dq, final_fc_wt_matrix3dq, final_fc_bias_matrix3dq, 1, NULL);
	dl_matrix3dq_free(input_matrix3dq);
	
	// softmax
	dl_matrix3d_t *prob = dl_matrix3d_from_matrixq(output_matrix3dq);
	dl_matrix3dq_free(output_matrix3dq);
	dl_matrix3d_softmax(prob);
	
	for (i = 0; i < 16; i++) {
		//ESP_LOGI("", "%f", *(prob->item + i));
		if (max < *(prob->item + i)) {
			max = *(prob->item + i);
			out = i;
		}
	}
	dl_matrix3d_free(prob);
	if (max>0.8)
	 return out;
	else
	 return 0;
	 
}
