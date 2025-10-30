//=========================== header ==========================
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "mfcc.h"
#include "model_manager.h"

#include <esp_heap_caps.h>
#include "model.h"

//=========================== variables ===========================
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* model_input = nullptr;
TfLiteTensor* model_output = nullptr;

const int kTensorArenaSize = 70 * 1024;
uint8_t tensor_arena[kTensorArenaSize];

uint8_t max_index[1] = {0};
float max_value = 0;

uint8_t speech_data_index = 0;
int32_t data_head = 0;
int16_t audio_data[16000];
uint8_t received_sensor_data[400];

QueueHandle_t xQueueSensorData = NULL;
//=========================== prototypes ==========================

//=========================== public ==============================
void model_setup() {
  ESP_LOGI(MODEL_MANAGER_TAG, "model setup start");

  // queue 연결
  xQueueSensorData = xQueueCreate(5, sizeof(received_sensor_data));

  model = tflite::GetModel(model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    MicroPrintf("Model provided is schema version %d not equal to supported "
                "version %d.", model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  static tflite::MicroMutableOpResolver<9> micro_op_resolver;
  if (micro_op_resolver.AddFullyConnected() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddRelu() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddSoftmax() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddConv2D() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddDepthwiseConv2D() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddMaxPool2D() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddReshape() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddQuantize() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddDequantize() != kTfLiteOk) {
    return;
  }

  // Build an interpreter to run the model with.
  static tflite::MicroInterpreter static_interpreter(
      model, micro_op_resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors.
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    MicroPrintf("AllocateTensors() failed");
    return;
  }

  // Get information about the memory area to use for the model's input.
  model_input = interpreter->input(0);
  model_output = interpreter->output(0);
  ESP_LOGI(MODEL_MANAGER_TAG, "Complete model setup!");
}

QueueHandle_t get_sensor_data_queue()
{
  return xQueueSensorData;
}

//=========================== tasks ===============================
void model_inference_task(void * arg)
{ 
  
  mfcc_init();
  ESP_LOGI(MODEL_MANAGER_TAG,"model_inference_task start!");

  while(1) {
    if (xQueueReceive(xQueueSensorData, &received_sensor_data, portMAX_DELAY))
    { 
      for(uint16_t i=0;i<400;i+=2)
      {
        uint8_t byte1 = received_sensor_data[i];
        uint8_t byte2 = received_sensor_data[i + 1];
        audio_data[speech_data_index * 200 + i / 2] = (byte2 << 8) | byte1;
      }
      speech_data_index++;
      
      if(speech_data_index == 80)
      {
        //여기서 추론시작
        ESP_LOGI(MODEL_MANAGER_TAG, "speech inference");
        speech_data_index = 0;
        data_head = 0; 
        float *freq_data = (float *)heap_caps_malloc(NUM_FRAMES * NUM_FBANK_BINS * sizeof(float), MALLOC_CAP_SPIRAM);
        memset(freq_data, 0, sizeof(float) * (NUM_FRAMES * NUM_FBANK_BINS));
        if (freq_data == NULL) {
          ESP_LOGE(MODEL_MANAGER_TAG, "freq malloc failed");
        }

        ESP_LOGI("good","---------------------freq-------------------------");
        // printf("[");
        for (uint8_t i = 0; i < 49; i++) {
          // printf("[");
          // //stft 변환
          mfcc_compute(audio_data + (i * FRAME_SHIFT), &freq_data[data_head]);
          data_head += NUM_FBANK_BINS;
          // printf("],");
          // vTaskDelay(1);
        }
        // for (int16_t k = 0; k <NUM_FRAMES * NUM_FBANK_BINS; k++) {
        //     printf("%f,", freq_data[k]);
        // }
        // printf("]\n\n\n");
        // ESP_LOGI("good","---------------------normalize-------------------------");
        // normalize(freq_data);
        // printf("[");
        // for (int16_t k = 0; k <NUM_FRAMES * NUM_FBANK_BINS; k++) {
        //     printf("%f,", freq_data[k]);
        // }
        // printf("]");
        // ESP_LOGI("good","---------------------normalize-------------------------");

        // printf("[");
        // for (int samp = 0; samp < 16000; samp++){
        //     printf("%d,", audio_data[samp]);
        //     if (samp % 3200 == 0) vTaskDelay(1);
        // }
        // printf("],");

        ESP_LOGI("test","Input size: %u",model_input->bytes / 4);
        ESP_LOGI("test", "Output size: %d", model_output->bytes / 4);

        for (int i = 0; i < model_input->bytes; ++i) {
          model_input->data.uint8[i] = 0.0f;
        }

        // float x_ratio = static_cast<float>(NUM_FBANK_BINS - 1) / (NUM_FBANK_BINS - 1);
        // float y_ratio = static_cast<float>(NUM_FRAMES - 1) / (NUM_FRAMES - 1);

        // for (uint8_t i = 0; i < NUM_FRAMES; i++) {
        //   for (uint8_t j = 0; j < NUM_FBANK_BINS; j++) {
        //     int x_l = static_cast<int>(x_ratio * j);
        //     int y_l = static_cast<int>(y_ratio * i);
        //     int x_h = static_cast<int>(ceil(x_ratio * j));
        //     int y_h = static_cast<int>(ceil(y_ratio * i));

        //     float x_weight = (x_ratio * j) - x_l;
        //     float y_weight = (y_ratio * i) - y_l;

        //     float a = freq_data[y_l * NUM_FBANK_BINS + x_l];
        //     float b = freq_data[y_l * NUM_FBANK_BINS + x_h];
        //     float c = freq_data[y_h * NUM_FBANK_BINS + x_l];
        //     float d = freq_data[y_h * NUM_FBANK_BINS + x_h];

        //     float pixel = a * (1 - x_weight) * (1 - y_weight)
        //           + b * x_weight * (1 - y_weight)
        //           + c * y_weight * (1 - x_weight)
        //           + d * x_weight * y_weight;

        //     model_input->data.f[i*NUM_FBANK_BINS + j] = pixel;
        //     // if(i == 0) {
        //     // ESP_LOGI("test", "input Data: %f", pixel);  
        //     // }
        //   }
        // }

        // ESP_LOGI("good","---------------------input-------------------------");
        // printf("model input index = %d \n",model_input->bytes);
        for (int16_t i = 0; i <  model_input->bytes/4; i++) {
            model_input->data.f[i] = freq_data[i];
           // printf("[%d] model input: %f, freq_data: %f\n", i,model_input->data.f[i],freq_data[i]);
            // if(i == 0) {
            // ESP_LOGI("test", "input Data: %f", pixel);  
            // }
        }
        heap_caps_free(freq_data);
        TfLiteStatus invokeStatus = interpreter->Invoke();
        if (invokeStatus != kTfLiteOk)
        {
          ESP_LOGE("test", "Invoke failed!");
          while (1);
          return;
        }
        max_index[1] = 0;
        max_value = 0;
        for (int i = 0; i < 5; i++) {
          float _value = model_output->data.f[i];
          if (_value > max_value)
          {
            max_value = _value;
            max_index[0] = i;
          }
          ESP_LOGI("test","%d:  %f",i, _value);
        }
        ESP_LOGI("test","Winner:  %d",max_index[0]);
      }

    }
  }
}



//=========================== private ==============================

