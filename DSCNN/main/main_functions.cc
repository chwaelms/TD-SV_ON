/* Copyright 2020 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/


#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "main_functions.h"
#include "model.h"

// Globals, used for compatibility with Arduino-style sketches.
namespace {
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;

constexpr int kTensorArenaSize = 800000;
uint8_t* tensor_arena = nullptr;
}

float received_data[KINPUT_SIZE] = {0.0f};
float model_output_data[KOUTPUT_SIZE] = {0.0f};

QueueHandle_t xQueueInputData = NULL;
QueueHandle_t xQueueOutputData = NULL;

QueueHandle_t get_input_data_queue()
{
  return xQueueInputData;
}

QueueHandle_t get_output_data_queue()
{
  return xQueueOutputData;
}

// The name of this function is important for Arduino compatibility.
void model_setup() {

  xQueueInputData = xQueueCreate(1, sizeof(float) * KINPUT_SIZE);
  xQueueOutputData = xQueueCreate(1, sizeof(float) * KOUTPUT_SIZE);
  tensor_arena = (uint8_t*) heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_SPIRAM);
  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  model = tflite::GetModel(g_model);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    MicroPrintf("Model provided is schema version %d not equal to supported "
                "version %d.", model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  static tflite::MicroMutableOpResolver<19> micro_op_resolver;
  if (micro_op_resolver.AddDepthwiseConv2D() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddFullyConnected() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddQuantize() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddDequantize() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddMean() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddSub() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddMul() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddAdd() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddRsqrt() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddTranspose() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddLogistic() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddExpandDims() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddConv2D() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddPad() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddReshape() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddSplitV() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddSpaceToBatchNd() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddBatchToSpaceNd() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddSoftmax() != kTfLiteOk) {
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

  // Obtain pointers to the model's input and output tensors.
  input = interpreter->input(0);
  output = interpreter->output(0);
  ESP_LOGI("main","input type=%d", input->type); 
  ESP_LOGI("main","output type=%d", output->type);
}

// The name of this function is important for Arduino compatibility.
void model_inference_task(void * arg) {
  ESP_LOGI("model", "model_inference_task created");
  while(1) {
    for(int i=0; i<KINPUT_SIZE; i++) {
      // 가짜 더미 데이터 생성(49*40 mfcc 입력 가정)
      input->data.f[i] = 1.0f / (i + 1);
    }

    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk) {
        MicroPrintf("Invoke failed");
        return;
    }

    ESP_LOGI("model", "Invoke successful");
    float max = -99.0f;
    int max_index = -1;
    for(int i=0; i<KOUTPUT_SIZE; i++) {
      if(output->data.f[i] > max) {
        max = output->data.f[i];
        max_index = i;
      }
    }
    ESP_LOGI("model", "Max value: %f at index %d", max, max_index);
  }
}

// void model_inference_task(void * arg) {
//   ESP_LOGI("model", "model_inference_task created");

//   while(1) {
//     if (xQueueReceive(xQueueInputData, &received_data, portMAX_DELAY)) {
//       memcpy(input->data.f, received_data, sizeof(float) * KINPUT_SIZE);

//       TfLiteStatus invoke_status = interpreter->Invoke();
//       if (invoke_status != kTfLiteOk) {
//           MicroPrintf("Invoke failed");
//           return;
//       }
//       memcpy(model_output_data, output->data.f, sizeof(float) * KOUTPUT_SIZE);

//       xQueueSend(xQueueOutputData, &model_output_data, portMAX_DELAY);
//     }
//   }
// }
