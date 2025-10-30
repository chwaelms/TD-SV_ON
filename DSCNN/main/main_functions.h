/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.

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

#ifndef TENSORFLOW_LITE_MICRO_EXAMPLES_HELLO_WORLD_MAIN_FUNCTIONS_H_
#define TENSORFLOW_LITE_MICRO_EXAMPLES_HELLO_WORLD_MAIN_FUNCTIONS_H_

#define KINPUT_SIZE 49*40
#define KOUTPUT_SIZE 25

// Expose a C friendly interface for main functions.
#ifdef __cplusplus
extern "C" {
#endif

void model_setup();
void model_inference_task(void * arg);

QueueHandle_t get_input_data_queue();
QueueHandle_t get_output_data_queue();

#ifdef __cplusplus
}
#endif

#endif  // TENSORFLOW_LITE_MICRO_EXAMPLES_HELLO_WORLD_MAIN_FUNCTIONS_H_
