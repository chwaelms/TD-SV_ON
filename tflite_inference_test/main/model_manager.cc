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
#include "ble_communication.h"
#include "state_controller.h"
#include "vision_provider.h"
#include "esp_camera.h"

#include <esp_heap_caps.h>

//=========================== variables ===========================

uint8_t model_data[MODEL_MAX_LENGTH] __attribute__((section(".ext_ram.bss")));
uint8_t *dynamic_model_data = &model_data[0];

float imu_buf[6];

uint8_t speech_data_index = 0;

// 요구 모델 정보를 저장할 변수
model_info_db_t requirement_model_info;
// 현재 모델 정보를 저장할 변수
model_info_db_t model_info;

uint8_t *new_model_file_data;

const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* model_input = nullptr;
TfLiteTensor* model_output = nullptr;

const int kTensorArenaSize = 300 * 1024;
uint8_t tensor_arena[kTensorArenaSize] __attribute__((section(".ext_ram.bss")));

uint8_t num_classes = 0;
uint8_t numSamples = 20;
uint8_t samplesRead = 20;
float acceleration_threshold = 0.03;
uint8_t inference_delay_time = 50;
TickType_t start_time = 0;
TickType_t current_time;

float accel_sum;

uint8_t max_index[1] = {0};
float max_value = 0;

int32_t data_head = 0;

int16_t audio_data[16000] __attribute__((section(".ext_ram.bss")));

//float freq_data[NUM_FRAMES * NUM_BINS]; // [NUM_FRAMES * NUM_BINS]

QueueHandle_t xQueueSensorData = NULL;
send_data_t received_sensor_data;

int8_t input_frame[RESIZE_ROWS * RESIZE_COLS * NUM_CHANNEL] __attribute__((section(".ext_ram.bss")));
//=========================== prototypes ==========================

//nvs에 저장된 모델이 존재하는지
bool _is_model_in_nvs();
//nvs에 저장된 모델을 읽어옴
bool _read_model_nvs(void);
//=========================== public ==============================
void model_setup() {
  ESP_LOGI(MODEL_MANAGER_TAG, "model setup start");

  model_info = read_model_info_nvs();
  acceleration_threshold = model_info.threshhlod;
  inference_delay_time = model_info.capture_delay;
  num_classes = model_info.num_classes;
  // queue 연결
  xQueueSensorData = xQueueCreate(5, sizeof(send_data_t));
  if(!_read_model_nvs())
  {
    ESP_LOGE(MODEL_MANAGER_TAG, "Fail model read");
    return;
  }
  model = tflite::GetModel(dynamic_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    MicroPrintf("Model provided is schema version %d not equal to supported "
                "version %d.", model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  static tflite::MicroMutableOpResolver<11> micro_op_resolver;
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
  if(micro_op_resolver.AddMul() != kTfLiteOk) {
    return;
  }
  if(micro_op_resolver.AddAdd() != kTfLiteOk) {
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

void set_model_type(uint8_t* value)
{
  switch (*value)
  {
    case 0:
      requirement_model_info.model_type = MODEL_CNN;
      break;
    
    case 1:
      requirement_model_info.model_type = MODEL_RNN;
      break;
    
    case 2:
      requirement_model_info.model_type = MODEL_KNN;
      break;
    
    default:
      requirement_model_info.error_flag = 1;
      ESP_LOGE(MODEL_MANAGER_TAG, "Error: store model type");
      break;
  }
}

void set_sensor_type(uint8_t* value)
{
  switch (*value)
  {
    case 0:
      requirement_model_info.sensor_type = MOTION_SENSOR;
      break;
    
    case 1:
      requirement_model_info.sensor_type = SPEECH_SENSOR;
      break;
    
    case 2:
      requirement_model_info.sensor_type = VISION_SENSOR;
      break;
    
    default:
      requirement_model_info.error_flag = 1;
      ESP_LOGE(MODEL_MANAGER_TAG, "Error: store sensor type");
      break;
  }
}

void set_num_classes(uint8_t* value)
{
  if (*value == 0)
  {
    requirement_model_info.error_flag = 1;
    ESP_LOGE(MODEL_MANAGER_TAG, "Error: store num classes");
    return;
  }
  requirement_model_info.num_classes = *value;
}

void set_classes_names(esp_ble_gatts_cb_param_t *param)
{
  if (param->write.len > sizeof(requirement_model_info.classes_names) - 1) {
    ESP_LOGE(MODEL_MANAGER_TAG, "Error: Classes names data exceeds the limit");
    requirement_model_info.error_flag = 1;
    return;
  }

  strncpy((char*)requirement_model_info.classes_names, (char*)param->write.value, param->write.len);
  requirement_model_info.classes_names[param->write.len] = '\0';

  ESP_LOGI(MODEL_MANAGER_TAG, "Stored Classes Names: %s", requirement_model_info.classes_names);
}

void set_capture_delay(uint8_t* value)
{
  if (*value == 0)
  {
    requirement_model_info.error_flag = 1;
    ESP_LOGE(MODEL_MANAGER_TAG, "Error: store capture delay");
    return;
  }
  requirement_model_info.capture_delay = *value;
}

void set_threshold(esp_ble_gatts_cb_param_t *param)
{
  uint32_t uintValue = 0;
  uint8_t* data = param->write.value;
  for(int i=0;i<param->write.len;i++) {
    uintValue = (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
  }
  float floatValue;
  memcpy(&floatValue, &uintValue, sizeof(float));
  requirement_model_info.threshhlod = floatValue;
  ESP_LOGI(MODEL_MANAGER_TAG, "%f", floatValue);
}

void set_model_length(uint32_t value)
{
  if (value > MODEL_MAX_LENGTH)
  {
    requirement_model_info.error_flag = 1;
    ESP_LOGE(MODEL_MANAGER_TAG, "Error: store model length");
    return;
  }
  requirement_model_info.model_legth = value;
}

bool check_validation_model_info()
{
  if(!_is_model_in_nvs())
  {
    ESP_LOGE(MODEL_MANAGER_TAG, "Model not found in NVS");
    return false;
  }

  model_info_db_t nvs_model_info = read_model_info_nvs();

  if (nvs_model_info.model_type != requirement_model_info.model_type)
  {
    ESP_LOGE(MODEL_MANAGER_TAG, "Invalidate model type!");
    return false;
  }

  if (nvs_model_info.sensor_type != requirement_model_info.sensor_type)
  {
    ESP_LOGE(MODEL_MANAGER_TAG, "Invalidate sensor type!");
    return false;
  }

  if (nvs_model_info.num_classes != requirement_model_info.num_classes)
  {
    ESP_LOGE(MODEL_MANAGER_TAG, "Invalidate num_classes! model: %d, require: %d", nvs_model_info.num_classes, requirement_model_info.num_classes);
    return false;
  }

  ESP_LOGI(MODEL_MANAGER_TAG, "Validate model!");
  return true;
}

bool write_model_nvs(uint8_t* data, uint32_t len)
{
    ESP_LOGI(NVS_WRITE_TAG, "Write model start!");
    esp_err_t err;
    nvs_handle_t wHandle;
    
    if (len - sizeof(uint32_t) == 0){
      ESP_LOGE(NVS_WRITE_TAG, "Nothing to write");
      return false;
    }
    err = nvs_open("storage", NVS_READWRITE, &wHandle);
    if (err != ESP_OK){
      ESP_LOGE(NVS_WRITE_TAG, "failed to open nvs");
      return false;
    }
    err = nvs_set_blob(wHandle, "model", data, len);
    if (err != ESP_OK){
      ESP_LOGE(NVS_WRITE_TAG, "Failed to write model blob [%s]", esp_err_to_name(err));
      return false;
    }

    err = nvs_set_blob(wHandle, "model_info", &requirement_model_info, sizeof(requirement_model_info));
    if (err != ESP_OK){
      ESP_LOGE(NVS_WRITE_TAG, "Failed to write model info blob [%s]", esp_err_to_name(err));
      return false;
    }

    err = nvs_commit(wHandle);
    if (err != ESP_OK){
      ESP_LOGE(NVS_WRITE_TAG, "Failed to commit blob [%d]", err);
      return false;
    }

    nvs_close(wHandle);
    
    return true;
}

bool erase_all_nvs_data() {
  esp_err_t err;

  // NVS 열기
  nvs_handle_t handle;
  err = nvs_open("storage", NVS_READWRITE, &handle);
  if (err != ESP_OK) {
      ESP_LOGE(NVS_DELETE_TAG, "Failed to open NVS [%s]", esp_err_to_name(err));
      return false;
  }

  // 모든 데이터 삭제
  err = nvs_erase_all(handle);
  if (err != ESP_OK) {
      ESP_LOGE(NVS_DELETE_TAG, "Failed to erase all NVS data [%s]", esp_err_to_name(err));
      nvs_close(handle);
      return false;
  }

  // 커밋하여 변경사항을 적용
  err = nvs_commit(handle);
  if (err != ESP_OK) {
      ESP_LOGE(NVS_DELETE_TAG, "Failed to commit NVS after erasing all data [%s]", esp_err_to_name(err));
      nvs_close(handle);
      return false;
  }

  ESP_LOGI(NVS_DELETE_TAG, "All data erased from NVS");
  
  // NVS 닫기
  nvs_close(handle);

  return true;
}

model_info_db_t read_model_info_nvs(void)
{
  model_info_db_t model_info;

  esp_err_t err;
  size_t required_size = 0;
  nvs_handle_t rHandle;

  err = nvs_open("storage", NVS_READWRITE, &rHandle);
  if (err != ESP_OK){
    ESP_LOGE(MODEL_MANAGER_TAG, "NVS Open Failed");
    nvs_close(rHandle);
  }

  // Read size of blob (binary large object)
  err = nvs_get_blob(rHandle, "model_info", NULL, &required_size);
  if (err != ESP_OK){
    ESP_LOGE(MODEL_MANAGER_TAG, "NVS BLOB error or not found");
    nvs_close(rHandle);
  }
  ESP_LOGI(MODEL_MANAGER_TAG, "Read %u bytes from NVS", required_size);

  // size check
  if (required_size == 0){
    ESP_LOGE(MODEL_MANAGER_TAG, "No model info");
    nvs_close(rHandle);
  }

  err = nvs_get_blob(rHandle, "model_info", &model_info, &required_size);
  if (err != ESP_OK){
    ESP_LOGE(MODEL_MANAGER_TAG, "Error: get model info");
    nvs_close(rHandle);
  }
  ESP_LOGI(MODEL_MANAGER_TAG, "Model info get complete!");

  nvs_close(rHandle);

  return model_info;
}

QueueHandle_t get_sensor_data_queue()
{
  return xQueueSensorData;
}

bool _read_model_nvs(void)
{
  esp_err_t err;
  size_t required_size = 0;
  nvs_handle_t rHandle;

  err = nvs_open("storage", NVS_READWRITE, &rHandle);
  if (err != ESP_OK){
    ESP_LOGE(MODEL_MANAGER_TAG, "NVS Open Failed");
    return false;
  }

  // Read size of blob (binary large object)
  err = nvs_get_blob(rHandle, "model", NULL, &required_size);
  if (err != ESP_OK){
    ESP_LOGE(MODEL_MANAGER_TAG, "NVS BLOB error or not found");
    return false;
  }
  ESP_LOGI(MODEL_MANAGER_TAG, "Read %u bytes from NVS", required_size);

  // size check
  if (required_size == 0){
    ESP_LOGE(MODEL_MANAGER_TAG, "No model");
    return false;
  }

  err = nvs_get_blob(rHandle, "model", dynamic_model_data, &required_size);
  if (err != ESP_OK){
    ESP_LOGE(MODEL_MANAGER_TAG, "Error: get model");
    return false;
  }
  ESP_LOGI(MODEL_MANAGER_TAG, "Model get complete!");

  // //length check
  // // memcpy(&(file_length_value), newModelFileData, sizeof(uint32_t));
  // // ESP_LOGI(MODEL_MANAGER_TAG, "read %u bytes & size %ld", required_size, file_length_value);
  // // _bufTest("NVS", ctrlApp_var.memBuf);
  // if(*newModelFileData == NULL) {
  //   ESP_LOGI(MODEL_MANAGER_TAG, "Model data is NULL");
  // }
  // else {
  //   ESP_LOGI(MODEL_MANAGER_TAG, "Model data is not NULL");
  // }
  
  nvs_close(rHandle);

  return true;
}

//=========================== tasks ===============================
void model_inference_task(void * arg)
{ 
  mfcc_init();
  ESP_LOGI(MODEL_MANAGER_TAG,"model_inference_task start!");

  while(1) {
    if (xQueueReceive(xQueueSensorData, &received_sensor_data, portMAX_DELAY))
    { 
      switch (received_sensor_data.type)
      {
        case IMU_DATA:
        {
          current_time = xTaskGetTickCount();
          if (current_time - start_time < inference_delay_time)
          {
            continue;
          }
          memcpy(imu_buf, received_sensor_data.data.imu_data.buf, sizeof(imu_buf));
          if (samplesRead == numSamples) {
            accel_sum = (fabs(imu_buf[0]) + fabs(imu_buf[1]) + fabs(imu_buf[2]) + fabs(imu_buf[3]) + fabs(imu_buf[4]) + fabs(imu_buf[5])) / 6.0;
            // check if it's above the threshold
            if (accel_sum >= acceleration_threshold) {
              samplesRead = 0;
              for (int i = 0; i < model_input->bytes; ++i) {
                model_input->data.uint8[i] = 0.0f;
              }
            }
          }
          if (samplesRead < numSamples) {
            // input에 데이터 전달
            for(int i=0;i<MOTION_DATA_LENGTH;i++)
            {
              model_input->data.f[samplesRead * MOTION_DATA_LENGTH + i] = imu_buf[i];
            }
            samplesRead++;

            if (samplesRead == numSamples)
            {
              // Run inferencing
              TfLiteStatus invokeStatus = interpreter->Invoke();
              if (invokeStatus != kTfLiteOk)
              {
                ESP_LOGE(MODEL_MANAGER_TAG, "Invoke failed!");
                while (1);
                return;
              }
              ESP_LOGI(MODEL_MANAGER_TAG,"Input size: %u",model_input->bytes / 4);
              ESP_LOGI(MODEL_MANAGER_TAG, "Output size: %d", model_output->bytes / 4);
              // Loop through the output tensor values from the model
              max_index[0] = 0;
              max_value = 0;
              for (int i = 0; i < num_classes; i++) {
                float _value = model_output->data.f[i];
                if (_value > max_value)
                {
                  max_value = _value;
                  max_index[0] = i;
                }
                ESP_LOGI("test","%d:  %f",i, _value);
              }
              ESP_LOGI("test","Winner:  %d",max_index[0]);
              send_data_to_ble(max_index, sizeof(max_index), INFERENCE_DATA);
              start_time = xTaskGetTickCount();
            }
          }
          break;
        }
        case SPEECH_DATA:
        {
          for(uint16_t i=0;i<400;i+=2)
          {
            uint8_t byte1 = received_sensor_data.data.speech_data.buf[i];
            uint8_t byte2 = received_sensor_data.data.speech_data.buf[i + 1];
            audio_data[speech_data_index * 200 + i / 2] = (byte2 << 8) | byte1;
            //printf("%d,",audio_data[speech_data_index * 200 + i / 2]);
            
          }
          //vTaskDelay(1);
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

            for (uint8_t i = 0; i < 49; i++) {
              // //stft 변환
              mfcc_compute(audio_data + (i * FRAME_SHIFT), &freq_data[data_head]);
              data_head += NUM_FBANK_BINS;
            }

            ESP_LOGI(MODEL_MANAGER_TAG,"Input size: %u",model_input->bytes / 4);
            ESP_LOGI(MODEL_MANAGER_TAG, "Output size: %d", model_output->bytes / 4);

            for (int i = 0; i < model_input->bytes; ++i) {
              model_input->data.uint8[i] = 0.0f;
            }

            for (int16_t i = 0; i <  model_input->bytes/4; i++) {
              model_input->data.f[i] = freq_data[i];
              //printf("%f,", model_input->data.f[i]);
            }
            heap_caps_free(freq_data);
            TfLiteStatus invokeStatus = interpreter->Invoke();
            if (invokeStatus != kTfLiteOk)
            {
              ESP_LOGE(MODEL_MANAGER_TAG, "Invoke failed!");
              while (1);
              return;
            }
            max_index[0] = 0;
            max_value = 0;
            for (int i = 0; i < model_output->bytes / 4; i++) {
              float _value = model_output->data.f[i];
              if (_value > max_value)
              {
                max_value = _value;
                max_index[0] = i;
              }
              ESP_LOGI("test","%d:  %f",i, _value);
            }
            ESP_LOGI("test","Winner:  %d",max_index[0]);
            send_data_to_ble(max_index, sizeof(max_index), INFERENCE_DATA);
          }
          break;
        }
        case VISION_DATA:
        {
          vision_db_t vision_data = received_sensor_data.data.vision_data;
          resize_and_convert_rgb_i8(vision_data.frame, input_frame);
          camera_fb_return(vision_data.frame);
          for (int i = 0; i < model_input->bytes; i++) {
            model_input->data.int8[i] = 0.0f;
          }

          for (int i = 0; i < sizeof(input_frame); i++) {
            model_input->data.int8[i] = input_frame[i];
          }
          
          TfLiteStatus invokeStatus = interpreter->Invoke();
          if (invokeStatus != kTfLiteOk)
          {
            ESP_LOGE("test", "Invoke failed!");
            return;
          }

          int8_t mask_score = model_output->data.int8[0];
          int8_t no_mask_score = model_output->data.int8[1];
          int8_t nothing_score = model_output->data.int8[2];

          ESP_LOGI("test","mask: %d no mask: %d nothing: %d",mask_score, no_mask_score, nothing_score);
          float person_score_f =
              (mask_score - model_output->params.zero_point) * model_output->params.scale;
          float no_person_score_f =
              (no_mask_score - model_output->params.zero_point) * model_output->params.scale;
          float nothing_score_f =
              (nothing_score - model_output->params.zero_point) * model_output->params.scale;

          ESP_LOGI("test","result -- mask: %f no mask: %f nothing: %f",person_score_f, no_person_score_f, nothing_score_f);
          //위 값들중 제일 큰 인덱스 전송
          max_index[0] = 0;
          max_value = -129;
          ESP_LOGI("test","%d", model_output->bytes);
          for (int i = 0; i < model_output->bytes; i++) {
            int8_t _value = model_output->data.int8[i];
            if (_value > max_value)
            {
              max_value = _value;
              max_index[0] = i;
            }
            ESP_LOGI("test","%d:  %d",i, _value);
          }
          ESP_LOGI("test","Winner:  %d",max_index[0]);
          send_data_to_ble(max_index, sizeof(max_index), INFERENCE_DATA);
          break;
        }
        default:
        {
          // 알 수 없는 데이터 형식 처리
          ESP_LOGE(MODEL_MANAGER_TAG, "data type error!");
          break;
        }
      }
    }
  }
}

//=========================== private ==============================

bool _is_model_in_nvs() {
  esp_err_t err;
  nvs_handle_t rHandle;

  // NVS 열기
  err = nvs_open("storage", NVS_READONLY, &rHandle);
  if (err != ESP_OK) {
      ESP_LOGE(NVS_READ_TAG, "Failed to open NVS [%s]", esp_err_to_name(err));
      return false;
  }

  // 모델 정보 읽기
  size_t modelSize;
  err = nvs_get_blob(rHandle, "model", NULL, &modelSize);
  if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
      ESP_LOGE(NVS_READ_TAG, "Failed to get model blob size [%s]", esp_err_to_name(err));
      nvs_close(rHandle);
      return false;
  }
  if (err == ESP_ERR_NVS_NOT_FOUND) {
      ESP_LOGI(NVS_READ_TAG, "Model not found in NVS");
      nvs_close(rHandle);
      return false;
  }

  // 모델 정보가 있는 경우에 추가적인 처리 가능
  ESP_LOGI(NVS_READ_TAG, "Model found in NVS with size %d", modelSize);
  nvs_close(rHandle);
  
  return true;
}

