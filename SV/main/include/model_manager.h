#ifndef MODEL_MANAGER_H
#define MODEL_MANAGER_H

//=========================== header ==========================
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "state_controller.h"


//=========================== define ===========================
#define MODEL_MANAGER_TAG "MODEL_MANAGER"
#define CLASSES_NAMES_MAX_LENGTH 210
#define NVS_WRITE_TAG "writeNVS"
#define NVS_READ_TAG "readNVS"
#define NVS_DELETE_TAG "deleteNVS"

#define MOTION_DATA_LENGTH 6

#define MODEL_MAX_LENGTH 400 * 1024


//=========================== typedef ===========================

// 모델의 구조 타입(CNN,RNN 등)
typedef enum {
    MODEL_CNN = 0,
    MODEL_RNN,
    MODEL_KNN
} model_t;

// 모델이 사용하는 센서 타입
typedef enum {
    MOTION_SENSOR = 0,
    SPEECH_SENSOR,
    VISION_SENSOR
} kit_sensor_t;

// 모델 정보를 저장할 구조체
typedef struct {
    model_t model_type;
    kit_sensor_t sensor_type;
    uint8_t num_classes;
    uint8_t classes_names[CLASSES_NAMES_MAX_LENGTH]; // 클래스 10개, 이름 20자, ','갯수 9개 => 10*20+9
    uint32_t model_legth;

    // 아래 두개는 motion에서만 사용
    float threshhlod;
    uint8_t capture_delay;

    uint8_t error_flag;
} model_info_db_t;

#ifdef __cplusplus
extern "C" {
#endif

//=========================== variables ===========================

//=========================== prototypes ===========================



// nvs에 저장된 모델을 불러와 setup(추론을 위한 준비)
void model_setup();

// 모델 관련된 메타 데이터를 임시 저장, 모델을 nvs에 저장할때 같이 nvs에 저장 
void set_model_type(uint8_t* value);
void set_sensor_type(uint8_t* value);
void set_num_classes(uint8_t* value);
void set_classes_names(esp_ble_gatts_cb_param_t *param);
void set_capture_delay(uint8_t* value);
void set_threshold(esp_ble_gatts_cb_param_t *param);
void set_model_length(uint32_t value);

//모델 정보(메타 데이터)를 nvs에서 읽어와 model_info_db_t구조체로 반환
model_info_db_t read_model_info_nvs(void);

// 모델의 유효성 검사,(해당 application에 적절한 모델인지)
bool check_validation_model_info();
// ble를 통해 전송받은 model을 nvs에 저장
bool write_model_nvs(uint8_t* data, uint32_t len);
// nvs의 모든 데이터를 삭제
bool erase_all_nvs_data();

//model inference task에서 센서 데이터를 받을 때 사용하는 queue handler반환
QueueHandle_t get_sensor_data_queue();



//=========================== tasks ===========================
void model_inference_task(void * arg);


#ifdef __cplusplus
}
#endif

#endif