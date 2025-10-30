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
//=========================== define ===========================
#define MODEL_MANAGER_TAG "MODEL_MANAGER"


//=========================== typedef ===========================



#ifdef __cplusplus
extern "C" {
#endif

//=========================== variables ===========================

//=========================== prototypes ===========================



// nvs에 저장된 모델을 불러와 setup(추론을 위한 준비)
void model_setup();

//model inference task에서 센서 데이터를 받을 때 사용하는 queue handler반환
QueueHandle_t get_sensor_data_queue();



//=========================== tasks ===========================
void model_inference_task(void * arg);


#ifdef __cplusplus
}
#endif

#endif