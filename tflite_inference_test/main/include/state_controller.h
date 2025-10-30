#ifndef STATE_CONTROLLER_H
#define STATE_CONTROLLER_H

//=========================== header ==========================
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "imu_provider.h"
#include "speech_provider.h"
#include "vision_provider.h"
#include "model_manager.h"

//=========================== define ===========================
#define STATE_CONTROLLER_TAG    "state_controller"  // log tag

//=========================== typedef ===========================

// 키트 상태 타입
typedef enum{
    KIT_STATE_IDLE = 0,
    KIT_STATE_TRAIN_MOTION,
    KIT_STATE_TRAIN_SPEECH,
    KIT_STATE_TRAIN_VISION,
    KIT_STATE_INFERENCE_MOTION,
    KIT_STATE_INFERENCE_SPEECH,
    KIT_STATE_INFERENCE_VISION,
    KIT_STATE_UPLOAD_MOTION,
    KIT_STATE_UPLOAD_SPEECH,
    KIT_STATE_UPLOAD_VISION
} kit_state_t;

// BLE 상태 타입
typedef enum{
    BLE_STATE_IDLE = 0,
    BLE_STATE_DISCONNECT,
    BLE_STATE_CONNECT
} ble_state_t;

// 키트에 발생하는 시그널 타입
typedef enum{
    SET_STATE_IDLE = 0,
    SET_STATE_TRAIN_MOTION,
    SET_STATE_TRAIN_SPEECH,
    SET_STATE_TRAIN_VISION,
    SET_STATE_UPLOAD_MOTION,
    SET_STATE_UPLOAD_SPEECH,
    SET_STATE_UPLOAD_VISION,
    SET_STATE_INFERENCE_MOTION,
    SET_STATE_INFERENCE_SPEECH,
    SET_STATE_INFERENCE_VISION,
    VALIDATE_MODEL_MOTION,
    VALIDATE_MODEL_SPEECH,
    VALIDATE_MODEL_VISION,
    COMPLETE_MODEL_UPLOAD,
    START_SEND_DATA,
    BUTTON_A_PRESSED,
    BUTTON_B_PRESSED,
    BUTTON_X_PRESSED,
    BUTTON_Y_PRESSED,
} kit_signal_t;

// Task 관리 용도로 쓰일것임, task handler
enum
{
    IDX_STATE_CONTROLLER_TASK = 0,
    IDX_DUMMY_TASK,
    IDX_SPEECH_PROVIDER_TASK,
    IDX_SEND_DATA_TASK, 
    IDX_IMU_PROVIDER_TASK,
    IDX_MODEL_INFERENCE_TASK,
    IDX_VISION_PROVIDER_TASK,
    IDX_VISION_DISPLAY_TASK,
    IDX_INFO_DISPLAY_TASK,

    IDX_TASK_COUNT_NB
};

// BLE 전송시 데이터 타입
typedef enum {
    IMU_DATA = 0,
    SPEECH_DATA,
    VISION_DATA,
    RECORDING_SIGNAL_DATA,
    INFERENCE_DATA,
    TRANSFER_SIGNAL_DATA,
    CLASSES_NAMES_DATA,
    HAS_MODEL_DATA
} ble_data_t;

// queue를 통해 주고 받을 때의 데이터 타입
typedef struct {
    ble_data_t type;
    union {
        imu_db_t imu_data;
        speech_db_t speech_data;
        vision_db_t vision_data;
    } data;
} send_data_t;




//=========================== variables ===========================

//=========================== prototypes ===========================

//signal 생성(signal에 따라 state가 변경될 수 있고 아닐 수도 있다.)
void kit_signaling(kit_signal_t t);

//현재 상태 변경, 여기서는 현재 키트 상태를 변경, 그에 맞는 테스크 생성만 수행
void transit_kit_state(kit_state_t t);

//현재 ble 연결 상태 변경
void transit_ble_state(ble_state_t t);

//현재 키트 상태 반환
kit_state_t get_kit_state(void);

//현재 ble 연결 상태 반환
ble_state_t get_ble_state(void);

//=========================== tasks ===========================

// state controll을 담당하는 task
void state_controller_task(void * arg);

#endif