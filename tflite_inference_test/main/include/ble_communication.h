#ifndef TEMPLATE_H
#define TEMPLATE_H

//=========================== header ===========================
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "state_controller.h"

//=========================== define ===========================
#define BLE_COMMUNICATION_TAG "BLE_communication"
#define BLE_TRANSFER_TAG "Model transfer"

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define GATTS_IF_NUM                3
#define ESP_APP_ID                  0x55
#define SAMPLE_DEVICE_NAME          "GAMBA EDU KIT"
#define SVC_INST_ID                 0

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

#define CONFIG_SET_RAW_ADV_DATA

//=========================== typedef ===========================

//=========================== variables ===========================
// BLE characteristic
enum
{
    IDX_SVC,

    // motion 데이터 전송
    IDX_CHAR_imuDataTx,
    IDX_CHAR_VAL_imuDataTx,
    IDX_CHAR_CFG_imuDataTx,

    // speech 데이터 전송
    IDX_CHAR_speechDataTx,
    IDX_CHAR_VAL_speechDataTx,
    IDX_CHAR_CFG_speechDataTx,

    // vision 데이터 전송
    IDX_CHAR_visionDataTx,
    IDX_CHAR_VAL_visionDataTx,
    IDX_CHAR_CFG_visionDataTx,

    // 추론 결과 전송
    IDX_CHAR_inferenceTx,
    IDX_CHAR_VAL_inferenceTx,
    IDX_CHAR_CFG_inferenceTx,

    // 무슨 프로젝트인지(모션, 음성, 비전)
    IDX_CHAR_typeSignal,
    IDX_CHAR_VAL_typeSignal,
    
    // 캡쳐 시작
    IDX_CHAR_recordingRx,
    IDX_CHAR_VAL_recordingRx,

    // HW Button을 통해 캡쳐 시작을 웹에게 알림, 웹에서는 받을 준비를 하고 recordingRx를 통해 캡쳐 시작을 결정
    IDX_CHAR_beginRecordingTx,
    IDX_CHAR_VAL_beginRecordingTx,
    IDX_CHAR_CFG_beginRecordingTx,

    // 전송을 허락한다는 메세지를 웹으로 보냄
    IDX_CHAR_transferPermissionTx,
    IDX_CHAR_VAL_transferPermissionTx,
    IDX_CHAR_CFG_transferPermissionTx,



    // 모델 전송이 완료되었는지
    IDX_CHAR_hasModelTx,
    IDX_CHAR_VAL_hasModelTx,
    IDX_CHAR_CFG_hasModelTx,

    // 모델 전송 관련 - 블럭단위 파일
    IDX_CHAR_fileBlock,
    IDX_CHAR_VAL_fileBlock,

    // 모델 전송 관련 - 파일 길이
    IDX_CHAR_fileLength,
    IDX_CHAR_VAL_fileLength,

    // 모델 전송 관련 - 파일 최대 길이
    IDX_CHAR_fileMaximumLength,
    IDX_CHAR_VAL_fileMaximumLength,

    // 모델 전송 관련 - 체크섬
    IDX_CHAR_fileChecksum,
    IDX_CHAR_VAL_fileChecksum,

    // 모델 전송 관련 - 모델 전송 명령
    IDX_CHAR_command,
    IDX_CHAR_VAL_command,

    // 모델 전송 관련 - 전송 상태
    IDX_CHAR_transferStatus,
    IDX_CHAR_VAL_transferStatus,
    IDX_CHAR_CFG_transferStatus,

    // 모델 전송 관련 - 에러 메세지
    IDX_CHAR_errorMessage,
    IDX_CHAR_VAL_errorMessage,
    IDX_CHAR_CFG_errorMessage,



    /* 여기서 부터는 모델 전송 시 사전 필요한 meta data */

    // 모델 타입(cnn,rnn등 나중에 추가할것 후순위 Todo)
    IDX_CHAR_modelTypeRx,
    IDX_CHAR_VAL_modelTypeRx,

    // 모델에서 사용하는 센서 타입
    IDX_CHAR_sensorTypeRx,
    IDX_CHAR_VAL_sensorTypeRx,

    // 모델의 클래스 갯수, model output size
    IDX_CHAR_numClassesRx,
    IDX_CHAR_VAL_numClassesRx,

    // 모델 클래스의 라벨 이름들
    IDX_CHAR_classesNamesRx,
    IDX_CHAR_VAL_classesNamesRx,

    IDX_CHAR_classesNamesTx,
    IDX_CHAR_VAL_classesNamesTx,
    IDX_CHAR_CFG_classesNamesTx,
    
    // 추론시 딜레이 시간(아마도 motion에서만 사용)
    IDX_CHAR_captureDelayRx,
    IDX_CHAR_VAL_captureDelayRx,

    // 모션데이터 캡쳐 threshhlod(아마도 motion에서만 사용)
    IDX_CHAR_thresholdRx,
    IDX_CHAR_VAL_thresholdRx,

    GEK_IDX_NB, //gamba edu kit ble index 갯수
};

//웹으로 부터 전송 받는 타입 시그널(현재 어떤 프로젝트와 연결되어 있는지, 요구하는게 무엇인지)
enum {
    TYPE_TRAINER_MOTION = 0,
    TYPE_TRAINER_SPEECH,
    TYPE_TRAINER_VISION,
    TYPE_UPLOAD_MOTION,
    TYPE_UPLOAD_SPEECH,
    TYPE_UPLOAD_VISION,
    TYPE_APPLICATION_MOTION,
    TYPE_APPLICATION_SPEECH,
    TYPE_APPLICATION_VISION,
    TYPE_CHECK_MODEL_MOTION,
    TYPE_CHECK_MODEL_SPEECH,
    TYPE_CHECK_MODEL_VISION,
};

//=========================== prototypes ===========================
//ble 서비스, 이벤트 감지 시작
void ble_begin(void);

//전송받은 모델을 nvs에 저장(성공시 true반환)
bool write_nvs_transferred_model();

#ifdef __cplusplus
extern "C" {
#endif
// 데이터(센서값, 추론값등)를 ble를 통해 전송
void send_data_to_ble(void* data, size_t size, ble_data_t data_type);
#ifdef __cplusplus
}
#endif

//=========================== tasks ===========================


#endif