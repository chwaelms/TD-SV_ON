#ifndef BLE_COMMUNICATION_H
#define BLE_COMMUNICATION_H

//=========================== header ===========================
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//=========================== define ===========================
#define BLE_COMMUNICATION_TAG "BLE_communication"

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define GATTS_IF_NUM                3
#define ESP_APP_ID                  0x55
#define SAMPLE_DEVICE_NAME          "ESP_TSF"
#define SVC_INST_ID                 0

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

#define CONFIG_SET_RAW_ADV_DATA

#define BLE_BLOCK_SIZE              100
//=========================== typedef ===========================

typedef struct {
    uint8_t data[BLE_BLOCK_SIZE]; // 데이터 조각을 담을 버퍼
    size_t len;                   // 실제 데이터 조각의 길이
} ble_chunk_t;

//=========================== variables ===========================
// BLE characteristic
enum
{
    IDX_SVC,

    IDX_CHAR_inputDataRx,
    IDX_CHAR_VAL_inputDataRx,

    IDX_CHAR_outputDataTx,
    IDX_CHAR_VAL_outputDataTx,
    IDX_CHAR_CFG_outputDataTx,

    BLE_IDX_NB, //ble index 갯수
};

//=========================== prototypes ===========================
#ifdef __cplusplus
extern "C" {
#endif

//ble 서비스, 이벤트 감지 시작
void ble_begin(void);

//=========================== tasks ===========================
// 데이터 통신을 위한 태스크
void data_send_task(void *arg);
void data_receive_task(void *arg);


// =======================================================
#ifdef __cplusplus
}
#endif

#endif