#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//=========================== define ===========================
// 상수 정의
#define BLE_COMMUNICATION_TAG "BLE_communication"
#define BLE_TRANSFER_TAG "Model transfer"

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define GATTS_IF_NUM                3
#define ESP_APP_ID                  0x55
#define SAMPLE_DEVICE_NAME          "BLE_TEST_DEVICE"
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
// enum 정의 (숫자에 이름 붙이기)
enum
{
    IDX_SVC,

    // 특정 데이터 전송
    IDX_CHAR_dataTx,
    IDX_CHAR_VAL_dataTx,
    IDX_CHAR_CFG_dataTx,

    GEK_IDX_NB, //ble index 갯수
};

//=========================== prototypes ===========================
//ble 서비스, 이벤트 감지 시작
//공개 함수 프로토타입
void ble_begin(void);
void send_data_to_ble(void* data, size_t size);
bool check_ble_connect(void);