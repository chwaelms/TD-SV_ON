//=========================== header ==========================
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>

#include "ble_communication.h"
#include "state_controller.h"
#include "model_manager.h"

#include <esp_heap_caps.h>

//=========================== variables ===========================
// BLE advertising config done 여부확인
static uint8_t adv_config_done = 0;

// characteristic handles
uint16_t edu_kit_handle_table[GEK_IDX_NB];

// 전송을 위한 characteristic 핸들
uint16_t characteristic_handle = 0;

#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
        /* flags */
        0x02, 0x01, 0x06,
        /* tx power*/
        0x02, 0x0a, 0xeb,
        /* service uuid */
        0x03, 0x03, 0xFF, 0x00,
        /* device name */
        0x0f, 0x09, 'G', 'A', 'M', 'B', 'A', ' ', 'E', 'D', 'U', ' ', 'K', 'I', 'T'
};
static uint8_t raw_scan_rsp_data[] = {
        /* flags */
        0x02, 0x01, 0x06,
        /* tx power */
        0x02, 0x0a, 0xeb,
        /* service uuid */
        0x03, 0x03, 0xFF,0x00
};

#else
static uint8_t service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval        = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance          = 0x00,
    .manufacturer_len    = 0,    //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //test_manufacturer,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp        = true,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006,
    .max_interval        = 0x0010,
    .appearance          = 0x00,
    .manufacturer_len    = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = {
    .adv_int_min         = 0x20,
    .adv_int_max         = 0x40,
    .adv_type            = ADV_TYPE_IND,
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
    .channel_map         = ADV_CHNL_ALL,
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;        //콜백함수 가르킴
    uint16_t gatts_if;              //GATT Server 인터페이스
    uint16_t app_id;                //GATT Server 어플리케이션 식별자
    uint16_t conn_id;               //연결 id
    uint16_t service_handle;        //서비스 핸들
    esp_gatt_srvc_id_t service_id;  //서비스 id를 나타내는 구조체 타입
    uint16_t char_handle;           //특성 핸들
    esp_bt_uuid_t char_uuid;        //특성 uuid
    esp_gatt_perm_t perm;           //권한을 나타내는 열거형 변수
    esp_gatt_char_prop_t property;  //특성의 속성
    uint16_t descr_handle;          //descriptor 핸들
    esp_bt_uuid_t descr_uuid;       //descriptor uuid
};

// prototype이지만 edu_kit_profile_tab variable 이전에 선언 필요
static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
					esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst edu_kit_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/* Service */
static const uint16_t GATTS_SERVICE_UUID_MAIN               = 0x00FF;
static const uint16_t GATTS_CHAR_UUID_IMU_DATA_TX           = 0xFF01; //imu 센서데이터
static const uint16_t GATTS_CHAR_UUID_SPEECH_DATA_TX        = 0xFF10; //음성 센서데이터
static const uint16_t GATTS_CHAR_UUID_VISION_DATA_TX        = 0xFF19; //이미지 센서데이터
static const uint16_t GATTS_CHAR_UUID_INFERENCE_TX          = 0xFF04; //추론 결과
static const uint16_t GATTS_CHAR_UUID_RECORDING_RX          = 0xFF11; //웹 recording버튼, 음성, 비전 데이터 전송
static const uint16_t GATTS_CHAR_UUID_TYPE_SIGNAL_RX        = 0xFF12; //웹에서 오는 여러 시그널
static const uint16_t GATTS_CHAR_UUID_BEGIN_RECORDING_TX    = 0xFF13; //키트 버튼을 눌러 웹으로 데이터 캡쳐 시그널 전송
static const uint16_t GATTS_CHAR_UUID_TRANSFER_PERMISSION_TX= 0xFF24; // 모델 전송 허가 ACK 메세지

static const uint16_t GATTS_CHAR_UUID_HAS_MODEL_TX          = 0xFF0D; //모델 존재, 유효성 검사 결과 전송

//모델 전송 관련 chracteristic
static const uint16_t GATTS_CHAR_UUID_FILE_BLOCK            = 0x00F1;
static const uint16_t GATTS_CHAR_UUID_FILE_LENGTH           = 0x00F2;
static const uint16_t GATTS_CHAR_UUID_FILE_MAXIMUM_LENGTH   = 0x00F3;
static const uint16_t GATTS_CHAR_UUID_FILE_CHECKSUM         = 0x00F4;
static const uint16_t GATTS_CHAR_UUID_COMMAND               = 0x00F5;
static const uint16_t GATTS_CHAR_UUID_TRANSFER_STATUS       = 0x00F6;
static const uint16_t GATTS_CHAR_UUID_ERROR_MESSAGE         = 0x00F7;

static const uint16_t GATTS_CHAR_UUID_MODEL_TYPE            = 0xFF20;
static const uint16_t GATTS_CHAR_UUID_SENSOR_TYPE           = 0xFF21;
static const uint16_t GATTS_CHAR_UUID_NUM_CLASSES           = 0xFF05;
static const uint16_t GATTS_CHAR_UUID_CLASSES_NAMES_RX      = 0xFF22;
static const uint16_t GATTS_CHAR_UUID_CLASSES_NAMES_TX      = 0xFF25;

static const uint16_t GATTS_CHAR_UUID_CAPTURE_DELAY         = 0xFF07;
static const uint16_t GATTS_CHAR_UUID_THRESHOLD             = 0xFF08;




static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
// static const uint8_t char_prop_read                =  ESP_GATT_CHAR_PROP_BIT_READ;
// static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

/* Characteristic related variables */
// 초기 값
static const uint8_t init_value[1]                 = {0};
// 최대 크기
static const int32_t maximum_value[1]              = {MODEL_MAX_LENGTH};
// true value
static const uint32_t true_value[1] = {1};      

/* model transfer related variables */
const int32_t file_block_byte_count = 480;    // 전송 받을 파일 블록의 바이트 사이즈
//uint8_t file_buffers[70*1024];                // 전송받은 파일 저장을 위한 공간
extern uint8_t model_data[MODEL_MAX_LENGTH];
uint8_t* finished_file_buffer = NULL;         // 가장 최근 전송이 끝난 파일(모델)의 버퍼
int32_t finished_file_buffer_byte_count = 0;  // 전송받은 파일의 길이

uint8_t* in_progress_file_buffer = NULL;      // 전송 진행 중의 파일 버퍼 위치
int32_t in_progress_bytes_received = 0;       // 전송 진행 중 받은 파일의 바이트
int32_t in_progress_bytes_expected = 0;       // 전송 받을 파일의 총 바이트 크기
uint32_t in_progress_checksum = 0;            // 파일의 checksum (웹에서 받은 정보)
uint16_t file_block_length = 0;               // 전송받는 파일 블록의 길이 (웹에서 받은 정보)
uint8_t* file_block_value = NULL;             // 전송받은 파일 블록의 값들
uint32_t file_length_value;                   // 전송받는 파일의 총 길이 (웹에서 받은 정보)

const uint8_t fileTransferType = 1;           // 전송 타입을 결정
uint8_t *newModelFileData = NULL;             // 새롭게 업로드된 파일(모델)의 데이터
int newModelFileLength = 0;                   // 새롭게 업로드된 파일(모델)의 길이


/* info variables */
uint8_t state = 0;                            // 현재 state를 나타내는 변수
uint8_t type_signal = 0;                     // 현재 어떤 프로젝트와 연결되었는지 확인하는 signal 변수
uint8_t recording_speech = 0;                 // Speech 예제에서 inference 시작 버튼
uint8_t num_label = 0;                        // 현재 탑재된 모델의 라벨 갯수

/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t gatt_db[GEK_IDX_NB] =
{
    [IDX_SVC]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_MAIN), (uint8_t *)&GATTS_SERVICE_UUID_MAIN}},

    [IDX_CHAR_imuDataTx]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [IDX_CHAR_VAL_imuDataTx] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_IMU_DATA_TX, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(init_value), (uint8_t *)init_value}},
    
    [IDX_CHAR_CFG_imuDataTx]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint8_t)*6, sizeof(init_value), (uint8_t *)init_value}},

    [IDX_CHAR_visionDataTx]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [IDX_CHAR_VAL_visionDataTx] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_VISION_DATA_TX, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(init_value), (uint8_t *)init_value}},
    
    [IDX_CHAR_CFG_visionDataTx]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint8_t)*6, sizeof(init_value), (uint8_t *)init_value}},
    
    [IDX_CHAR_inferenceTx]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [IDX_CHAR_VAL_inferenceTx]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_INFERENCE_TX, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(init_value), (uint8_t *)init_value}},
    
    [IDX_CHAR_CFG_inferenceTx]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(init_value), (uint8_t *)init_value}},
    
    [IDX_CHAR_numClassesRx]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [IDX_CHAR_VAL_numClassesRx]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_NUM_CLASSES, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(init_value), (uint8_t *)init_value}},
    
    [IDX_CHAR_captureDelayRx]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [IDX_CHAR_VAL_captureDelayRx]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_CAPTURE_DELAY, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(init_value), (uint8_t *)init_value}},
    
    [IDX_CHAR_thresholdRx]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [IDX_CHAR_VAL_thresholdRx]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_THRESHOLD, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(init_value), (uint8_t *)init_value}},
    
    [IDX_CHAR_hasModelTx]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [IDX_CHAR_VAL_hasModelTx]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_HAS_MODEL_TX, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(init_value), (uint8_t *)init_value}},
    
    [IDX_CHAR_CFG_hasModelTx]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(init_value), (uint8_t *)init_value}},
    
    [IDX_CHAR_fileBlock]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [IDX_CHAR_VAL_fileBlock]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_FILE_BLOCK, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(init_value), (uint8_t *)init_value}},
    
    [IDX_CHAR_fileLength]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [IDX_CHAR_VAL_fileLength]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_FILE_LENGTH, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(init_value), (uint8_t *)init_value}},
    
    [IDX_CHAR_fileMaximumLength]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [IDX_CHAR_VAL_fileMaximumLength]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_FILE_MAXIMUM_LENGTH, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(maximum_value), (uint8_t *)maximum_value}},
    
    [IDX_CHAR_fileChecksum]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [IDX_CHAR_VAL_fileChecksum]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_FILE_CHECKSUM, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(init_value), (uint8_t *)init_value}},
    
    [IDX_CHAR_command]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [IDX_CHAR_VAL_command]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_COMMAND, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(init_value), (uint8_t *)init_value}},
    
    [IDX_CHAR_transferStatus]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [IDX_CHAR_VAL_transferStatus]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TRANSFER_STATUS, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(init_value), (uint8_t *)init_value}},
    
    [IDX_CHAR_CFG_transferStatus]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(init_value), (uint8_t *)init_value}},
    
    [IDX_CHAR_errorMessage]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [IDX_CHAR_VAL_errorMessage]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_ERROR_MESSAGE, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(init_value), (uint8_t *)init_value}},
    
    [IDX_CHAR_CFG_errorMessage]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(init_value), (uint8_t *)init_value}},
    
    // audioData(speechData)를 전송할 characteristic 생성
    [IDX_CHAR_speechDataTx]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [IDX_CHAR_VAL_speechDataTx] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_SPEECH_DATA_TX, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(init_value), (uint8_t *)init_value}},
    
    [IDX_CHAR_CFG_speechDataTx]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint8_t)*5, sizeof(init_value), (uint8_t *)init_value}},
    
    // recording을 원하는지 전달 받을 characteristic 생성
    [IDX_CHAR_recordingRx]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [IDX_CHAR_VAL_recordingRx] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_RECORDING_RX, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(init_value), (uint8_t *)init_value}},

    [IDX_CHAR_typeSignal]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [IDX_CHAR_VAL_typeSignal] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TYPE_SIGNAL_RX, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(init_value), (uint8_t *)init_value}},

    [IDX_CHAR_beginRecordingTx]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [IDX_CHAR_VAL_beginRecordingTx] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_BEGIN_RECORDING_TX, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(init_value), (uint8_t *)init_value}},
    
    [IDX_CHAR_CFG_beginRecordingTx]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(init_value), (uint8_t *)init_value}},
    
    [IDX_CHAR_transferPermissionTx]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [IDX_CHAR_VAL_transferPermissionTx] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TRANSFER_PERMISSION_TX, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(init_value), (uint8_t *)init_value}},
    
    [IDX_CHAR_CFG_transferPermissionTx]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(init_value), (uint8_t *)init_value}},

    [IDX_CHAR_modelTypeRx] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [IDX_CHAR_VAL_modelTypeRx] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_MODEL_TYPE, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(init_value), (uint8_t *)init_value}},

    [IDX_CHAR_sensorTypeRx] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [IDX_CHAR_VAL_sensorTypeRx] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_SENSOR_TYPE, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(init_value), (uint8_t *)init_value}},

    [IDX_CHAR_classesNamesRx] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [IDX_CHAR_VAL_classesNamesRx] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_CLASSES_NAMES_RX, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(init_value), (uint8_t *)init_value}},

    [IDX_CHAR_classesNamesTx] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [IDX_CHAR_VAL_classesNamesTx]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_CLASSES_NAMES_TX, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(init_value), (uint8_t *)init_value}},
    
    [IDX_CHAR_CFG_classesNamesTx]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(init_value), (uint8_t *)init_value}},
    
    [IDX_CHAR_hasModelTx]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
};
//=========================== prototypes ==========================
/* model transfer, upload 관련 */
// model 세팅을 마무리하고 hasModel을 BLE로 알리고 state값을 갱신
void _completeModelUpload(void);
// BLE를 통해 전송받은 파일로 새로운 모델 지정
void _onBLEFileReceived(uint8_t *file_data, int8_t file_length);
// BLE 파일 전송도중 생긴 에러를 알림
void _notifyError(const char* error_message);
// BLE 파일 전송이 성공했음을 알림
void _notifySuccess(void);
// BLE 파일이 전송 중임을 알림
void _notifyInProgress(void);
// 파일 전송(정확히는 받는 것)을 시작
void _startFileTransfer(void);
// 파일 전송 받는 것을 멈춤, 취소
void _cancelFileTransfer(void);
// 바이트단위로 crc
int32_t _crc32_for_byte(uint32_t r);
// 전체 파일에 대한 crc
uint32_t _crc32(const uint8_t* data, size_t data_length);
// 전송받은 파일의 checksum을 계산하여 오류제어
void _onFileTransferComplete(void);
// 전송받은 블럭단위의 파일을 in_progress_file_buffer에 합치며 저장
void _onFileBlockWritten(void);
// 전송받은 command 값을 통해 파일 전송을 시작하거나 멈추는 것을 결정
void _onCommandWritten(uint8_t command_value);

//웹에서 전송받은 type_signal에 맞는 처리를 수행하는 함수
void _process_type_sinal(uint8_t type_signal);


/* BLE event_handler */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

//=========================== public ==============================
void ble_begin(void)
{
    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(BLE_COMMUNICATION_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(BLE_COMMUNICATION_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(BLE_COMMUNICATION_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(BLE_COMMUNICATION_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(BLE_COMMUNICATION_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(BLE_COMMUNICATION_TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(ESP_APP_ID);
    if (ret){
        ESP_LOGE(BLE_COMMUNICATION_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(BLE_COMMUNICATION_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    transit_ble_state(BLE_STATE_DISCONNECT);
}

void send_data_to_ble(void* data, size_t size, ble_data_t data_type)
{
  if (get_ble_state() == BLE_STATE_CONNECT)
  {
    switch (data_type)
    {
      case IMU_DATA:
        characteristic_handle = edu_kit_handle_table[IDX_CHAR_VAL_imuDataTx];
        break;
      
      case SPEECH_DATA:
        characteristic_handle = edu_kit_handle_table[IDX_CHAR_VAL_speechDataTx];
        break;

      case VISION_DATA:
        characteristic_handle = edu_kit_handle_table[IDX_CHAR_VAL_visionDataTx];
        break;
      
      case RECORDING_SIGNAL_DATA:
        characteristic_handle = edu_kit_handle_table[IDX_CHAR_VAL_beginRecordingTx];
        break;
      
      case INFERENCE_DATA:
        characteristic_handle = edu_kit_handle_table[IDX_CHAR_VAL_inferenceTx];
        break;
      
      case TRANSFER_SIGNAL_DATA:
        characteristic_handle = edu_kit_handle_table[IDX_CHAR_VAL_transferPermissionTx];
        break;

      case CLASSES_NAMES_DATA:
        characteristic_handle = edu_kit_handle_table[IDX_CHAR_VAL_classesNamesTx];
        break;

      case HAS_MODEL_DATA:
        characteristic_handle = edu_kit_handle_table[IDX_CHAR_VAL_hasModelTx];
        break;

      default:
        ESP_LOGE(BLE_COMMUNICATION_TAG, "Error: Send error data type");
        break;
    }
    esp_err_t send_ble_ret = esp_ble_gatts_send_indicate(GATTS_IF_NUM, PROFILE_APP_IDX, characteristic_handle,
                                              size, data, false);
    if (send_ble_ret)
    {
      ESP_LOGE(BLE_COMMUNICATION_TAG, "send ble error, error code = %s", esp_err_to_name(send_ble_ret));
    }
  }
  else
  {
    ESP_LOGE(BLE_COMMUNICATION_TAG, "Frist, Connect before send data.");
  }
}

bool write_nvs_transferred_model()
{
  if (write_model_nvs(newModelFileData, file_length_value)) {
    ESP_LOGI(BLE_COMMUNICATION_TAG, "Write model to NVS");
  }
  else {
    ESP_LOGE(BLE_COMMUNICATION_TAG, "Fail to write a model to NVS");
    return false;
  }
  return true;
}


//=========================== tasks ===============================




//=========================== event handler ===============================
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    #ifdef CONFIG_SET_RAW_ADV_DATA
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
    #else
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
    #endif
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            /* advertising start complete event to indicate advertising start successfully or failed */
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(BLE_COMMUNICATION_TAG, "advertising start failed");
            }else{
                ESP_LOGI(BLE_COMMUNICATION_TAG, "advertising start successfully");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(BLE_COMMUNICATION_TAG, "Advertising stop failed");
            }
            else {
                ESP_LOGI(BLE_COMMUNICATION_TAG, "Stop adv successfully\n");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(BLE_COMMUNICATION_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:{
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
            if (set_dev_name_ret){
                ESP_LOGE(BLE_COMMUNICATION_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }
    #ifdef CONFIG_SET_RAW_ADV_DATA
            esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
            if (raw_adv_ret){
                ESP_LOGE(BLE_COMMUNICATION_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
            if (raw_scan_ret){
                ESP_LOGE(BLE_COMMUNICATION_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
    #else
            //config adv data
            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret){
                ESP_LOGE(BLE_COMMUNICATION_TAG, "config adv data failed, error code = %x", ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            //config scan response data
            ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
            if (ret){
                ESP_LOGE(BLE_COMMUNICATION_TAG, "config scan response data failed, error code = %x", ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
    #endif
            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, GEK_IDX_NB, SVC_INST_ID);
            if (create_attr_ret){
                ESP_LOGE(BLE_COMMUNICATION_TAG, "create attr table failed, error code = %x", create_attr_ret);
            }
        }
       	    break;
        case ESP_GATTS_READ_EVT:
       	    break;
        case ESP_GATTS_WRITE_EVT:
            if (!param->write.is_prep){
                if (edu_kit_handle_table[IDX_CHAR_VAL_recordingRx] == param->write.handle) {
                  ESP_LOGI(BLE_COMMUNICATION_TAG, "ESP_GATTS_WRITE_EVT,STATE handle = %d, value: %u", param->write.handle, *param->write.value);
                  esp_log_buffer_hex(BLE_COMMUNICATION_TAG, param->write.value, param->write.len);
                  if(*param->write.value == 1)
                  {
                    kit_signaling(START_SEND_DATA);
                  }
                }
                else if (edu_kit_handle_table[IDX_CHAR_VAL_fileBlock] == param->write.handle) {
                  /* fileBlock Handler, 
                     file block을 전송받고 이를 통해 file_block_length, file_block_value값을 최신화 한뒤
                     onFileBlockWritten 함수를 통해 파일 합치기
                  */
                  ESP_LOGI(BLE_COMMUNICATION_TAG, "ESP_GATTS_WRITE_EVT,STATE handle = %d, fileBlock", param->write.handle);
                  file_block_length = param->write.len;
                  ESP_LOGI(BLE_COMMUNICATION_TAG, "fileBlock Length: (%" PRIu16 ") ", param->write.len);
                  file_block_value = param->write.value;
                  _onFileBlockWritten();
                }
                else if (edu_kit_handle_table[IDX_CHAR_VAL_command] == param->write.handle) {
                  // command Handler, 전달받은 command를 인자로값으로 하여 onCommandWritten 함수 호출
                  ESP_LOGI(BLE_COMMUNICATION_TAG, "ESP_GATTS_WRITE_EVT,STATE handle = %d, Command", param->write.handle);
                  _onCommandWritten(*param->write.value);
                }
                else if (edu_kit_handle_table[IDX_CHAR_VAL_fileLength] == param->write.handle) {
                  // fileLength Handler, 전달 받은 값(파일의 총 길이)을 형변환을 통해 file_length_value에 저장
                  ESP_LOGI(BLE_COMMUNICATION_TAG, "ESP_GATTS_WRITE_EVT,STATE handle = %d, fileLength, value:", param->write.handle);
                  esp_log_buffer_hex(BLE_COMMUNICATION_TAG, param->write.value, param->write.len);
                  uint32_t temp = 0;
                  temp |= param->write.value[0];
                  temp |= (uint32_t)param->write.value[1] << 8;
                  temp |= (uint32_t)param->write.value[2] << 16;
                  temp |= (uint32_t)param->write.value[3] << 24;
                  file_length_value = temp;
                  set_model_length(file_length_value);
                  ESP_LOGI(BLE_COMMUNICATION_TAG, "file Length: (%" PRId32 ") ", file_length_value);
                }
                else if (edu_kit_handle_table[IDX_CHAR_VAL_fileChecksum] == param->write.handle) {
                  // fileChecksum Handler, 전달받은 값(file checksum)을 형변환을 통해 in_progress_checksum에 저장
                  ESP_LOGI(BLE_COMMUNICATION_TAG, "ESP_GATTS_WRITE_EVT,STATE handle = %d, fileChecksum, value:", param->write.handle);
                  esp_log_buffer_hex(BLE_COMMUNICATION_TAG, param->write.value, param->write.len);
                  in_progress_checksum = 0;
                  in_progress_checksum |= param->write.value[0];
                  in_progress_checksum |= (uint32_t)param->write.value[1] << 8;
                  in_progress_checksum |= (uint32_t)param->write.value[2] << 16;
                  in_progress_checksum |= (uint32_t)param->write.value[3] << 24;
                  ESP_LOGI(BLE_COMMUNICATION_TAG, "Checksum: (%" PRIu32 ") ", in_progress_checksum);
                }
                else if (edu_kit_handle_table[IDX_CHAR_VAL_typeSignal] == param->write.handle) {
                  type_signal = *param->write.value;
                  _process_type_sinal(type_signal);
                }
                else if (edu_kit_handle_table[IDX_CHAR_VAL_modelTypeRx] == param->write.handle) {
                  ESP_LOGI(BLE_COMMUNICATION_TAG, "Model type: %d", *param->write.value);
                  set_model_type(param->write.value);
                }
                else if (edu_kit_handle_table[IDX_CHAR_VAL_sensorTypeRx] == param->write.handle) {
                  ESP_LOGI(BLE_COMMUNICATION_TAG, "Sensor type: %d", *param->write.value);
                  set_sensor_type(param->write.value);
                }
                else if (edu_kit_handle_table[IDX_CHAR_VAL_numClassesRx] == param->write.handle) {
                  ESP_LOGI(BLE_COMMUNICATION_TAG, "Num classes: %d", *param->write.value);
                  set_num_classes(param->write.value);
                } 
                else if (edu_kit_handle_table[IDX_CHAR_VAL_classesNamesRx] == param->write.handle) {
                  ESP_LOGI(BLE_COMMUNICATION_TAG, "Classes names: %d", *param->write.value);
                  set_classes_names(param);
                } 
                else if (edu_kit_handle_table[IDX_CHAR_VAL_thresholdRx] == param->write.handle) {
                  set_threshold(param);
                }
                else if (edu_kit_handle_table[IDX_CHAR_VAL_captureDelayRx] == param->write.handle) {
                  ESP_LOGI(BLE_COMMUNICATION_TAG, "Capture delay: %d", *param->write.value);
                  set_capture_delay(param->write.value);
                } 
                //notification 사용을 위한 처리
                else if ((edu_kit_handle_table[IDX_CHAR_CFG_errorMessage] == param->write.handle || 
                edu_kit_handle_table[IDX_CHAR_CFG_hasModelTx] == param->write.handle || 
                edu_kit_handle_table[IDX_CHAR_CFG_inferenceTx] == param->write.handle ||
                edu_kit_handle_table[IDX_CHAR_CFG_beginRecordingTx] == param->write.handle ||
                edu_kit_handle_table[IDX_CHAR_CFG_transferPermissionTx] == param->write.handle ||
                edu_kit_handle_table[IDX_CHAR_CFG_classesNamesTx] == param->write.handle ||
                edu_kit_handle_table[IDX_CHAR_CFG_imuDataTx] == param->write.handle ||
                edu_kit_handle_table[IDX_CHAR_CFG_transferStatus] == param->write.handle ||
                edu_kit_handle_table[IDX_CHAR_CFG_speechDataTx] == param->write.handle  ||
                edu_kit_handle_table[IDX_CHAR_CFG_visionDataTx] == param->write.handle  ))
                {
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, param->write.handle,
                                            sizeof(init_value), init_value, false);
                }
                if (param->write.need_rsp){
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
      	    break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            ESP_LOGI(BLE_COMMUNICATION_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(BLE_COMMUNICATION_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_CONF_EVT:
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(BLE_COMMUNICATION_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(BLE_COMMUNICATION_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            esp_log_buffer_hex(BLE_COMMUNICATION_TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            //키트 ble 상태 변경
            transit_ble_state(BLE_STATE_CONNECT);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(BLE_COMMUNICATION_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
            //키트 ble 상태 변경
            transit_ble_state(BLE_STATE_DISCONNECT);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
            if (param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(BLE_COMMUNICATION_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != GEK_IDX_NB){
                ESP_LOGE(BLE_COMMUNICATION_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to GEK_IDX_NB(%d)", param->add_attr_tab.num_handle, GEK_IDX_NB);
            }
            else {
                ESP_LOGI(BLE_COMMUNICATION_TAG, "create attribute table successfully, the number handle = %d\n",param->add_attr_tab.num_handle);
                memcpy(edu_kit_handle_table, param->add_attr_tab.handles, sizeof(edu_kit_handle_table));
                esp_ble_gatts_start_service(edu_kit_handle_table[IDX_SVC]);
            }
            break;
        }
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_DELETE_EVT:
        default:
            break;
    }
    // ESP_LOGI(BLE_COMMUNICATION_TAG, "Event %d", event);
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            edu_kit_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGE(BLE_COMMUNICATION_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == edu_kit_profile_tab[idx].gatts_if) {
                if (edu_kit_profile_tab[idx].gatts_cb) {
                    edu_kit_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}




//=========================== private ==============================

void _process_type_sinal(uint8_t type_signal)
{
  // kit signal과 같게 맞춤.
  kit_signaling(type_signal + 1);
}

void _completeModelUpload(void)
{
  ESP_LOGI(BLE_TRANSFER_TAG, "void completeModelUpload called!");
  kit_signaling(COMPLETE_MODEL_UPLOAD);
}
void _onBLEFileReceived(uint8_t *file_data, int8_t file_length)
{
    ESP_LOGI(BLE_TRANSFER_TAG, "void onBLEFileReceived called!");
    switch (fileTransferType)
    {
        case 1:
          // Queue up the model swap
          newModelFileData = file_data;
          newModelFileLength = file_length;
          ESP_LOGI(BLE_TRANSFER_TAG, "fileLength: %d, %ld", file_length, file_length_value);
          break;
        default:
          ESP_LOGE(BLE_TRANSFER_TAG, "onBLEFileReceived Error");
          break;
  }
  _completeModelUpload();
}

void _notifyError(const char* error_message)
{
    ESP_LOGE(BLE_COMMUNICATION_TAG, "void notifyError called! error: %s", error_message);

    esp_err_t ret = esp_ble_gatts_send_indicate(GATTS_IF_NUM, PROFILE_APP_IDX, edu_kit_handle_table[IDX_CHAR_VAL_transferStatus],
                                        sizeof(true_value), true_value, false);
    if(ret) {
        ESP_LOGE(BLE_COMMUNICATION_TAG, "State No send!!!!!!!!!");
    }

    uint8_t error_message_buffer[128];
    bool at_string_end = false;
    for (int i = 0; i < 128; ++i) {
        const bool at_last_byte = (i == (128 - 1));
        if (!at_string_end && !at_last_byte) {
        const char current_char = error_message[i];
        if (current_char == 0) {
            at_string_end = true;
        } else {
            error_message_buffer[i] = current_char;
        }
        }

        if (at_string_end || at_last_byte) {
        error_message_buffer[i] = 0;
        }
    }
    ret = esp_ble_gatts_send_indicate(GATTS_IF_NUM, PROFILE_APP_IDX, edu_kit_handle_table[IDX_CHAR_VAL_errorMessage],
                                        sizeof(error_message_buffer), error_message_buffer, false);
    if(ret) {
        ESP_LOGE(BLE_COMMUNICATION_TAG, "Error Message No send!!!!!!!!!");
    }
}

void _notifySuccess(void)
{
  ESP_LOGI(BLE_COMMUNICATION_TAG,"void notifySuccess called!");
  const int32_t success_status_code[1] = {0};
  esp_err_t ret = esp_ble_gatts_send_indicate(GATTS_IF_NUM, PROFILE_APP_IDX, edu_kit_handle_table[IDX_CHAR_VAL_transferStatus],
                                      sizeof(success_status_code), success_status_code, false);
  if(ret) {
    ESP_LOGE(BLE_COMMUNICATION_TAG, "No send!!!!!!!!!");
  }   
}

void _notifyInProgress(void) 
{
  ESP_LOGI(BLE_COMMUNICATION_TAG,"void notifyInProgress called!");
  const int32_t in_progress_status_code[1] = {2};
  esp_err_t ret = esp_ble_gatts_send_indicate(GATTS_IF_NUM, PROFILE_APP_IDX, edu_kit_handle_table[IDX_CHAR_VAL_transferStatus],
                                    sizeof(in_progress_status_code), in_progress_status_code, false);
  if(ret) {
    ESP_LOGE(BLE_COMMUNICATION_TAG, "No send!!!!!!!!!");
  }                                  
}

void _startFileTransfer(void) 
{
  ESP_LOGI(BLE_COMMUNICATION_TAG,"void startFileTransfer called!");
  if (in_progress_file_buffer != NULL) {
    _notifyError("File transfer command received while previous transfer is still in progress");
    return;
  }
  ESP_LOGI(BLE_COMMUNICATION_TAG,"void startFileTransfer Step 1");
  const int32_t in_progress_status_code[1] = {2};
  esp_ble_gatts_send_indicate(GATTS_IF_NUM, PROFILE_APP_IDX, edu_kit_handle_table[IDX_CHAR_VAL_transferStatus],
                                    sizeof(in_progress_status_code), in_progress_status_code, false);

  if (file_length_value > maximum_value[0]) {
    _notifyError("File too large: Maximum is ");
    return;
  }
  ESP_LOGI(BLE_COMMUNICATION_TAG,"void startFileTransfer Step 2");
  
  in_progress_file_buffer = &model_data[0];
  ESP_LOGI(BLE_COMMUNICATION_TAG,"void startFileTransfer Step 3 : (%p) ", in_progress_file_buffer);
  in_progress_bytes_received = 0;
  ESP_LOGI(BLE_COMMUNICATION_TAG,"void startFileTransfer Step 4");
  in_progress_bytes_expected = file_length_value;
  ESP_LOGI(BLE_COMMUNICATION_TAG,"void startFileTransfer Step 5");

  _notifyInProgress();
}

void _cancelFileTransfer(void) 
{
  ESP_LOGI(BLE_COMMUNICATION_TAG,"void cancelFileTransfer called!");
  if (in_progress_file_buffer != NULL) {
    _notifyError("File transfer cancelled");
    in_progress_file_buffer = NULL;
  }
}

int32_t _crc32_for_byte(uint32_t r) 
{
  ESP_LOGI(BLE_COMMUNICATION_TAG, "int32_t crc32_for_byte called!");
  for (int j = 0; j < 8; ++j) {
    r = (r & 1? 0: (uint32_t)0xedb88320L) ^ r >> 1;
  }
  return r ^ (uint32_t)0xff000000L;
}

uint32_t _crc32(const uint8_t* data, size_t data_length) 
{
  ESP_LOGI(BLE_COMMUNICATION_TAG, "uint32_t crc32 called!");
  static uint32_t table[256];
  static bool is_table_initialized = false;
  if (!is_table_initialized) {
    for(size_t i = 0; i < 256; ++i) {
      table[i] = _crc32_for_byte(i);
    }
    is_table_initialized = true;
  }
  uint32_t crc = 0;
  for (size_t i = 0; i < data_length; ++i) {
    const uint8_t crc_low_byte = (uint8_t)crc;
    const uint8_t data_byte = data[i];
    const uint8_t table_index = crc_low_byte ^ data_byte;
    crc = table[table_index] ^ (crc >> 8);
  }
  return crc;
}

void _onFileTransferComplete(void) 
{
  ESP_LOGI(BLE_COMMUNICATION_TAG, "void onFileTransferComplete() called!");
  uint32_t computed_checksum = _crc32(in_progress_file_buffer, in_progress_bytes_expected);
  ESP_LOGI(BLE_COMMUNICATION_TAG, "in_progress_checksum: (%" PRIu32 "),     computed_checksum: (%" PRIu32 ")", in_progress_checksum, computed_checksum);
  for(int i=0;i<30;i++) {
    ESP_LOGI(BLE_COMMUNICATION_TAG, "Buffer: %02x", model_data[i]);
  }
  if (in_progress_checksum != computed_checksum) {
    _notifyError("File transfer failed: Expected checksum 0x");
    in_progress_file_buffer = NULL;
    return;
  }

  finished_file_buffer = &model_data[0];
  finished_file_buffer_byte_count = in_progress_bytes_expected;

  in_progress_file_buffer = NULL;
  in_progress_bytes_received = 0;
  in_progress_bytes_expected = 0;

  _notifySuccess();

  _onBLEFileReceived(finished_file_buffer, finished_file_buffer_byte_count);
}

void _onFileBlockWritten(void)
{
  ESP_LOGI(BLE_COMMUNICATION_TAG,"onFileBlockWritten Handler called!");

  if (in_progress_file_buffer == NULL) {
    ESP_LOGE(BLE_COMMUNICATION_TAG,"File block sent while no valid command is active");
    _notifyError("File block sent while no valid command is active");
    return;
  }
  
  if (file_block_length > file_block_byte_count) {
    ESP_LOGE(BLE_COMMUNICATION_TAG,"Too many bytes in block: Expected (%" PRId32 "),  but received (%" PRIu16 ")", file_block_byte_count, file_block_length);
    _notifyError("Too many bytes in block: Expected ");
    in_progress_file_buffer = NULL;
    return;
  }
  
  const int32_t bytes_received_after_block = in_progress_bytes_received + file_block_length;
  if ((bytes_received_after_block > in_progress_bytes_expected) ||
    (bytes_received_after_block > maximum_value[0])) {
    ESP_LOGE(BLE_COMMUNICATION_TAG,"Too many bytes: Expected (%" PRId32 "),  but received (%" PRId32 ")", in_progress_bytes_expected, bytes_received_after_block);
    _notifyError("Too many bytes: Expected ");
    in_progress_file_buffer = NULL;
    return;
  }
  
  memcpy(&(in_progress_file_buffer[in_progress_bytes_received]), file_block_value, file_block_length);
  

  if (bytes_received_after_block == in_progress_bytes_expected) {
    _onFileTransferComplete();
  } else {
    ESP_LOGI(BLE_COMMUNICATION_TAG,"bytes_received_after_block Value: (%" PRId32 ")", bytes_received_after_block);
    in_progress_bytes_received = bytes_received_after_block;
  }
}

void _onCommandWritten(uint8_t command_value) {
  ESP_LOGI(BLE_COMMUNICATION_TAG,"void onCommandWritten Handler called!, Command Value: %d",command_value);
  if ((command_value != 1) && (command_value != 2)) {
    ESP_LOGE(BLE_COMMUNICATION_TAG,"Bad command value: Expected 1 or 2 but received %d",command_value);
    _notifyError("Bad command value: Expected 1 or 2 but received ");
    return;
  }
  if (command_value == 1) {
    _startFileTransfer();
  } else if (command_value == 2) {
    _cancelFileTransfer();
  }
}