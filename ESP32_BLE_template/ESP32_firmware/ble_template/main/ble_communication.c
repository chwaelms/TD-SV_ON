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

#include <esp_heap_caps.h>

//=========================== variables ===========================
// BLE advertising config done 여부확인
static uint8_t adv_config_done = 0;     // 내부 변수

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
        0x0f, 0x09, 'B', 'L', 'E', '_', 'T', 'E', 'S', 'T', '_', 'K', 'I', 'T'
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
    0x42, 0x4C, 0x45, 0x5F, 0x54, 0x45, 0x53, 0x54, 0x5F, 0x4B, 0x49, 0x54,
};

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval        = 0x0010, //slave connection max interval, Time = max_interval * 1.25 
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
static const uint16_t GATTS_SERVICE_UUID_MAIN               = 0x00FF;   // service uuid
static const uint16_t GATTS_CHAR_UUID_DATA_TX               = 0xFF01;   // 데이터 전송 characteristic uuid

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
// static const uint8_t char_prop_read                =  ESP_GATT_CHAR_PROP_BIT_READ;
// static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

/* Characteristic related variables */
static const uint8_t init_value[1] = {0};   // 초기화 데이터

/* BLE connect check value */
bool is_ble_connect = false;                // 내부 변수 (연결 상태, 외부에서 접근 가능)

/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t gatt_db[GEK_IDX_NB] =
{
    [IDX_SVC]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_MAIN), (uint8_t *)&GATTS_SERVICE_UUID_MAIN}},

    [IDX_CHAR_dataTx]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [IDX_CHAR_VAL_dataTx] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_DATA_TX, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(init_value), (uint8_t *)init_value}},
    
    [IDX_CHAR_CFG_dataTx]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint8_t)*6, sizeof(init_value), (uint8_t *)init_value}},
};

//=========================== prototypes ==========================
/* BLE event_handler */
// 내부 함수 프로토타입 (static=내부용)
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

//=========================== public ==============================
// 공개 함수 구현
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
}

void send_data_to_ble(void* data, size_t size)
{   
    // BLE 데이터 전송은 특정 characteristic을 통해 전송합니다. 데이터 구분을 하기위해서는 characteristic을 추가하는 것을 추천 합니다.
    characteristic_handle = edu_kit_handle_table[IDX_CHAR_VAL_dataTx];
    esp_err_t send_ble_ret = esp_ble_gatts_send_indicate(GATTS_IF_NUM, PROFILE_APP_IDX, characteristic_handle,
                                                size, data, false);
    if (send_ble_ret)
    {
        ESP_LOGE(BLE_COMMUNICATION_TAG, "send ble error, error code = %s", esp_err_to_name(send_ble_ret));
    }
}

bool check_ble_connect(void) {
    return is_ble_connect;
}

//=========================== tasks ===============================


//=========================== event handler ===============================
// 내부 함수 구현
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
            // 연결 상태 확인 변수 업데이트 
            is_ble_connect = true;
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(BLE_COMMUNICATION_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
            // 연결 상태 확인 변수 업데이트 
            is_ble_connect = false;
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
