//=========================== header ==========================
#include "state_controller.h"
#include "app_button.h"
#include "imu_provider.h"
#include "speech_provider.h"
#include "ble_communication.h"
#include "dummy_test.h"
#include "send_data.h"
#include "model_manager.h"
#include "vision_provider.h"
#include "lcd_manager.h"

#include "esp_log.h"
#include "esp_event.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>

#include "icm42670.h"
//=========================== variables ===========================
kit_state_t kit_state = KIT_STATE_IDLE;
ble_state_t ble_state = BLE_STATE_IDLE;

// 각 Task의 Handler들, Task 실행중이 아닐때는 NULL
TaskHandle_t xTaskHandles[IDX_TASK_COUNT_NB];

// ble로 보낼 긍정 데이터 
uint8_t positive_data[1] = {1};

// ble로 보낼 부정 데이터 
uint8_t negative_data[1] = {0};

// task 삭제를 위한 시그널 값(i2c 자원을 반납하고 테스크를 삭제하기 위해)
uint8_t delete_signal = 1;

// task 삭제를 위한 queue
QueueHandle_t xQueueDeleteSignal = NULL;

extern bool test11;

//=========================== prototypes ==========================
void _handler_A(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data);
void _handler_B(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data);
void _handler_X(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data);
void _handler_Y(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data);
void _init_button_handlers();

// 현재 실행중인 tasks 모두 삭제(특정 삭제하면 안되는 task들만 제외)
void _delete_tasks();

//=========================== public ==============================
void kit_signaling(kit_signal_t t)
{
    switch (t) 
    {
        case SET_STATE_IDLE:
            if (get_kit_state() != KIT_STATE_IDLE)
            {
                _delete_tasks();
                transit_kit_state(KIT_STATE_IDLE);
            }
            break;
        case SET_STATE_TRAIN_MOTION:
            if (get_kit_state() != KIT_STATE_TRAIN_MOTION)
            {
                _delete_tasks();
                //imu Task에 적절한 queue 연결
                set_imu_queue(get_send_queue());
                set_imu_delete_signal_queue(xQueueDeleteSignal);
                transit_kit_state(KIT_STATE_TRAIN_MOTION);
            }
            break;
        
        case SET_STATE_TRAIN_SPEECH:
            if (get_kit_state() != KIT_STATE_TRAIN_SPEECH)
            {
                _delete_tasks();
                //speech Task에 적절한 queue 연결
                set_speech_queue(get_send_queue());
                transit_kit_state(KIT_STATE_TRAIN_SPEECH);
            }
            break;
        
        case SET_STATE_TRAIN_VISION:
            if (get_kit_state() != KIT_STATE_TRAIN_VISION)
            {
                _delete_tasks();
                //VISION Task에 적절한 queue 연결
                set_camera_frame_out_queue(get_lcd_frame_in_queue());
                transit_kit_state(KIT_STATE_TRAIN_VISION);
            }
            break;

        case SET_STATE_UPLOAD_MOTION:
            _delete_tasks();
            transit_kit_state(KIT_STATE_UPLOAD_MOTION);
            break;
        
        case SET_STATE_UPLOAD_SPEECH:
            _delete_tasks();
            transit_kit_state(KIT_STATE_UPLOAD_SPEECH);
            break;
        
        case SET_STATE_UPLOAD_VISION:
            _delete_tasks();
            transit_kit_state(KIT_STATE_UPLOAD_VISION);
            break;

        case SET_STATE_INFERENCE_MOTION:
            if (get_kit_state() != KIT_STATE_INFERENCE_MOTION)
            {
                _delete_tasks();    // 이전 task들 삭제    
                model_setup();  // 모델 setup
                set_imu_queue(get_sensor_data_queue());
                set_imu_delete_signal_queue(xQueueDeleteSignal);
                transit_kit_state(KIT_STATE_INFERENCE_MOTION);
            }
            break;
        
        case SET_STATE_INFERENCE_SPEECH:
            if(get_kit_state() != KIT_STATE_INFERENCE_SPEECH)
            {
                _delete_tasks();    // 이전 task들 삭제
                model_setup();
                set_speech_queue(get_sensor_data_queue());
                transit_kit_state(KIT_STATE_INFERENCE_SPEECH);
            }
            break;
        
        case SET_STATE_INFERENCE_VISION:
            if(get_kit_state() != KIT_STATE_INFERENCE_VISION)
            {
                _delete_tasks();    // 이전 task들 삭제
                model_setup();
                //VISION Task에 적절한 queue 연결
                set_camera_frame_out_queue(get_lcd_frame_in_queue());
                transit_kit_state(KIT_STATE_INFERENCE_VISION);
            }
            break;

        case VALIDATE_MODEL_MOTION:
        case VALIDATE_MODEL_SPEECH:
        case VALIDATE_MODEL_VISION:
            if (check_validation_model_info())
            {
                // 라벨 갯수, 이름들 전송
                model_info_db_t model_info = read_model_info_nvs();
                //send_data_to_ble(model_info.classes_names, sizeof(model_info.classes_names), CLASSES_NAMES_DATA);
                //적절한 모델임을 웹으로 알림 (hasModel로)
                send_data_to_ble(positive_data, sizeof(positive_data), HAS_MODEL_DATA);
            }
            else {
                //부적합한 모델임을 웹으로 알림 (hasModel로)
                send_data_to_ble(negative_data, sizeof(negative_data), HAS_MODEL_DATA);
            }
            break;
        
        case COMPLETE_MODEL_UPLOAD:
            if(!write_nvs_transferred_model())
            {
                break;
            }
            switch (get_kit_state())
            {
                case KIT_STATE_UPLOAD_MOTION:
                    kit_signaling(SET_STATE_INFERENCE_MOTION);
                    break;

                case KIT_STATE_UPLOAD_SPEECH:
                    kit_signaling(SET_STATE_INFERENCE_SPEECH);
                    break;

                case KIT_STATE_UPLOAD_VISION:
                    kit_signaling(SET_STATE_INFERENCE_VISION);
                    break;
                default:
                    ESP_LOGE(STATE_CONTROLLER_TAG,"COMPLETE_MODEL_UPLOAD signaling error!");
                    break;
            }
            break;
        
        case START_SEND_DATA:
            if( get_kit_state() == KIT_STATE_TRAIN_SPEECH ||
                get_kit_state() == KIT_STATE_INFERENCE_SPEECH )
            {   
                // speech Task 생성
                xTaskCreatePinnedToCore(speech_provider_task, "speech_provider_task", 1024 * 3, NULL, 5, &(xTaskHandles[IDX_SPEECH_PROVIDER_TASK]), 0);
            }
            else if(get_kit_state() == KIT_STATE_TRAIN_VISION)
            {
                set_lcd_frame_out_queue(get_send_queue());
            }
            else if(get_kit_state() == KIT_STATE_INFERENCE_VISION)
            {
                set_lcd_frame_out_queue(get_sensor_data_queue());
            }
            break;

        case BUTTON_A_PRESSED:
            break;
        
        case BUTTON_B_PRESSED:
            break;
        
        case BUTTON_X_PRESSED:
            break;

        case BUTTON_Y_PRESSED:
            // 데이터 캡쳐후 BLE전송을 위한 Y버튼
            if(get_kit_state() == KIT_STATE_TRAIN_SPEECH)
            {
                // 먼저 recording 시작 시그널 전송
                send_data_to_ble(positive_data, sizeof(positive_data), RECORDING_SIGNAL_DATA);
            }
            else if(get_kit_state() == KIT_STATE_INFERENCE_SPEECH)
            {
                //speech 데이터 캡쳐 후 추론 테스크로 전송
                kit_signaling(START_SEND_DATA);
            }
            else if(get_kit_state() == KIT_STATE_TRAIN_VISION)
            {
                // 먼저 recording 시작 시그널 전송
                send_data_to_ble(positive_data, sizeof(positive_data), RECORDING_SIGNAL_DATA);
            }
            else if(get_kit_state() == KIT_STATE_INFERENCE_VISION)
            {
                kit_signaling(START_SEND_DATA);
            }
            break;

        default:
            ESP_LOGE(STATE_CONTROLLER_TAG,"signaling error! siganl: %d", t);
            break;
    }
}

void transit_kit_state(kit_state_t t)
{   
    ESP_LOGI(STATE_CONTROLLER_TAG,"kit state change: %d", t);
    switch (t) 
    {
        case KIT_STATE_IDLE:
            kit_state = KIT_STATE_IDLE;
            break;

        case KIT_STATE_TRAIN_MOTION:
            kit_state = KIT_STATE_TRAIN_MOTION;
            // 데이터를 전송하는 Task 생성(현재는 ble만 사용)
            xTaskCreatePinnedToCore(send_data_task, "send_data_task", 1024 * 3, NULL, 5, &(xTaskHandles[IDX_SEND_DATA_TASK]), 0);
            // imu Task 생성
            xTaskCreatePinnedToCore(imu_provider_task, "imu_provider_task", 1024 * 3, NULL, 5, &(xTaskHandles[IDX_IMU_PROVIDER_TASK]), 0);
            break;
        case KIT_STATE_TRAIN_SPEECH:
            kit_state = KIT_STATE_TRAIN_SPEECH;
            // 데이터를 전송하는 Task 생성(현재는 ble만 사용)
            xTaskCreatePinnedToCore(send_data_task, "send_data_task", 1024 * 3, NULL, 5, &(xTaskHandles[IDX_SEND_DATA_TASK]), 0);
            break;
        case KIT_STATE_TRAIN_VISION:
            kit_state = KIT_STATE_TRAIN_VISION;
            xTaskCreatePinnedToCore(send_data_task, "send_data_task", 1024 * 3, NULL, 5, &(xTaskHandles[IDX_SEND_DATA_TASK]), 0);
            // LCD task 생성
            xTaskCreatePinnedToCore(vision_display_task, "vision_display_task", 1024 * 3, NULL, 5, &(xTaskHandles[IDX_VISION_DISPLAY_TASK]), 0);
            // vision provider task 생성
            xTaskCreatePinnedToCore(vision_provider_task, "vision_provider_task", 1024 * 3, NULL, 5, &(xTaskHandles[IDX_VISION_PROVIDER_TASK]), 0);
            break;

        case KIT_STATE_INFERENCE_MOTION:
            kit_state = KIT_STATE_INFERENCE_MOTION;
            // 추론 테스크 생성
            xTaskCreatePinnedToCore(model_inference_task, "model_inference_task", 1024 * 3, NULL, 5, &(xTaskHandles[IDX_MODEL_INFERENCE_TASK]), 0);
            // imu Task 생성
            xTaskCreatePinnedToCore(imu_provider_task, "imu_provider_task", 1024 * 3, NULL, 5, &(xTaskHandles[IDX_IMU_PROVIDER_TASK]), 0);
            break;
        case KIT_STATE_INFERENCE_SPEECH:
            kit_state = KIT_STATE_INFERENCE_SPEECH;
            // 추론 테스크 생성
            xTaskCreatePinnedToCore(model_inference_task, "model_inference_task", 1024 * 3, NULL, 5, &(xTaskHandles[IDX_MODEL_INFERENCE_TASK]), 0);
            break;
        case KIT_STATE_INFERENCE_VISION:
            kit_state = KIT_STATE_INFERENCE_VISION;
            // 추론 테스크 생성
            xTaskCreatePinnedToCore(model_inference_task, "model_inference_task", 1024 * 3, NULL, 5, &(xTaskHandles[IDX_MODEL_INFERENCE_TASK]), 0);
            // LCD task 생성
            xTaskCreatePinnedToCore(vision_display_task, "vision_display_task", 1024 * 3, NULL, 5, &(xTaskHandles[IDX_VISION_DISPLAY_TASK]), 0);
            // vision provider task 생성
            xTaskCreatePinnedToCore(vision_provider_task, "vision_provider_task", 1024 * 3, NULL, 5, &(xTaskHandles[IDX_VISION_PROVIDER_TASK]), 0);
            break;

        case KIT_STATE_UPLOAD_MOTION:
            ESP_LOGI(STATE_CONTROLLER_TAG, "MOTION: Upload start!");
            kit_state = KIT_STATE_UPLOAD_MOTION;
            //모델 전송 시작을 알림
            send_data_to_ble(positive_data, sizeof(positive_data), TRANSFER_SIGNAL_DATA);
            break;
        case KIT_STATE_UPLOAD_SPEECH:
            ESP_LOGI(STATE_CONTROLLER_TAG, "SPEECH: Upload start!");
            kit_state = KIT_STATE_UPLOAD_SPEECH;
            //모델 전송 시작을 알림
            send_data_to_ble(positive_data, sizeof(positive_data), TRANSFER_SIGNAL_DATA);
            break;
        case KIT_STATE_UPLOAD_VISION:
            ESP_LOGI(STATE_CONTROLLER_TAG, "VISION: Upload start!");
            kit_state = KIT_STATE_UPLOAD_VISION;
            send_data_to_ble(positive_data, sizeof(positive_data), TRANSFER_SIGNAL_DATA);
            break;
        default:
            break;
    }
}

void transit_ble_state(ble_state_t t)
{
    switch (t) 
    {
        case BLE_STATE_IDLE:
            ble_state = BLE_STATE_IDLE;
            break;
        case BLE_STATE_DISCONNECT:
            ble_state = BLE_STATE_DISCONNECT;
            kit_signaling(SET_STATE_IDLE);
            break;
        case BLE_STATE_CONNECT:
            ble_state = BLE_STATE_CONNECT;
            break;
        default:
            break;
    }
}

kit_state_t get_kit_state(void)
{
    return kit_state;
}

ble_state_t get_ble_state(void)
{
    return ble_state;
}

//=========================== tasks ===============================
// 사실상 여기가 메인이라 생각하면 된다.
void state_controller_task(void * arg)
{   
    ESP_LOGI(STATE_CONTROLLER_TAG, "Start: state_controller_task");

    // task 핸들러 등록
    xTaskHandles[IDX_STATE_CONTROLLER_TASK] = xTaskGetCurrentTaskHandle();

    // 버튼 핸들러 등록, 초기화
    _init_button_handlers();

    // 현재 kit의 상태를 초기화
    transit_kit_state(KIT_STATE_IDLE);

    // 현재 ble 상태를 초기화
    transit_ble_state(BLE_STATE_IDLE);
    
    // ble 서비스 시작, 인벤트로 처리하여 따로 Task가 존재하지 않는 것으로 보임
    ble_begin();
    
    xQueueDeleteSignal = xQueueCreate(1,sizeof(uint8_t));

    while(1) {
        // 테스크 돌리기
        vTaskDelay(50);
    }
}


//=========================== private ==============================
void _handler_A(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data)
{
    ESP_LOGI("HDR", "A Pressed");
    kit_signaling(BUTTON_A_PRESSED);
}

void _handler_B(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data) 
{
    ESP_LOGI("HDR", "B Pressed");
    kit_signaling(BUTTON_B_PRESSED);
}

void _handler_X(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data) 
{
    ESP_LOGI("HDR", "X Pressed");
    kit_signaling(BUTTON_X_PRESSED);
}

void _handler_Y(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data) 
{
    ESP_LOGI("HDR", "Y Pressed");
    kit_signaling(BUTTON_Y_PRESSED);
}

void _init_button_handlers()
{
    // 버튼A 이벤트 핸들러 등록
    esp_event_handler_register(BUTTON_EVENTS, BUTTON_EVENT_A_PRESSED, _handler_A, 0);

    // 버튼B 이벤트 핸들러 등록
    esp_event_handler_register(BUTTON_EVENTS, BUTTON_EVENT_B_PRESSED, _handler_B, 0);
    
    // 버튼X 이벤트 핸들러 등록
    esp_event_handler_register(BUTTON_EVENTS, BUTTON_EVENT_X_PRESSED, _handler_X, 0);

    // 버튼Y 이벤트 핸들러 등록
    esp_event_handler_register(BUTTON_EVENTS, BUTTON_EVENT_Y_PRESSED, _handler_Y, 0);
}

void _delete_tasks()
{   
    //state_controller task, dummy task, speech provider task 제외하고 모두 삭제
    for(uint8_t i=3;i<IDX_TASK_COUNT_NB;i++)
    {
        if(xTaskHandles[i] != NULL)
        {
            if(i == IDX_IMU_PROVIDER_TASK)
            {
                xQueueSend(xQueueDeleteSignal,&delete_signal,portMAX_DELAY);
                xTaskHandles[i] = NULL;
                continue;
            }
            if(i == IDX_VISION_PROVIDER_TASK)
            {
                esp_err_t err = esp_camera_deinit();
                if (err != ESP_OK){
                    ESP_LOGE(STATE_CONTROLLER_TAG, "esp_camera_deinit Failed");
                }
                else {
                    ESP_LOGI(STATE_CONTROLLER_TAG, "esp_camera_deinit complete");
                }
            }
            
            vTaskDelete(xTaskHandles[i]);
            xTaskHandles[i] = NULL;

            ESP_LOGI(STATE_CONTROLLER_TAG, "delete Task num: %u", i);
        }
    }
}

