#include "app_main.h"
#include "app_button.h"

#include "speech_provider.h"
#include "model_manager.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>

#include "nvs_flash.h"
#include "esp_log.h"


//=========================== variables ===========================


//=========================== prototypes ==========================
void _init_nvs(void);
void _handler_Y(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data);

//=========================== public ==============================
void app_main() {
    esp_err_t ret;

    // init NVS
    _init_nvs();
    ESP_LOGI(APP_MAIN_TAG, "NVS Initialization Complete");

    // init Button
    esp_event_loop_create_default();
    init_button();
    ESP_LOGI(APP_MAIN_TAG, "Button Initialization Complete");

    //speech_data를 위한 MIC 초기화
    i2s_init();
    ESP_LOGI(APP_MAIN_TAG, "Mic Initialization Complete");
    
    //추론모델 초기화
    model_setup();

    // 테스크간 큐 연결
    set_speech_queue(get_sensor_data_queue());
    
    // 추론 테스크 생성
    xTaskCreatePinnedToCore(model_inference_task, "model_inference_task", 1024 * 3, NULL, 5, NULL, 0);
    
    // 버튼Y 이벤트 핸들러 등록
    esp_event_handler_register(BUTTON_EVENTS, BUTTON_EVENT_Y_PRESSED, _handler_Y, 0);
}

//=========================== private ==============================
void _init_nvs(void)
{
    /* Initialize NVS. */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void _handler_Y(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data) 
{
    ESP_LOGI("HDR", "Y Pressed");
    // 추론 테스크 생성
    xTaskCreatePinnedToCore(speech_provider_task, "speech_provider_task", 1024 * 3, NULL, 5, NULL, 0);
}