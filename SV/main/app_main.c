#include "app_main.h"
#include "ble_communication.h"
#include "app_button.h"
#include "imu_provider.h"
#include "state_controller.h"
#include "speech_provider.h"
#include "send_data.h"
#include "lcd_manager.h"
#include "vision_provider.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>

#include "nvs_flash.h"
#include "esp_log.h"

#include "icm42670.h"
//=========================== variables ===========================


//=========================== prototypes ==========================
void _init_nvs(void);
sv_config_t my_config;
sv_handle_t* sv_system = sv_system_init(&my_config);
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

    //send_data_task를 위한 초기화
    init_send_data();
    ESP_LOGI(APP_MAIN_TAG, "Send Data Initialization Complete");
    
    //speech_data를 위한 MIC 초기화
    i2s_init();
    ESP_LOGI(APP_MAIN_TAG, "Mic Initialization Complete");

    //lcd 출력을 위한 lcd 초기화
    init_lcd();
    ESP_LOGI(APP_MAIN_TAG, "LCD Initialization Complete");
    
    // state controll
    xTaskCreatePinnedToCore(state_controller_task, "state_controller_task", 1024 * 5, NULL, 5, NULL, 0);
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