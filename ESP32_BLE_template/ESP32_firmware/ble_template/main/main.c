#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include "esp_log.h"

#include <ble_communication.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

void app_main(void)
{
    // ble 초기화
    ble_begin();

    // ble 연결 대기
    while(1) {
        if(check_ble_connect()) {
            ESP_LOGI("Main", "Connected ble!");
            break;
        }
        ESP_LOGI("Main", "BLE connect waiting...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // 임의의 더미값
    // BLE 전송은 무조건 배열 형태로 전송해야합니다. 데이터 1개를 보낼 때도 배열로 선언하여 전송합니다.
    uint8_t dummy_data[7] = {1,2,3,4,5,6,7};

    // 임의로 1초마다 50회 전송하도록 구현
    for(int i=0; i<50;i++) {
        // 전송하고자 하는 데이터 배열의 주소를 입력으로 줘야합니다.
        send_data_to_ble(&dummy_data, sizeof(dummy_data));
        ESP_LOGI("Main", "Send data compelte...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}