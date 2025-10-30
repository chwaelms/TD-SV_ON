//=========================== header ==========================
#include "send_data.h"

#include "esp_log.h"
#include "esp_event.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>

#include "ble_communication.h"
#include "imu_provider.h"
#include "state_controller.h"
#include "vision_provider.h"

#include "esp_heap_caps.h"

//=========================== variables ===========================
//센서의 데이터를 받는 queue
QueueHandle_t xQueueSendData = NULL;
//xQueueSendData queue를 통해 받은 데이터를 저장할 변수
send_data_t received_data;
// 이미지 데이터 변환시 저장할 공간
int8_t resize_frame[RESIZE_ROWS * RESIZE_COLS * NUM_CHANNEL] __attribute__((section(".ext_ram.bss")));

//=========================== prototypes ==========================

//=========================== public ==============================
void init_send_data()
{
    xQueueSendData = xQueueCreate(5, sizeof(send_data_t));
}

QueueHandle_t get_send_queue()
{
    return xQueueSendData;
}


//=========================== tasks ===============================
void send_data_task(void * arg)
{   
    ESP_LOGI(SEND_DATA_TAG,"send_data_task start!");
    while(1) {
        if (xQueueReceive(xQueueSendData, &received_data, portMAX_DELAY))
        {   
            switch (received_data.type) {
                case IMU_DATA: {
                    imu_db_t imu_data = received_data.data.imu_data;
                    send_data_to_ble(imu_data.buf, sizeof(imu_data.buf), IMU_DATA);
                    break;
                }
                case SPEECH_DATA: {
                    speech_db_t speech_data = received_data.data.speech_data;
                    send_data_to_ble(speech_data.buf, sizeof(speech_data.buf), SPEECH_DATA);
                    break;
                }
                case VISION_DATA: {
                    vision_db_t vision_data = received_data.data.vision_data;
                    resize_and_convert_rgb_i8(vision_data.frame, resize_frame);
                    camera_fb_return(vision_data.frame);
                    // ble로 96*3씩 끊어서 93번 전송, 96*10 = 960ms로 약 1초정도 소요
                    for(int i=0;i<RESIZE_ROWS;i++) {
                        send_data_to_ble(&resize_frame[i*RESIZE_COLS* NUM_CHANNEL], RESIZE_ROWS* NUM_CHANNEL, VISION_DATA);
                        vTaskDelay(pdMS_TO_TICKS(10));
                    }
                    break;
                }
                default:
                    // 알 수 없는 데이터 형식 처리
                    ESP_LOGE(SEND_DATA_TAG, "data type error!");
                    break;
            }
        }
    }
}




//=========================== private ==============================