//=========================== header ==========================
#include "dummy_test.h"

#include "esp_log.h"
#include "esp_event.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>

#include "imu_provider.h"

//=========================== variables ===========================
static QueueHandle_t xQueueDummy = NULL;
imu_db_t recievd_imu_buffer;

//=========================== prototypes ==========================


//=========================== public ==============================

QueueHandle_t get_dummy_queue_handler()
{
    return xQueueDummy;
}

//=========================== tasks ===============================
void dummy_test_task(void * arg)
{
    xQueueDummy = xQueueCreate(5, sizeof(imu_db_t));

    while(1) 
    {
        if (xQueueReceive(xQueueDummy, &recievd_imu_buffer, portMAX_DELAY))
        {
            ESP_LOGI(DUMMY_TAG, "%f %f %f %f %f %f", recievd_imu_buffer.buf[0], recievd_imu_buffer.buf[1], recievd_imu_buffer.buf[2], recievd_imu_buffer.buf[3], recievd_imu_buffer.buf[4], recievd_imu_buffer.buf[5]);
        }
    }
}

//=========================== private ==============================


