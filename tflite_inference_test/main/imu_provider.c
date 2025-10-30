//=========================== header ===========================
#include "imu_provider.h"
#include "state_controller.h"

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>

#include "icm42670.h"

//=========================== variables ===========================
//imu 데이터값을 읽어오기 위해 일시적으로 값을 저장할 변수
int16_t temp_imu_value;
//imu 데이터 타입
icm42670_t dev = { 0 };
//imu 데이터를 보낼 queue, 다른곳에서 생성된 큐를 연결하는 데 사용
static QueueHandle_t xQueueIMU = NULL;
//imu Task를 안전하게 삭제하는데 사용
static QueueHandle_t xQueueIMUDeleteSignal = NULL;
//queue를 통해 보내기 위해 전송 데이터타입 변수
send_data_t imu_data;

uint8_t temp_uint8;

//=========================== prototypes ==========================


//=========================== public ==============================
void init_imu(void)
{   
    ESP_ERROR_CHECK(i2cdev_init());

    ESP_ERROR_CHECK(icm42670_init_desc(&dev, I2C_ADDR, PORT, EXAMPLE_I2C_MASTER_SDA, EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(icm42670_init(&dev));

    // enable accelerometer and gyro in low-noise (LN) mode
    ESP_ERROR_CHECK(icm42670_set_gyro_pwr_mode(&dev, ICM42670_GYRO_ENABLE_LN_MODE));
    ESP_ERROR_CHECK(icm42670_set_accel_pwr_mode(&dev, ICM42670_ACCEL_ENABLE_LN_MODE));

    /* OPTIONAL */
    // enable low-pass-filters on accelerometer and gyro
    ESP_ERROR_CHECK(icm42670_set_accel_lpf(&dev, ICM42670_ACCEL_LFP_53HZ));
    ESP_ERROR_CHECK(icm42670_set_gyro_lpf(&dev, ICM42670_GYRO_LFP_53HZ));
    // set output data rate (ODR)
    ESP_ERROR_CHECK(icm42670_set_accel_odr(&dev, ICM42670_ACCEL_ODR_200HZ));
    ESP_ERROR_CHECK(icm42670_set_gyro_odr(&dev, ICM42670_GYRO_ODR_200HZ));
    // set full scale range (FSR)
    ESP_ERROR_CHECK(icm42670_set_accel_fsr(&dev, ICM42670_ACCEL_RANGE_16G));
    ESP_ERROR_CHECK(icm42670_set_gyro_fsr(&dev, ICM42670_GYRO_RANGE_2000DPS));

    // read temperature sensor value once
    float temperature;
    ESP_ERROR_CHECK(icm42670_read_temperature(&dev, &temperature));
    ESP_LOGI(IMU_PROVIDER_TAG, "Temperature reading: %f", temperature);
}

void free_imu(void)
{
    ESP_ERROR_CHECK(i2cdev_done());
    ESP_ERROR_CHECK(icm42670_free_desc(&dev));
    ESP_LOGI(IMU_PROVIDER_TAG,"free imu complete!");
}

void set_imu_queue(QueueHandle_t handler)
{
    xQueueIMU = handler;
}

void set_imu_delete_signal_queue(QueueHandle_t handler)
{
    xQueueIMUDeleteSignal = handler;
}

//=========================== tasks ===============================
void imu_provider_task(void * arg)
{
    // imu센서 초기화
    init_imu();

    // 데이터 타입 설정
    imu_data.type = IMU_DATA;
    imu_data.data.imu_data.len = IMU_BUF_SIZE;

    while (1)
    {
        BaseType_t delete_signal =  xQueueReceive(xQueueIMUDeleteSignal, &temp_uint8, 0);
        if(delete_signal == pdPASS) break;
        ESP_ERROR_CHECK(icm42670_read_raw_data(&dev, ICM42670_REG_ACCEL_DATA_X1, &temp_imu_value));
        imu_data.data.imu_data.buf[0] = temp_imu_value / INT16_MAX_VALUE;
        ESP_ERROR_CHECK(icm42670_read_raw_data(&dev, ICM42670_REG_ACCEL_DATA_Y1, &temp_imu_value));
        imu_data.data.imu_data.buf[1] = temp_imu_value / INT16_MAX_VALUE;
        ESP_ERROR_CHECK(icm42670_read_raw_data(&dev, ICM42670_REG_ACCEL_DATA_Z1, &temp_imu_value));
        imu_data.data.imu_data.buf[2] = temp_imu_value / INT16_MAX_VALUE;
        ESP_ERROR_CHECK(icm42670_read_raw_data(&dev, ICM42670_REG_GYRO_DATA_X1, &temp_imu_value));
        imu_data.data.imu_data.buf[3] = temp_imu_value / INT16_MAX_VALUE;
        ESP_ERROR_CHECK(icm42670_read_raw_data(&dev, ICM42670_REG_GYRO_DATA_Y1, &temp_imu_value));
        imu_data.data.imu_data.buf[4] = temp_imu_value / INT16_MAX_VALUE;
        ESP_ERROR_CHECK(icm42670_read_raw_data(&dev, ICM42670_REG_GYRO_DATA_Z1, &temp_imu_value));
        imu_data.data.imu_data.buf[5] = temp_imu_value / INT16_MAX_VALUE;

        //ESP_LOGI("IMU_PROVIDER_TAG", "Acc %f, %f, %f | Gyro %f, %f, %f", imu_data.data.imu_data.buf[0], imu_data.data.imu_data.buf[1], imu_data.data.imu_data.buf[2], imu_data.data.imu_data.buf[3], imu_data.data.imu_data.buf[4], imu_data.data.imu_data.buf[5]);

        //queue send
        xQueueSend(xQueueIMU, &imu_data, portMAX_DELAY);
        
        // 해당 시간은 웹에서 요구하는 딜레이 시간에 맞춘것
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    free_imu();
    vTaskDelete(NULL);
}

//=========================== private ==============================