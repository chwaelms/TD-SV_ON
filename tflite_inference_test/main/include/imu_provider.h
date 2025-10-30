#ifndef IMU_H
#define IMU_H

//=========================== header ===========================
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

//=========================== define ===========================
#define IMU_PROVIDER_TAG    "imu_provider"  // log tag
#define IMU_BUF_SIZE        (6)
#define INT16_MAX_VALUE     32768.0

#define ICM42670_I2C_ADDR_GND (0x68)
#define ICM42670_I2C_ADDR_VCC (0x69)

#define EXAMPLE_I2C_MASTER_SCL (5) // GPIO number for I2C Master clock line
#define EXAMPLE_I2C_MASTER_SDA (4) // GPIO number for I2C Master data line
// #define EXAMPLE_I2C_MASTER_SCL (8) // GPIO number for I2C Master clock line
// #define EXAMPLE_I2C_MASTER_SDA (10) // GPIO number for I2C Master data line
#define EXAMPLE_INT_INPUT_PIN (0)   // GPIO number for Interrupt Input Pin

#define PORT 0
#define I2C_ADDR ICM42670_I2C_ADDR_GND

//=========================== typedef ===========================

//imu데이터 저장 타입
typedef struct {
    float buf[IMU_BUF_SIZE];
    size_t len;        
} imu_db_t;

//=========================== variables ===========================


//=========================== prototypes ===========================
//imu 센서 초기화
void init_imu(void);

//i2c 자원 반납, imu센서 free
void free_imu(void);

//imu queue를 세팅, 어디로 보낼것인지
void set_imu_queue();

//imu task 삭제 signal을 전송 받는 queue 세팅
void set_imu_delete_signal_queue();
//=========================== tasks ===========================
//imu_data 수집, queue로 send하는 task
void imu_provider_task(void * arg);

#endif