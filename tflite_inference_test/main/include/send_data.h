#ifndef SEND_DATA_H
#define SEND_DATA_H

//=========================== header ==========================
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "state_controller.h"

//=========================== define ===========================
#define SEND_DATA_TAG "send_data"

//=========================== typedef ===========================


//=========================== variables ===========================


//=========================== prototypes ===========================

//데이터 전송을 위한 초기화
void init_send_data();

//send(데이터 전송)을 위한 queue handler 반환
QueueHandle_t get_send_queue();

//=========================== tasks ===========================

//센서 데이터를 ble로 전송하는 task(send queue로 전송받은 데이터만 전송)
void send_data_task(void * arg);

#endif