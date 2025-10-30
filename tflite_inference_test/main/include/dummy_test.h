#ifndef DUMMY_H
#define DUMMY_H

//=========================== header ==========================
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

//=========================== define ===========================
#define DUMMY_TAG    "dummy test task"  // log tag

//=========================== typedef ===========================


//=========================== variables ===========================


//=========================== prototypes ===========================
//더미테스크에서 사용하는 queue handler 반환
QueueHandle_t get_dummy_queue_handler();

//=========================== tasks ===========================
//테스크용 더미테스크
void dummy_test_task(void * arg);

#endif