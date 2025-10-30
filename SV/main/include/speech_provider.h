#ifndef SPEECH_PROVIDER_H
#define SPEECH_PROVIDER_H

//=========================== header ==========================
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

//=========================== define ===========================
#define SPEECH_PROVIDER_TAG "speech_provider"
#define SPEECH_BUF_SIZE        (400)
//=========================== typedef ===========================
//음성 데이터(mic 센서) 구조체
typedef struct {
    uint8_t buf[SPEECH_BUF_SIZE];
    size_t len;        
} speech_db_t;

//=========================== variables ===========================


//=========================== prototypes ===========================

//speech 데이터를 전송할 queue handler 설정
void set_speech_queue(QueueHandle_t handler);

//mic를 사용하기위한 초기 설정
void i2s_init(void);

//=========================== tasks ===========================

//speech data를 전송하는 task(1회용 테스크, 동작 수행 후 스스로 delete)
void speech_provider_task(void * arg);

#endif