// 중복 include 방지 (헤더 가드 시작)
#ifndef SPEECH_H 					// SPEECH_H가 정의되지 않았다면
#define SPEECH_H					// SPEECH_H를 정의하라

// 헤더파일 및 다른 모듈 헤더
#include <stdio.h>					// 표준 라이브러리
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>
// 내 프로젝트 파일
#include "freertos/FreeRTOS.h"		
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "xtensa/core-macros.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_partition.h"
#include "esp_spiffs.h"
#include "esp_log.h"
#include "driver/i2s.h"
#include "sdkconfig.h"
#include "app_main.h"
#include "mfcc.h"
#include "dscnn.h"

// 구조체 정의 (typedef struct)
typedef struct {
	QueueHandle_t *queue;			// 멤버 변수 1
	int item_size;					// 멤버 변수 2
} src_cfg_t;						// 이름 -> 다른 파일에서 src_cfg_t 타입 사용 가능

// 함수 프로토타입 (선언만)
void app_speech_init();

// (헤더 가드 끝)
#endif
