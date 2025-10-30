#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "speaker_verifier.h"

//=========================== define ===========================
#define APP_MAIN_TAG    "app_main"  // log tag
#define DEFAULT_SV_SENSITIVITY   (0.5f)
#define DEFAULT_SV_ALGORITHM     (POST_CONSECUTIVE)


// (여기에 KWS 등 다른 모듈의 설정도 추가할 수 있습니다)

#endif