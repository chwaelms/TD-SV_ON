#ifndef VISION_PROVIDER_H
#define VISION_PROVIDER_H

//=========================== header ==========================
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_camera.h"
//=========================== define ===========================
#define VISION_PROVIDER_TAG "vision_provider"

#define CAMERA_MODULE_NAME "GAMBA_AI_EDU_KIT"
#define CAMERA_PIN_PWDN -1
#define CAMERA_PIN_RESET -1

#define CAMERA_PIN_VSYNC 6
#define CAMERA_PIN_HREF 7
#define CAMERA_PIN_PCLK 13
#define CAMERA_PIN_XCLK 15

#define CAMERA_PIN_SIOD 4
#define CAMERA_PIN_SIOC 5

#define CAMERA_PIN_D0 11
#define CAMERA_PIN_D1 9
#define CAMERA_PIN_D2 8
#define CAMERA_PIN_D3 10
#define CAMERA_PIN_D4 12
#define CAMERA_PIN_D5 18
#define CAMERA_PIN_D6 17
#define CAMERA_PIN_D7 16

#define XCLK_FREQ_HZ 10000000

#define RESIZE_ROWS 96
#define RESIZE_COLS 96
#define NUM_CHANNEL 3

#define FRAME_SIZE 240
//=========================== typedef ===========================
//vision데이터 저장 타입
typedef struct {
    camera_fb_t* frame;
} vision_db_t;

//=========================== variables ===========================


//=========================== prototypes ===========================
//camera sensor 사용을 위한 초기화
void init_camera(const pixformat_t pixel_fromat,
                 const framesize_t frame_size,
                 const uint8_t fb_count);

//vision 데이터 전송을 위한 queue 연결
void set_camera_frame_out_queue(QueueHandle_t handler);

#ifdef __cplusplus
extern "C" {
#endif
//이미지 96*96으로 변환하여 전송 (int8_t형으로 반환)
void resize_and_convert_rgb_i8(camera_fb_t* frame, int8_t *result_buf);

//카메라 프레임을 사용후 다시 반환
void camera_fb_return(camera_fb_t* frame_buffer); 
#ifdef __cplusplus
}
#endif
//=========================== tasks ===========================
// 카메라 센서로 이미지를 읽어와 lcd 테스크로 전송
void vision_provider_task(void *arg);



#endif