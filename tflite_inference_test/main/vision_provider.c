//=========================== header ==========================
#include "vision_provider.h"
#include "esp_log.h"
#include "esp_system.h"
#include "state_controller.h"

#include <stdint.h>

//=========================== variables ===========================
// lcd말고 데이터 수집에 쓰이는 queue
static QueueHandle_t xQueueFrameO = NULL;

//=========================== prototypes ==========================


//=========================== public ==============================
void init_camera(const pixformat_t pixel_fromat,
                     const framesize_t frame_size,
                     const uint8_t fb_count)
{
    ESP_LOGI(VISION_PROVIDER_TAG, "Camera module is %s", CAMERA_MODULE_NAME);

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = CAMERA_PIN_D0;
    config.pin_d1 = CAMERA_PIN_D1;
    config.pin_d2 = CAMERA_PIN_D2;
    config.pin_d3 = CAMERA_PIN_D3;
    config.pin_d4 = CAMERA_PIN_D4;
    config.pin_d5 = CAMERA_PIN_D5;
    config.pin_d6 = CAMERA_PIN_D6;
    config.pin_d7 = CAMERA_PIN_D7;
    config.pin_xclk = CAMERA_PIN_XCLK;
    config.pin_pclk = CAMERA_PIN_PCLK;
    config.pin_vsync = CAMERA_PIN_VSYNC;
    config.pin_href = CAMERA_PIN_HREF;
    config.pin_sscb_sda = CAMERA_PIN_SIOD;
    config.pin_sscb_scl = CAMERA_PIN_SIOC;
    config.pin_pwdn = CAMERA_PIN_PWDN;
    config.pin_reset = CAMERA_PIN_RESET;
    config.xclk_freq_hz = XCLK_FREQ_HZ;
    config.pixel_format = pixel_fromat;
    config.frame_size = frame_size;
    config.jpeg_quality = 12;
    config.fb_count = fb_count;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    // camera init
    esp_err_t err = esp_camera_init(&config);
    //초기화 불안정을 해결하기위해 여러번 시도.
    uint8_t count = 5;
    while(err != ESP_OK && count > 0)
    {
        err = esp_camera_init(&config);
        count--;
        vTaskDelay(10);
    }
    if (err != ESP_OK)
    {
        ESP_LOGE(VISION_PROVIDER_TAG, "Camera init failed with error 0x%x", err);
        esp_restart();
    }
    sensor_t *s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID || s->id.PID == OV2640_PID) {
        s->set_vflip(s, 1); //flip it back    
    } else if (s->id.PID == GC0308_PID) {
        s->set_hmirror(s, 0);
    } else if (s->id.PID == GC032A_PID) {
        s->set_vflip(s, 1);
    }

    //initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID)
    {
        s->set_brightness(s, 1);  //up the blightness just a bit
        s->set_saturation(s, -2); //lower the saturation
    }
    ESP_LOGI(VISION_PROVIDER_TAG, "Camera init complete!");
}

void set_camera_frame_out_queue(QueueHandle_t handler)
{
    xQueueFrameO = handler;
}

void resize_and_convert_rgb_i8(camera_fb_t* frame, int8_t *result_buf)
{
    for (int i = 0; i < RESIZE_ROWS; i++){
        for(int j = 0; j < RESIZE_COLS; j++){
            int index1 = (int)(i*2.5) * FRAME_SIZE + (int)(j * 2.5);
            int index2 = (int)(i*2.5) * FRAME_SIZE + (int)(j * 2.5) + 1;
            int index3 = (int)((i+1)*2.5) * FRAME_SIZE + (int)(j * 2.5);
            int index4 = (int)((i+1)*2.5) * FRAME_SIZE + (int)(j * 2.5) + 1;
            uint16_t pixel_1 = ((uint16_t *) (frame->buf))[index1];
            uint16_t pixel_2 = ((uint16_t *) (frame->buf))[index2];
            uint16_t pixel_3 = ((uint16_t *) (frame->buf))[index3];
            uint16_t pixel_4 = ((uint16_t *) (frame->buf))[index4];

            // for inference
            uint8_t hb1 = pixel_1 & 0xFF;
            uint8_t lb1 = pixel_1 >> 8;
            uint8_t b1 = ((lb1 & 0x1F) << 3);
            uint8_t g1 = (((hb1 & 0x07) << 5) | ((lb1 & 0xE0) >> 3));
            uint8_t r1 = (hb1 & 0xF8);

            uint8_t hb2 = pixel_2 & 0xFF;
            uint8_t lb2 = pixel_2 >> 8;
            uint8_t b2 = ((lb2 & 0x1F) << 3);
            uint8_t g2 = (((hb2 & 0x07) << 5) | ((lb2 & 0xE0) >> 3));
            uint8_t r2 = (hb2 & 0xF8);

            uint8_t hb3 = pixel_3 & 0xFF;
            uint8_t lb3 = pixel_3 >> 8;
            uint8_t b3 = ((lb3 & 0x1F) << 3);
            uint8_t g3 = (((hb3 & 0x07) << 5) | ((lb3 & 0xE0) >> 3));
            uint8_t r3 = (hb3 & 0xF8);

            uint8_t hb4 = pixel_4 & 0xFF;
            uint8_t lb4 = pixel_4 >> 8;
            uint8_t b4 = ((lb4 & 0x1F) << 3);
            uint8_t g4 = (((hb4 & 0x07) << 5) | ((lb4 & 0xE0) >> 3));
            uint8_t r4 = (hb4 & 0xF8);

            uint8_t r = (r1 / 4) + (r2 / 4) + (r3 / 4) + (r4 / 4);
            uint8_t g = (g1 / 4) + (g2 / 4) + (g3 / 4) + (g4 / 4);
            uint8_t b = (b1 / 4) + (b2 / 4) + (b3 / 4) + (b4 / 4);
            
            //int8_t로 보내기
            result_buf[(i*RESIZE_COLS+j)*NUM_CHANNEL] = r ^ 0x80;
            result_buf[(i*RESIZE_COLS+j)*NUM_CHANNEL + 1] = g ^ 0x80;
            result_buf[(i*RESIZE_COLS+j)*NUM_CHANNEL + 2] = b ^ 0x80;
        }
    }
}

void camera_fb_return(camera_fb_t* frame_buffer)
{
    esp_camera_fb_return(frame_buffer);
}
//=========================== tasks ===============================
void vision_provider_task(void *arg)
{
    ESP_LOGI(VISION_PROVIDER_TAG, "vision_provider_task Start!");
    init_camera(PIXFORMAT_RGB565, FRAMESIZE_240X240, 2);
    while (1)
    {
        camera_fb_t *frame = esp_camera_fb_get();
        if (frame)
        {
            xQueueSend(xQueueFrameO, &frame, portMAX_DELAY);
        }
    }
}


//=========================== private ==============================


