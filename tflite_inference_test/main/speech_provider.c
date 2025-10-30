//=========================== header ==========================
#include "speech_provider.h"
#include "state_controller.h"
#include "dummy_test.h"
#include "ble_communication.h"

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>

#include "driver/i2s.h"

//=========================== variables ===========================
const int16_t i2s_bytes_to_read = 3200;
size_t bytes_read;
uint8_t i2s_read_buffer[3200] = {};

static QueueHandle_t xQueueSpeech = NULL;

speech_db_t speech_buffer;
send_data_t speech_data;

//=========================== prototypes ==========================


//=========================== public ==============================
// audio sensor init
void i2s_init(void) {
  // Start listening for audio: MONO @ 16KHz
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX),
      .sample_rate = 16000,
      .bits_per_sample = (i2s_bits_per_sample_t)16,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_I2S,
      .intr_alloc_flags = 0,
      .dma_buf_count = 3,
      .dma_buf_len = 300,
      .use_apll = false,
      .tx_desc_auto_clear = false,
      .fixed_mclk = -1,
  };
  i2s_pin_config_t pin_config = {
      .bck_io_num = 41,    // IIS_SCLK
      .ws_io_num = 42,     // IIS_LCLK
      .data_out_num = -1,  // IIS_DSIN
      .data_in_num = 2,   // IIS_DOUT
  };
  esp_err_t ret = 0;
  ret = i2s_driver_install((i2s_port_t)1, &i2s_config, 0, NULL);
  if (ret != ESP_OK) {
    ESP_LOGE(SPEECH_PROVIDER_TAG, "Error in i2s_driver_install");
  }
  ret = i2s_set_pin((i2s_port_t)1, &pin_config);
  if (ret != ESP_OK) {
    ESP_LOGE(SPEECH_PROVIDER_TAG, "Error in i2s_set_pin");
  }

  ret = i2s_zero_dma_buffer((i2s_port_t)1);
  if (ret != ESP_OK) {
    ESP_LOGE(SPEECH_PROVIDER_TAG, "Error in initializing dma buffer with 0");
  }
  speech_buffer.len = SPEECH_BUF_SIZE;
  speech_data.type = SPEECH_DATA;
}

void set_speech_queue(QueueHandle_t handler)
{
    xQueueSpeech = handler;
}

//=========================== tasks ===============================
// 1초동안 audio data Capture


void speech_provider_task(void * arg) {
  ESP_LOGI(SPEECH_PROVIDER_TAG, "speech_provider_task created!");
  bytes_read = i2s_bytes_to_read;
  // 100ms 마다 3200 bytes씩 음성 데이터를 읽어와 전송 (10번 반복하여 1초동안의 오디오 데이터를 전송)
  for(int i=0;i<10;i++) {
    /* read 100ms data at once from i2s */
    i2s_read((i2s_port_t)1, (void*)i2s_read_buffer, i2s_bytes_to_read,
            &bytes_read, 10);
    if (bytes_read <= 0) {
      ESP_LOGE(SPEECH_PROVIDER_TAG, "Error in I2S read : %d", bytes_read);
    }
    else {
      if (bytes_read < i2s_bytes_to_read) {
        ESP_LOGW(SPEECH_PROVIDER_TAG, "%d: Partial I2S read %d < %d", i, bytes_read, i2s_bytes_to_read);
      }
      for(uint8_t j=0;j<8;j++) {
        // 3200 bytes를 400bytes씩 8번 나누어 전송 (MTU가 500으로 설정되어있어 끊어서 전송이 필요함)
        memcpy(speech_buffer.buf,(i2s_read_buffer +j*400),sizeof(uint8_t)*400);
        speech_data.data.speech_data = speech_buffer;
        xQueueSend(xQueueSpeech, &speech_data, portMAX_DELAY);
      }
    }
  }
  ESP_LOGI(SPEECH_PROVIDER_TAG, "send COMPLETE!!");
  vTaskDelete(NULL);
}

//=========================== private ==============================


