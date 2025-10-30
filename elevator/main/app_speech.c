#include "app_speech.h"             
// 자기 자신의 헤더 (필수)

const char output_class[16][13] = {"", "구층", "사층", "삼층", "십사층", "십삼층", "십오층", "십이층", "십일층", " 십층","오층","육층","이층","일층","칠층","팔층"};
// 내부 변수 (static)
static int16_t mfcc_buffer[NUM_FRAMES * NUM_MFCC_COEFFS]; // MFCC 버퍼 (전역변수 - RAM의 .bss 섹션)
static src_cfg_t srcif;     // 구조체 생성 -> 이게 handle (src_cfg_t는 typedef로 만든 타입 이름, srcif는 실제 handle)
QueueHandle_t sndQueue;


// 내부 함수 (static) -> 이 파일 내부에서만 사용
static void i2s_init(void)
{
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,			//the mode must be set according to DSP configuration
        .sample_rate = 16000,                           //must be the same as DSP configuration
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,   //must be the same as DSP configuration
        .bits_per_sample = 32,                          //must be the same as DSP configuration
        .communication_format = I2S_COMM_FORMAT_I2S,
        .dma_buf_count = 3,
        .dma_buf_len = 300,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2,
    };
    i2s_pin_config_t pin_config = {
        .bck_io_num = 26,  // IIS_SCLK
        .ws_io_num = 32,   // IIS_LCLK
        .data_out_num = -1,// IIS_DSIN
        .data_in_num = 33  // IIS_DOUT
    };
    i2s_driver_install(1, &i2s_config, 0, NULL);
    i2s_set_pin(1, &pin_config);
    i2s_zero_dma_buffer(1);
}

void src_task(void *arg)
{
    i2s_init();

    // 헤더 파일의 구조체를 인자로 받음 (src_cfg_t) -> 전역 변수 아니라 인자로 받은 구조체 사용
    src_cfg_t *cfg=(src_cfg_t*)arg;
    // cfg를 통해 데이터 접근
    size_t samp_len = cfg->item_size*2*sizeof(int)/sizeof(int16_t);
    int *samp = heap_caps_malloc(samp_len, MALLOC_CAP_8BIT);
    size_t read_len = 0;

    while(1) {
        i2s_read(1, samp, samp_len, &read_len, portMAX_DELAY);
        for (int x=0; x<cfg->item_size/4; x++) {
            int s1 = ((samp[x * 4] + samp[x * 4 + 1]) >> 13) & 0x0000FFFF;
            int s2 = ((samp[x * 4 + 2] + samp[x * 4 + 3]) << 3) & 0xFFFF0000;
            samp[x] = s1 | s2;
        }
        xQueueSend(*cfg->queue, samp, portMAX_DELAY);
    }
	
    vTaskDelete(NULL);
}

void nn_task(void *arg)
{
    int audio_chunksize = 16000;
    // 실시간 오디오 버퍼 (RAM에 동적 할당)
	int16_t * audio_buffer = heap_caps_malloc(audio_chunksize * sizeof(int16_t), MALLOC_CAP_8BIT);
	int16_t * p1 = audio_buffer;
    int16_t * p2 = audio_buffer + 3200;
	int16_t * p3 = audio_buffer + 6400;
	int16_t * p4 = audio_buffer + 9600;
	int16_t * p5 = audio_buffer + 12800;

    assert(audio_buffer);
	xQueueReceive(sndQueue, p1, portMAX_DELAY);
	xQueueReceive(sndQueue, p2, portMAX_DELAY);
	xQueueReceive(sndQueue, p3, portMAX_DELAY);
	xQueueReceive(sndQueue, p4, portMAX_DELAY);
	
    while(1) {
        xQueueReceive(sndQueue, p5, portMAX_DELAY);                                         // 새 오디오 수신
		int32_t mfcc_buffer_head = (NUM_FRAMES - RECORDING_WIN) * NUM_MFCC_COEFFS; 
		for (uint16_t f = 0; f < RECORDING_WIN; f++) {
			mfcc_compute(audio_buffer + (f * FRAME_SHIFT), &mfcc_buffer[mfcc_buffer_head]); // mfcc 계산 (RAM에서 작업)
			mfcc_buffer_head += NUM_MFCC_COEFFS;
		}
		g_state = dscnn_run(mfcc_buffer);                                                   // 모델 추론
		ESP_LOGI("", "%s", output_class[g_state]);
		
		memcpy(p1, p2, 3200 * 2);                                                           // 버퍼 순환 (오래된 데이터 삭제)
		memcpy(p2, p3, 3200 * 2);
		memcpy(p3, p4, 3200 * 2);
		memcpy(p4, p5, 3200 * 2);
    }

    free(audio_buffer);
    vTaskDelete(NULL);
}

// 공개 함수 (public)
void app_speech_init()      // 헤더에 선언된 함수(함수 프로토타입?)의 실제 구현
{
	int audio_chunksize = 3200;
	
	mfcc_init();            // 다른 모듈의 함수 호출
	dscnn_init();

    // 태스트 생성 코드
    sndQueue = xQueueCreate(2, (audio_chunksize * sizeof(int16_t)));
    // handle srcif에 상태 정보 저장
    srcif.queue = &sndQueue;   
    srcif.item_size = audio_chunksize * sizeof(int16_t);

    // srcif 구제체를 인자로 전달 -> handle를 함수에 전달
    xTaskCreatePinnedToCore(&src_task, "src", 3*1024, (void*)&srcif, 5, NULL, 0);
    xTaskCreatePinnedToCore(&nn_task, "nn", 2*1024, NULL, 5, NULL, 1);
}
