#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h> // rand, RAND_MAX
/* 

통합 단계:

기존 app_main.c 파일을 엽니다. (이것이 앞으로 작업할 메인 파일입니다.)

제가 만든 main.c (사용 예시) 파일을 옆에 엽니다.

[초기화 코드 추가]

제 main.c에 있는 sv_config_t my_config; 와 sv_system = sv_system_init(&my_config); 부분을 복사합니다.

이것을 기존 app_main.c의 app_main 함수 상단 (다른 모듈들, 예를 들어 model_manager_init() 같은 것을 호출하는 곳)에 붙여넣습니다.

(당연히 app_main.h에 #include "speaker_verifier.h"가 추가되어 있어야 합니다.)

[판정 코드 추가]

제 main.c의 메인 루프(while (true)) 안에 있는 sv_result_t result = sv_system_verify(...) 와 if (result.final_speaker_id ...)로 시작하는 결과 처리 부분을 복사합니다.

이것을 기존 app_main.c의 메인 루프(while (true)) 안, 실제 TFLite 추론이 끝난 직후 (예: run_tflite_inference(...) 또는 interpreter->Invoke()가 끝난 바로 다음 줄)에 붙여넣습니다.

중요: sv_system_verify에 전달하는 current_embedding 인자는, 제가 만든 가짜 current_embedding이 아니라, app_main.c가 TFLite에서 '실제로' 받아온 임베딩 출력 텐서(예: output->data.f)여야 합니다.

*/







/* * 임베디드 환경에서는 <unistd.h>의 sleep 대신 
 * FreeRTOS의 vTaskDelay를 사용합니다.
 *
 * #include "freertos/FreeRTOS.h"
 * #include "freertos/task.h"
 * #define app_sleep_ms(ms) vTaskDelay(pdMS_TO_TICKS(ms))
 */
#include <unistd.h> 
#define app_sleep_ms(ms) sleep(ms / 1000) // 표준 C에서는 초 단위 sleep


// --- TFLite 모델 추론 (가상 시뮬레이션 함수) ---
/*
 * 실제 프로젝트에서는 이 함수를 TFLite-Micro 추론 코드로 대체해야 합니다.
 * (tflite_inference_test/main/model_manager.cc의
 * TfLiteTensor* output = interpreter->output(0);
 * memcpy(output_embedding, output->data.f, ...);
 * 와 같은 코드가 될 것입니다.)
 */
bool run_tflite_inference(int frame_count, float* output_embedding) {
    
    // --- 테스트를 위한 임베딩 데이터 조작 ---
    // 2회 연속 판정 알고리즘(POST_CONSECUTIVE)을 테스트하기 위해
    // 2번은 "Viola_Avg", 2번은 "Viola_ON", 1번은 "Unknown"을 시뮬레이션합니다.

    if (frame_count == 0 || frame_count == 1) {
        // "Viola_Avg" (ID: 0) 임베딩 복사 + 약간의 노이즈
        memcpy(output_embedding, VIOLA_AVG_EMB, sizeof(float) * SV_EMBEDDING_DIM);
        output_embedding[0] += 0.001f * (rand() % 10); // 노이즈
    } 
    else if (frame_count == 2 || frame_count == 3) {
        // "Viola_ON" (ID: 1) 임베딩 복사 + 약간의 노이즈
        memcpy(output_embedding, VIOLA_ON_EMB, sizeof(float) * SV_EMBEDDING_DIM);
        output_embedding[0] += 0.001f * (rand() % 10); // 노이즈
    }
    else {
        // "Unknown" (완전한 랜덤 데이터)
        for(int i=0; i < SV_EMBEDDING_DIM; ++i) {
            output_embedding[i] = (float)rand() / (float)RAND_MAX;
        }
    }
    
    return true; 
}


// --- 오디오 입력 (가상 시뮬레이션 함수) ---
/*
 * 실제 프로젝트에서는 이 함수가 I2S 마이크 등에서
 * 200ms 분량의 오디오 청크를 받아오는 루프입니다.
 * (tflite_inference_test/main/speech_provider.c 참조)
 */
bool get_audio_chunk_and_run_inference(int frame_count, float* output_embedding) {
    // 200ms 대기 시뮬레이션
    printf("."); // 200ms 마다 '.' 출력
    fflush(stdout); // 즉시 출력
    app_sleep_ms(200);

    // 오디오 데이터로 TFLite 추론 실행 (가상)
    return run_tflite_inference(frame_count, output_embedding);
}


/*
 * C++ 프로젝트 (ESP-IDF)에서는 main.c가 아닌 app_main.c 이며,
 * C++ 컴파일러가 main을 찾도록 C++ 링키지를 사용해야 할 수 있습니다.
 *
 * extern "C" void app_main(void) { ... }
 */
int main() {
    printf("화자 검증(SV) 시스템 C언어 데모 시작\n");
    printf("200ms 주기로 TFLite 추론을 시뮬레이션합니다.\n\n");

    // 1. 시스템 설정
    sv_config_t my_config;
    my_config.threshold = DEFAULT_SV_SENSITIVITY;   // 임계값 (매우 높게 설정 - 시뮬레이션이므로)
    my_config.algorithm = DEFAULT_SV_ALGORITHM;   // 2회 연속 판정 알고리즘 사용
    
    // (Majority Voting 테스트 시)
    // my_config.algorithm = POST_MAJORITY_VOTING;

    // 2. 시스템 초기화 (사전 등록된 DB 로드)
    sv_handle_t* sv_system = sv_system_init(&my_config);
    if (!sv_system) {
        printf("\n[오류] 시스템 초기화 실패! (메모리 부족?)\n");
        return -1;
    }
    
    for (int i=0; i < sv_system->num_speakers; ++i) {
        printf("  - 로드된 화자 ID %d: %s\n", 
               sv_system->speakers_db[i].speaker_id,
               sv_system->speakers_db[i].speaker_name);
    }
    printf("\n실시간 추론 루프 시작 (총 10 프레임, 2초 시뮬레이션):\n");

    // 3. (옵션) 런타임에 화자 등록
    // float runtime_emb[SV_EMBEDDING_DIM] = { ... }; // 3회 녹음 평균 임베딩
    // sv_system_register(sv_system, runtime_emb, "NewUser");


    // 4. 실시간 추론 루프 (200ms 주기)
    float current_embedding[SV_EMBEDDING_DIM];

    for (int frame = 0; frame < 10; ++frame) {
        
        // (A) 오디오 입력 및 TFLite 추론
        if (!get_audio_chunk_and_run_inference(frame, current_embedding)) {
            break;
        }

        // (B) 화자 판정 (핵심 API 호출)
        sv_result_t result = sv_system_verify(sv_system, current_embedding);

        // (C) 결과 처리
        if (result.final_speaker_id != -1) {
            // 최종 판정 성공!
            int id = result.final_speaker_id;
            printf("\n\n==============================================\n");
            printf("[최종 판정!] 화자: %s (ID: %d), Score: %.4f\n", 
                   sv_system->speakers_db[id].speaker_name, id, result.best_score);
            printf("==============================================\n\n");
        } 
        else if (result.raw_speaker_id != -1) {
            // 원시 판정 성공 (후처리 대기 중)
            printf("  (판정 중... %s, Score: %.4f)\n", 
                   sv_system->speakers_db[result.raw_speaker_id].speaker_name,
                   result.best_score);
        } else {
            // 원시 판정 실패 (Unknown)
             printf("  (Unknown, Score: %.4f)\n", result.best_score);
        }
    }

    // 5. 시스템 종료
    sv_system_deinit(sv_system);
    printf("\n\n데모 종료.\n");
    return 0;
}