//=========================== header ==========================
#include <stdio.h>
#include <stdlib.h> // malloc, free
#include <string.h> // memcpy, strncpy, memset
#include <math.h>   // sqrtf

#include "speaker_verifier.h"  // 본 모듈의 헤더 파일
#include "sv_database.h"       // 사전에 등록된 임베딩 DB

#define LOG_I(tag, format, ...) printf("[%s] " format "\n", tag, ##__VA_ARGS__)
#define LOG_E(tag, format, ...) printf("[ERROR %s] " format "\n", tag, ##__VA_ARGS__)


//=========================== variables ===========================
// 전역 변수 (내부에서만 사용)
static const char* TAG = SV_LOG_TAG; 


//=========================== prototypes ==========================
/* private 함수들의 프로토타입 (내부에서만 사용) */ 

// cosine similarity 계산
static float sv_cosine_similarity(const float* emb1, const float* emb2, int dim);

// majority voting 윈도우에서 과반수 이상 등장한 화자 찾기
static int sv_find_majority(sv_handle_t* handle);

// sv_database.h에 정의된 등록 화자를 RAM DB로 복사
static void _load_preregistered_speakers(sv_handle_t* handle);


//=========================== public ==============================
/* speaker_verifier.h 선언된 public 함수 구현 */

sv_handle_t* sv_system_init(sv_config_t* config) {
    // 1. 핸들 메모리 할당
    sv_handle_t* handle = (sv_handle_t*)malloc(sizeof(sv_handle_t));
    if (!handle) {
        LOG_E(TAG, "Failed to allocate memory for handle");
        return NULL;
    }
    memset(handle, 0, sizeof(sv_handle_t));

    // 2. 설정 복사
    memcpy(&handle->settings, config, sizeof(sv_config_t));
    handle->max_speakers = SV_MAX_SPEAKERS; // 헤더에 정의된 값 사용

    // 3. 화자 DB 메모리 할당
    handle->speakers_db = (sv_speaker_entry_t*)malloc(sizeof(sv_speaker_entry_t) * handle->max_speakers);
    if (!handle->speakers_db) {
        LOG_E(TAG, "Failed to allocate memory for speakers_db");
        free(handle);
        return NULL;
    }
    memset(handle->speakers_db, 0, sizeof(sv_speaker_entry_t) * handle->max_speakers);

    // 4. 내부 상태 초기화 (Private 함수 호출)
    sv_system_reset_state(handle);
    
    // 5. 사전 등록된 화자 로드 (Private 함수 호출)
    handle->num_speakers = 0;
    _load_preregistered_speakers(handle);
    
    LOG_I(TAG, "SV System Initialized. %d speakers loaded. Algorithm: %d", 
          handle->num_speakers, handle->settings.algorithm);

    return handle;
}

void sv_system_deinit(sv_handle_t* handle) {
    if (handle) {
        if (handle->speakers_db) {
            free(handle->speakers_db);
        }
        free(handle);
        LOG_I(TAG, "SV System Deinitialized.");
    }
}

sv_status_t sv_system_register(sv_handle_t* handle, float* new_embedding, const char* name) {
    if (!handle) return SV_ERROR;
    // if (handle->num_speakers >= handle->max_speakers) {
    //     LOG_E(TAG, "Speaker DB is full. Cannot register new speaker.");
    //     return SV_DB_FULL;
    // }

    int new_idx = handle->num_speakers;
    sv_speaker_entry_t* entry = &handle->speakers_db[new_idx];

    // 새 ID 할당 (단순 순차 증가)
    entry->speaker_id = new_idx; 
    
    strncpy(entry->speaker_name, name, SV_MAX_NAME_LEN - 1);
    entry->speaker_name[SV_MAX_NAME_LEN - 1] = '\0'; // 널 종료 보장
    
    memcpy(entry->embedding, new_embedding, sizeof(float) * SV_EMBEDDING_DIM);

    handle->num_speakers++;
    LOG_I(TAG, "Registered new speaker: %s (ID: %d)", name, entry->speaker_id);

    return SV_SUCCESS;
}

void sv_system_reset_state(sv_handle_t* handle) {
    if (!handle) return;
    
    // sv_internal_state_t 구조체 전체를 0으로 초기화
    memset(&handle->state, 0, sizeof(sv_internal_state_t));

    // -1이 기본값이어야 하는 변수들만 다시 설정
    handle->state.last_speaker_id = -1;
    
    for(int i=0; i < SV_VOTING_WINDOW_SIZE; ++i) {
        handle->state.history_buffer[i] = -1; // -1 (unknown)으로 초기화
    }
}

sv_result_t sv_system_verify(sv_handle_t* handle, float* current_embedding) {
    sv_result_t result;
    memset(&result, 0, sizeof(sv_result_t));
    result.raw_speaker_id = -1;
    result.final_speaker_id = -1; // -1 = 판정 보류/결과 없음
    result.best_score = 0.0f;

    if (!handle || handle->num_speakers == 0) {
        // (로그를 너무 자주 찍지 않도록 주석 처리)
        // LOG_E(TAG, "Handle is null or no speakers are registered.");
        return result;
    }

    // --- 1. 원시(Raw) 판정: DB의 모든 화자와 유사도 비교 ---
    float best_score = -1.0f;
    int best_speaker_id = -1;

    for (int i = 0; i < handle->num_speakers; ++i) {
        // [Private 함수 호출]
        float score = sv_cosine_similarity(current_embedding, handle->speakers_db[i].embedding, SV_EMBEDDING_DIM);
        
        if (score > best_score) {
            best_score = score;
            best_speaker_id = handle->speakers_db[i].speaker_id;
        }
    }

    result.best_score = best_score;

    // --- 2. 임계값(민감도) 체크 ---
    if (best_score >= handle->settings.threshold) {
        result.raw_speaker_id = best_speaker_id;
    } else {
        result.raw_speaker_id = -1; // "Unknown"
    }

    // --- 3. 후처리 판정 알고리즘 적용 ---
    sv_internal_state_t* state = &handle->state;

    switch (handle->settings.algorithm) {
        case POST_CONSECUTIVE:
            if (result.raw_speaker_id != -1 && result.raw_speaker_id == state->last_speaker_id) {
                state->consecutive_count++;
            } else {
                state->last_speaker_id = result.raw_speaker_id;
                state->consecutive_count = 1;
            }

            if (state->consecutive_count >= 2) {
                result.final_speaker_id = state->last_speaker_id;
                // 판정 후 상태 초기화
                state->last_speaker_id = -1; 
                state->consecutive_count = 0;
            }
            break;

        case POST_MAJORITY_VOTING:
            // 1. 현재 판정 결과를 히스토리 버퍼(Ring Buffer)에 추가
            state->history_buffer[state->history_index] = result.raw_speaker_id;
            state->history_index = (state->history_index + 1) % SV_VOTING_WINDOW_SIZE;
            
            if (state->history_count < SV_VOTING_WINDOW_SIZE) {
                state->history_count++;
            }

            // 2. 윈도우가 가득 찼을 때만 판정 수행
            if (state->history_count == SV_VOTING_WINDOW_SIZE) {
                // [Private 함수 호출]
                int majority_id = sv_find_majority(handle);
                
                result.final_speaker_id = majority_id; // -1일 수도 있음
                
                // 판정 후 윈도우 초기화
                sv_system_reset_state(handle); 
            }
            break;

        case POST_NONE:
        default:
            result.final_speaker_id = result.raw_speaker_id;
            break;
    }

    return result;
}


//=========================== tasks ===============================
// (라이브러리 모듈이므로 Task를 직접 생성하지 않습니다.)


//=========================== private ==============================
// (이 C 파일 내부에서만 사용되는 'Private' 함수들의 실제 구현)

/**
 * @brief [Private] 두 임베딩 간의 코사인 유사도를 계산합니다.
 */
static float sv_cosine_similarity(const float* emb1, const float* emb2, int dim) {
    float dot_product = 0.0f;
    float norm1 = 0.0f;
    float norm2 = 0.0f;

    for (int i = 0; i < dim; ++i) {
        dot_product += emb1[i] * emb2[i];
        norm1 += emb1[i] * emb1[i];
        norm2 += emb2[i] * emb2[i];
    }

    // 0으로 나누기 방지
    float denom = sqrtf(norm1) * sqrtf(norm2);
    if (denom < 1e-6f) { // 1e-6 (float)
        return 0.0f;
    }

    return dot_product / denom;
}

/**
 * @brief [Private] Majority Voting 윈도우에서 과반수 이상 등장한 화자를 찾습니다.
 */
static int sv_find_majority(sv_handle_t* handle) {
    // speaker_id가 0부터 SV_MAX_SPEAKERS-1 까지라고 가정
    int speaker_counts[SV_MAX_SPEAKERS]; // 사용자가 설정한 40개 크기
    memset(speaker_counts, 0, sizeof(speaker_counts));
    
    int unknown_count = 0;

    for (int i = 0; i < handle->state.history_count; ++i) {
        int id = handle->state.history_buffer[i];
        if (id == -1) {
            unknown_count++;
        } else if (id < handle->num_speakers && id < SV_MAX_SPEAKERS) { // 유효한 ID인지 확인
            speaker_counts[id]++;
        }
    }

    int majority_id = -1;
    // 과반수 기준 (e.g., 9개 중 5개)
    int majority_threshold = (handle->state.history_count / 2) + 1;

    for (int i = 0; i < handle->num_speakers; ++i) {
        if (speaker_counts[i] >= majority_threshold) {
            majority_id = i; // i가 speaker_id와 동일
            break;
        }
    }
    
    return majority_id;
}

/**
 * @brief [Private] sv_database.h에 정의된 사전 등록 화자를 RAM DB로 복사합니다.
 */
static void _load_preregistered_speakers(sv_handle_t* handle) {
    // sv_database.h 에 정의된 전역 상수 배열을 사용
    for (int i = 0; i < NUM_PRE_REGISTERED_SPEAKERS; ++i) {
        if (handle->num_speakers >= handle->max_speakers) {
            LOG_E(TAG, "Preregistered DB is larger than max_speakers (%d)", handle->max_speakers);
            break;
        }
        
        const sv_preregistered_speaker_t* src = &PRE_REGISTERED_SPEAKERS[i];
        sv_speaker_entry_t* dst = &handle->speakers_db[handle->num_speakers];

        // ID가 중복되지 않도록 .h의 ID를 그대로 사용
        dst->speaker_id = src->speaker_id; 
        strncpy(dst->speaker_name, src->name, SV_MAX_NAME_LEN - 1);
        dst->speaker_name[SV_MAX_NAME_LEN - 1] = '\0'; // 널 종료 보장
        
        memcpy(dst->embedding, src->embedding, sizeof(float) * SV_EMBEDDING_DIM);

        handle->num_speakers++;
    }
}