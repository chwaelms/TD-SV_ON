#ifndef SPEAKER_VERIFIER_H
#define SPEAKER_VERIFIER_H

//=========================== header ==========================
#include <stdint.h>
#include <stdbool.h>


//=========================== define ===========================
#ifndef SV_EMBEDDING_DIM
#define SV_EMBEDDING_DIM 192
#endif

#define SV_MAX_SPEAKERS 40     // 시스템이 최대로 저장할 화자 수
#define SV_MAX_NAME_LEN 20     // 화자 이름 최대 길이

#define SV_VOTING_WINDOW_SIZE 9  // majority voting 윈도우 크기: (1.8s/200ms 주기 = 9개 프레임)


//=========================== typedef ===========================
// 함수 반환
typedef enum {
    SV_SUCCESS = 0,     // 성공
    SV_ERROR,           // 일반 오류
    //SV_DB_FULL,
    SV_NOT_FOUND        // 화자를 찾을 수 없음
} sv_status_t;

// 후처리 판정 알고리즘
typedef enum {
    POST_NONE,               // 후처리 없음 (가장 높은 점수 즉시 반환)
    POST_CONSECUTIVE,        // 2회 연속 판정
    POST_MAJORITY_VOTING    // Majority Voting (윈도우: 1.8초)
} sv_post_algo_t;

// 화자 DB 항목 (RAM에 저장됨)
typedef struct {
    int speaker_id;
    char speaker_name[SV_MAX_NAME_LEN];
    float embedding[SV_EMBEDDING_DIM];
} sv_speaker_entry_t;

// 
typedef struct {
    float threshold;             // 코사인 유사도 임계값
    sv_post_algo_t algorithm;    // 사용할 판정 알고리즘
} sv_config_t;

// 화자 판정 결과
typedef struct {
    int raw_speaker_id;          // 후처리 전, 현재 프레임의 최고 점수 화자 ID (-1: 임계값 미만)
    int final_speaker_id;        // 후처리 후, 최종 판정된 화자 ID (-1: 판정 보류)
    float best_score;            // 현재 프레임의 최고 유사도 점수
} sv_result_t;


// 내부 상태 구조체 (handle 내부에 포함)
typedef struct {
    // 2회 연속 판정 (POST_CONSECUTIVE) 상태
    int last_speaker_id;
    int consecutive_count;

    // Majority Voting (POST_MAJORITY_VOTING) 상태
    int history_buffer[SV_VOTING_WINDOW_SIZE]; // 화자 ID 저장 (-1은 unknown)
    int history_index; // 현재 버퍼 위치
    int history_count; // 윈도우가 찼는지 확인 (최대 SV_VOTING_WINDOW_SIZE)
} sv_internal_state_t;


// handle 구조체 (시스템의 모든 상태와 DB 관리, 사용자는 포인터 sv_handle_t*만 다룸)
typedef struct {
    sv_config_t settings;
    int max_speakers;

    // 화자 데이터베이스 (RAM)
    sv_speaker_entry_t* speakers_db; // 동적 할당될 배열
    int num_speakers;                // 현재 등록된 화자 수

    // 판정 알고리즘을 위한 내부 상태
    sv_internal_state_t state;

} sv_handle_t;


//=========================== variables ===========================


//=========================== prototypes ===========================
/**
 * @brief 화자 검증 시스템 핸들을 초기화합니다.
 * sv_database.h에 정의된 사전 등록 화자를 RAM으로 로드합니다.
 *
 * @param config 시스템 설정 (임계값, 알고리즘)
 * @return 성공 시 sv_handle_t 포인터, 실패(메모리 부족 등) 시 NULL
 */
sv_handle_t* sv_system_init(sv_config_t* config);

/**
 * @brief 시스템 핸들 및 할당된 모든 메모리를 해제합니다.
 *
 * @param handle sv_system_init()에서 반환된 핸들
 */
void sv_system_deinit(sv_handle_t* handle);

/**
 * @brief 런타임에 새로운 화자를 DB에 등록합니다.
 * (3회 녹음 후 평균화된 임베딩을 인자로 받는다고 가정)
 *
 * @param handle 핸들
 * @param new_embedding 등록할 화자의 평균 임베딩 벡터 (크기: SV_EMBEDDING_DIM)
 * @param name 등록할 화자 이름
 * @return sv_status_t (SV_SUCCESS, SV_DB_FULL 등)
 */
sv_status_t sv_system_register(sv_handle_t* handle, float* new_embedding, const char* name);

/**
 * @brief 현재 오디오 청크에서 추출된 임베딩으로 화자를 판정합니다.
 * 이 함수는 200ms마다 호출되어야 합니다.
 *
 * @param handle 핸들
 * @param current_embedding TFLite 모델에서 방금 추론된 임베딩 (크기: SV_EMBEDDING_DIM)
 * @return sv_result_t (후처리 전/후 결과 포함)
 */
sv_result_t sv_system_verify(sv_handle_t* handle, float* current_embedding);

/**
 * @brief 후처리 알고리즘의 내부 상태를 초기화합니다.
 * (예: 디렉토리 내 파일 인식 모드에서 다음 파일 시작 시 호출)
 *
 * @param handle 핸들
 */
void sv_system_reset_state(sv_handle_t* handle);


//=========================== tasks ===========================


#endif