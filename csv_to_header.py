import csv
import re
import os

# --- 설정 ---
CSV_FILE_PATH = "csv_file/ALL.csv"  # 변환할 CSV 파일 경로
OUTPUT_H_FILE = 'sv_database.h'   # 생성할 C 헤더 파일 이름
# ---

def create_c_variable_name(name):
    """
    "Viola_Avg" -> "VIOLA_AVG_EMB"
    "my-speaker 1" -> "MY_SPEAKER_1_EMB"
    """
    # C 변수명으로 사용할 수 없는 문자를 '_'로 변경
    s = re.sub(r'[^A-Za-z0-9_]+', '_', name)
    return s.upper() + "_EMB"

def parse_embedding_string(emb_str):
    """
    "[ 0.012 -0.045 ... ]" 문자열을 float 리스트로 변환
    kws_utils.py의 load_speakers_db 로직을 참고
    """
    # 대괄호 제거
    emb_str = emb_str.strip('[]')
    
    # 공백 기준으로 분리 (연속된 공백도 처리)
    float_strings = [s for s in emb_str.split(' ') if s]
    
    # float 리스트로 변환
    return [float(s) for s in float_strings]

# --- 메인 로직 ---
speakers_data = []
embedding_dim = 0

print(f"'{CSV_FILE_PATH}' 파일을 읽는 중...")

if not os.path.exists(CSV_FILE_PATH):
    print(f"오류: '{CSV_FILE_PATH}' 파일을 찾을 수 없습니다.")
    print("스크립트를 speaker_db.csv 파일과 같은 폴더에 위치시키거나 CSV_FILE_PATH 변수를 수정해주세요.")
    exit()

try:
    with open(CSV_FILE_PATH, mode='r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for i, row in enumerate(reader):
            speaker_name = row['speaker_name']
            
            try:
                embeddings = parse_embedding_string(row['embedding'])
            except Exception as e:
                print(f"오류: '{speaker_name}'의 임베딩 파싱 중 오류 발생: {e}")
                continue
                
            if not embeddings:
                print(f"경고: '{speaker_name}'의 임베딩 데이터가 없습니다. 건너뜁니다.")
                continue

            # 첫 번째 화자의 임베딩 길이를 기준으로 DIM 설정
            if embedding_dim == 0:
                embedding_dim = len(embeddings)
            # 임베딩 길이가 다른 경우 오류
            elif embedding_dim != len(embeddings):
                print(f"오류: '{speaker_name}'의 임베딩 길이가 다릅니다!")
                print(f"  (기대: {embedding_dim}, 실제: {len(embeddings)})")
                continue
                
            speakers_data.append({
                'id': i,
                'name': speaker_name,
                'var_name': create_c_variable_name(speaker_name),
                'embeddings': embeddings
            })

except Exception as e:
    print(f"CSV 파일 처리 중 알 수 없는 오류 발생: {e}")
    exit()

if not speakers_data:
    print("처리할 화자 데이터가 없습니다. 스크립트를 종료합니다.")
    exit()

print(f"총 {len(speakers_data)}명의 화자 데이터를 찾았습니다. (임베딩 차원: {embedding_dim})")
print(f"'{OUTPUT_H_FILE}' 파일 생성 중...")

# --- .h 파일 생성 ---
try:
    with open(OUTPUT_H_FILE, mode='w', encoding='utf-8') as f:
        
        f.write("#ifndef SV_DATABASE_H\n")
        f.write("#define SV_DATABASE_H\n\n")
        
        f.write(f"// TFLite 모델에서 출력되는 임베딩 벡터의 차원\n")
        f.write(f"#define SV_EMBEDDING_DIM {embedding_dim}\n\n")
        
        # 1. 모든 화자의 const float[] 배열 선언
        for speaker in speakers_data:
            f.write(f"// 화자: {speaker['name']}\n")
            f.write(f"const float {speaker['var_name']}[SV_EMBEDDING_DIM] = {{\n    ")
            
            # 8개씩 끊어서 줄바꿈
            for i, val in enumerate(speaker['embeddings']):
                f.write(f"{val:.8f}f, ")
                if (i + 1) % 8 == 0 and (i + 1) < len(speaker['embeddings']):
                    f.write("\n    ")
            
            f.write("\n};\n\n")
            
        # 2. 화자 목록을 참조하는 메인 구조체 배열 선언
        f.write("// ====================================================\n")
        f.write("//     spk DB structure & list\n")
        f.write("// ====================================================\n\n")
        
        f.write("// sv_init()에서 사용할 사전 등록 화자 목록\n")
        f.write("typedef struct {\n")
        f.write("    int speaker_id;\n")
        f.write("    const char* name;\n")
        f.write("    const float* embedding;\n")
        f.write("} sv_preregistered_speaker_t;\n\n")
        
        f.write("static const sv_preregistered_speaker_t PRE_REGISTERED_SPEAKERS[] = {\n")
        
        for speaker in speakers_data:
            # {ID, "이름", C변수명}
            f.write(f'    {{{speaker["id"]}, "{speaker["name"]}", {speaker["var_name"]}}},\n')
            
        f.write("};\n\n")
        
        # 3. 화자 수 define
        f.write(f"// enrolled spk: {len(speakers_data)}명\n")
        f.write(f"static const int NUM_PRE_REGISTERED_SPEAKERS = {len(speakers_data)};\n\n")
        
        f.write("#endif // SV_DATABASE_H\n")

    print(f"성공: '{OUTPUT_H_FILE}' 파일이 생성되었습니다.")

except Exception as e:
    print(f"파일 쓰기 중 오류 발생: {e}")