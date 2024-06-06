import pandas as pd

# 원본 CSV 파일 경로
input_file_path = 'original_general_centerline.csv'
# 뒤집힌 데이터 저장할 CSV 파일 경로
output_file_path = 'general_centerline.csv'

# CSV 파일 읽기
data = pd.read_csv(input_file_path, header=None, names=['x', 'y'])

# 데이터 순서 뒤집기
reversed_data = data.iloc[::-1]

# 뒤집힌 데이터 저장
reversed_data.to_csv(output_file_path, index=False, header=False)

# 데이터 확인
print(reversed_data.head())