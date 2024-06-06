import pandas as pd
from scipy.interpolate import splrep, splev
import numpy as np
import matplotlib.pyplot as plt

# CSV 파일 읽기
file_path = 'centerline.csv'
data = pd.read_csv(file_path, header=None, names=['x', 'y'])

# x와 y 데이터를 numpy 배열로 변환
x = data['x'].values
y = data['y'].values

# 스플라인 보간을 위한 tck 생성
tck = splrep(x, y, s=0)

# 새로운 x 값 생성 (보간된 곡선을 그리기 위해)
x_new = np.linspace(x.min(), x.max(), 400)
y_new = splev(x_new, tck)

# 원래 데이터와 스플라인 데이터 시각화
plt.figure(figsize=(10, 6))
plt.plot(x, y, 'o', label='Original data')
plt.plot(x_new, y_new, '-', label='Spline fit')
plt.legend()
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Spline Interpolation of Waypoints')
plt.show()
