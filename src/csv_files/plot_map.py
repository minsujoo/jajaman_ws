### csv 파일을 읽어서 순서대로 색깔을 부여하며 시각화하는 python 파일입니다.
### 첫 번째 점이 나타내는 점 --> 보라색
### 마지막 점이 나타내는 점 --> 노란색

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np

##########################
###### CSV 파일 경로 #######
##########################

# file_path = 'general_centerline.csv'
file_path = 'centerline.csv'
file_path = '0.5_modified3_centerline.csv'

##########################
##########################
##########################

data = pd.read_csv(file_path, header=None, names=['x', 'y'])

# print(data.head())
colors = cm.viridis(np.linspace(0, 1, len(data)))

plt.scatter(data['x'], data['y'], c=colors)
plt.title('Scatter Plot of x vs y')
plt.xlabel('x')
plt.ylabel('y')
plt.grid(True)
plt.show()
