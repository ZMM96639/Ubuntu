import os

import pdal
import matplotlib.pyplot as plt

font = {'family': 'serif',
        'style': 'italic',
        'variant': 'normal',
        'weight': 'bold',
        'stretch': 'normal',
        'size': 25
        }

path = "/home/zmm/practice/version2/json/drawPointpairs/Err"

theta_u = []
theta_v = []
for num in range(1, 12):
    file = str(num) + ".txt"
    filepath = os.path.join(path, file)
    pipeline = pdal.Reader(filename=filepath).pipeline()
    pipeline.execute()
    arr = pipeline.arrays[0]

    for i in range(4):
        if abs(arr[i][5]) < 50 and abs(arr[i][6]) < 50:
            theta_u.append(arr[i][5])
            theta_v.append(arr[i][6])

fig, ax = plt.subplots()
plt.scatter(theta_u, theta_v, s=150, linewidths=1.5, c='r', marker='+')

plt.xlim([-50, 40])
plt.ylim([-40, 40])
plt.title("The reprojection of error", fontdict=font)
plt.xlabel("u/pixels", fontdict=font)
plt.ylabel("v/pixels", fontdict=font)
plt.xticks(fontsize=22, rotation=0, style='italic', weight='bold')
plt.yticks(fontsize=22, rotation=0, style='italic', weight='bold')

# plt.text()

plt.grid(False)
# plt.legend(["pixels error along u and v direction"], fontsize=22)
plt.show()
