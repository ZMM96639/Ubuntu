import os
import numpy as np
import laspy
from matplotlib import pyplot as plt
from matplotlib.ticker import MultipleLocator, FuncFormatter

font = {'family': 'serif',
        'style': 'italic',
        'variant': 'normal',
        'weight': 'bold',
        'stretch': 'normal',
        'size': 25
        }


def get_suffix_files_in_dir(idir, ext):
    result_files = []
    for relative_path in os.listdir(idir):
        name, extension = os.path.splitext(relative_path)
        if extension == ext:
            result_file = os.path.join(idir, relative_path)
            result_files.append(result_file)

    size = len(result_files)
    return result_files, size


def main(path, iext):
    file, num = get_suffix_files_in_dir(path, iext)

    totalPoints = []
    for i in range(num):
        with laspy.open(file[i]) as fh:
            pointNums = fh.header.point_count
            totalPoints.append(pointNums)
    return totalPoints


if __name__ == "__main__":

    filepath = '/home/zmm/practice/version3/data/result'

    names = []
    cloudpoints = []
    for file in os.listdir(filepath):
        name, extension = os.path.splitext(file)
        result_file = os.path.join(filepath, file)
        points = main(result_file, ".laz")
        names.append(name)
        cloudpoints.append(points)

    for i in range(5):
        print("/**************************************************/")
        print("The numbers of points in " + names[i] + ":")
        print(cloudpoints[i])

        x = list(range(1, np.size(cloudpoints[i]) + 1))

        fig, ax = plt.subplots()

        plt.plot(x, cloudpoints[i], color='r', marker='o', linestyle='dashed', linewidth=2, markersize=6)
        # plt.scatter(x, cloudpoints[i], s=300, linewidths=2, c='r', marker='+')

        plt.xlim([0, 26])
        # plt.ylim([1e6, 5e6])

        # label = ["origin",
        #          "pitch-10", "pitch-5", "pitch5", "pitch10",
        #          "roll-10", "roll-5", "roll5", "roll10",
        #          "yaw-10", "yaw-5", "yaw5", "yaw10",
        #          "x-10", "x-5", "x5", "x10",
        #          "y-10", "y-5", "y5", "y10",
        #          "z-10", "z-5", "z5", "z10"]

        label = ["pitch-10", "pitch5",
                 "roll-10", "roll5",
                 "yaw-10", "yaw5",
                 "x-10", "x5",
                 "y-10", "y5",
                 "z-10", "z5"]

        plt.title(names[i], fontdict=font)

        plt.xticks([2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24], labels=label, fontsize=22, rotation=20,
                   style='italic', weight='bold')
        plt.tick_params(which='major', direction='in', length=6, width=2, bottom=0.229)

        plt.xticks(fontsize=22, rotation=30, style='italic', weight='bold')
        plt.yticks(fontsize=22, rotation=0, style='italic', weight='bold')


        # # 修改主刻度
        # xmajorLocator = MultipleLocator(2)  # 将x主刻度标签设置为2的倍数
        # # xmajorFormatter = FormatStrFormatter(label)  # 设置x轴标签文本的格式
        #
        # # 设置主刻度标签的位置,标签文本的格式
        # ax.xaxis.set_major_locator(xmajorLocator)

        # 修改副刻度
        xminorLocator = MultipleLocator(1)  # 将x轴副刻度标签设置为1的倍数

        # 设置副刻度标签的位置,没有标签文本格式
        ax.xaxis.set_minor_locator(xminorLocator)

        plt.tick_params(which='minor', direction='in', length=4, width=1.5)

        # plt.text()

        plt.xlabel("points/num", fontdict=font)
        plt.ylabel("pose/num", fontdict=font)
        plt.grid(False, which="both")
        # plt.legend(["pixels error along u and v direction"], fontsize=22)

    plt.show()
    print("/*****************/")
