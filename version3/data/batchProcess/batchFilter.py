# -*- coding: utf-8 -*-
import os
import subprocess
from multiprocessing import Process

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R


def eulerconvert(path, idir):
    extr = pd.read_table(path, sep=',', engine='python')
    data = pd.read_table(idir, sep='\t', engine='python')

    Size = int(data.values.size / data.values[0].size)
    rot_ = R.from_matrix(extr.values[0:3, 0:3])
    trans_ = extr.values[0:3, 3]
    euler_ = rot_.as_euler('xyz', degrees=True)
    # print("/************/")
    # print(euler_)
    # print(trans_)
    # print("/************/")

    # Euler = []
    # Trans = []
    # Rot = []
    # Transform = []
    Transform_flatten = []
    for i in range(Size):
        euler = euler_ + data.values[i][0:3]
        # Euler.append(euler)

        tmp = R.from_euler('xyz', euler, degrees=True)
        rot = tmp.as_matrix()
        # Rot.append(rot)
        # print("/************/")
        # print(rot)

        trans = trans_ + data.values[i][3:6]
        trans = trans[:, np.newaxis]
        # Trans.append(trans)
        # print("/************/")
        # print(trans)

        fill = np.array([0, 0, 0, 1])
        fill = fill[np.newaxis, :]
        # print("/************/")
        # print(fill)

        transform = np.hstack((rot, trans))
        transform = np.vstack((transform, fill))
        # Transform.append(transform)
        # print("/************/")
        # print(transform)
        # print("/************/")

        transform_flatten = transform.flatten()
        Transform_flatten.append(transform_flatten)
        # print("/************/")
        # print(transform_flatten)
        # print("/************/")

    return Transform_flatten, Size


def execute_command(command):
    p2 = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    while p2.poll() is None:
        line = p2.stdout.readline()
        if line:
            print(line.decode("utf8", "ignore"))

    if p2.returncode == 0:
        print("execute success!")
    else:
        print("execute failure!")


# TODO too many params, try e.g. -i "--readers.text.filename=/test/*.txt" -o
# TODO  multile threads parallel

def pdal_batch_process(path, idir, odir, writer_name, oext, json_path, options):
    data, num = eulerconvert(path, idir)
    # print("/************/")
    # print(data)
    # print("/************/")
    # print(num)
    # print("/************/")
    # print(data[23])
    # print("/************/")
    # print(data[23][0])

    procssList = []
    for i in range(5):
        output_relative = "dcmt_stp25_vxlds_1e-08_" + str(i) + oext
        output_file = os.path.join(odir, output_relative)
        arg = "\"" + str(data[i][0]) + " " + str(data[i][1]) + " " + str(data[i][2]) + " " + str(data[i][3]) + \
              " " + str(data[i][4]) + " " + str(data[i][5]) + " " + str(data[i][6]) + " " + str(data[i][7]) + \
              " " + str(data[i][8]) + " " + str(data[i][9]) + " " + str(data[i][10]) + " " + str(data[i][11]) + \
              " " + str(data[i][12]) + " " + str(data[i][13]) + " " + str(data[i][14]) + " " + str(data[i][15]) + "\""

        command = "pdal pipeline -i " + json_path + " --filters.transformation.matrix=" + \
                  arg + " --writers." + writer_name + ".filename=" + output_file + " " + options

        print(command)
        p = Process(target=execute_command, args=(command,))
        p.start()
        procssList.append(p)

    for pr in procssList:
        pr.join()


if __name__ == '__main__':
    json_path = "/home/zmm/practice/version3/data/script/point_cloud.json"
    extrinsic_path = "/home/zmm/practice/version3/data/param/extrinsic.txt"

    idir = "/home/zmm/practice/version3/data/param/var.txt"
    odir = "/home/zmm/practice/version3/data/result/decimation_step25_voxeldownsize1e-08"

    pdal_batch_process(extrinsic_path, idir, odir, "las", ".laz", json_path, "")
