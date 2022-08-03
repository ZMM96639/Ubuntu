# -*- coding: utf-8 -*-

import subprocess
import threading
from multiprocessing import Process


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
def pdal_batch_process(odir, writer_name, oext, json_path, options):
    threads = []
    procssList = []

    for num in range(1, 18, 2):
        i = num / 100000
        output_file = odir + str(i) + oext
        command = " pdal pipeline -i " + json_path + " --filters.voxeldownsize.cell=" + str(i) + \
                  " --writers." + writer_name + ".filename=" + output_file + " " + options

        print("/*************************************/")
        print(command)

        t = threading.Thread(target=execute_command, args=(command,))
        t.start()
        threads.append(t)

        # p = Process(target=execute_command, args=(command,))
        # p.start()
        # procssList.append(p)

    for th in threads:
        th.join()

    # for pr in procssList:
    #     pr.join()


if __name__ == '__main__':
    json_path = "/home/zmm/practice/version3/data/script/point_cloud.json"
    odir = "/home/zmm/practice/version3/data/result/"

    pdal_batch_process(odir, "las", ".laz", json_path, "")
