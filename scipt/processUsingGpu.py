from numba import cuda
import subprocess
from time import time


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
    commands_ = []
    for num in range(1, 52, 10):
        i = num / 1000000
        output_file = odir + str(i) + oext
        command = " pdal pipeline -i " + json_path + " --filters.voxeldownsize.cell=" + str(i) + \
                  " --writers." + writer_name + ".filename=" + output_file + " " + options
        commands_.append(command)
    return commands_

    # print("/*************************************/")
    # print(command)
    # start = cuda.threadIdx.x + cuda.blockIdx * cuda.blockDim.x
    # final = cuda.blockDim * cuda.gridDim.x
    # execute_command(command)


@cuda.jit()
def add_kernel(commands):
    start = cuda.threadIdx.x + cuda.blockIdx * cuda.blockDim.x
    final = cuda.blockDim * cuda.gridDim.x
    for i in range(start, final):
        print("/*************************************/")
        print(commands[i])
        execute_command(commands[i])


if __name__ == '__main__':
    json_path = "/home/zmm/practice/version3/data/script/point_cloud.json"
    odir = "/home/zmm/practice/version3/data/result/"

    commands = pdal_batch_process(odir, "las", ".laz", json_path, "")

    # add_kernel[blocks_per_grid, threads_per_block]()
    add_kernel[3, 2](commands)
