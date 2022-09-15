# -*- coding: utf-8 -*-
import os
import subprocess
from multiprocessing import Process


def get_suffix_files_in_dir(idir, ext):
    result_files = []
    for relative_path in os.listdir(idir):
        name, extension = os.path.splitext(relative_path)
        if extension == ext:
            result_file = os.path.join(idir, relative_path)
            result_files.append(result_file)

    return result_files


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
def pdal_batch_process(idir_png, idir_pcd, reader_name, iext, odir, oext, json_path, options):
    input_files = get_suffix_files_in_dir(idir_pcd, iext)

    procssList = []
    for input_file in input_files:
        input_dir, input_relative = os.path.split(input_file)
        file_name, input_extension = os.path.splitext(input_relative)

        imgdir_relative = file_name + ".png"
        imgdir_file = os.path.join(idir_png, imgdir_relative)

        output_relative = file_name + oext
        output_file = os.path.join(odir, output_relative)

        command = " pdal pipeline -i " + json_path + \
                  " --readers." + reader_name + ".filename=" + input_file + \
                  " --filters.pcdprojecttoimages.image_file=" + imgdir_file + \
                  " --filters.pcdprojecttoimages.image_outfile=" + output_file + " " + options

        print(command)
        p = Process(target=execute_command, args=(command,))
        p.start()
        procssList.append(p)

    for pr in procssList:
        pr.join()


if __name__ == '__main__':
    json_path = "/home/zmm/practice/version2/json/pcdProjectToimages.json"
    idir_pcd = "/home/zmm/practice/version2/json/pcdToimage/pointsCloud"
    idir_png = "/home/zmm/practice/version2/json/data/pic"

    odir = "/home/zmm/practice/version2/json/pcdToimage/result"

    # oext must be equal to the suffix defined in the json_path file
    pdal_batch_process(idir_png, idir_pcd, "text", ".txt", odir, ".png", json_path, "")
