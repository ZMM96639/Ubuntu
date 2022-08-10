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
def pdal_batch_process(idir_png, idir_pcd, reader_name, iext, odir_img, odir_writer, writer_name, oext, json_path,
                       options):
    input_files = get_suffix_files_in_dir(idir_pcd, iext)

    procssList = []
    for input_file in input_files:
        input_dir, input_relative = os.path.split(input_file)
        file_name, input_extension = os.path.splitext(input_relative)

        imgdir_relative = file_name + ".png"
        imgdir_file = os.path.join(idir_png, imgdir_relative)

        imgoutdir_relative = file_name + ".png"
        imgoutdir_file = os.path.join(odir_img, imgoutdir_relative)

        output_relative = file_name + oext
        output_file = os.path.join(odir_writer, output_relative)

        command = " pdal pipeline -i " + json_path + \
                  " --readers." + reader_name + ".filename=" + input_file + \
                  " --filters.drawpointpaironimage.image_file=" + imgdir_file + \
                  " --filters.drawpointpaironimage.image_outfile=" + imgoutdir_file + \
                  " --writers." + writer_name + ".filename=" + output_file + \
                  " " + options

        print(command)
        p = Process(target=execute_command, args=(command,))
        p.start()
        procssList.append(p)

    for pr in procssList:
        pr.join()


if __name__ == '__main__':
    json_path = "/home/zmm/practice/version2/json/drawpointpairOnimage.json"
    idir_ppair = "/home/zmm/practice/version2/json/drawPointpairs/XYZuv"
    idir_png = "/home/zmm/practice/version2/json/data/pic"

    odir_img = "/home/zmm/practice/version2/json/drawPointpairs/result"
    odir_writer = "/home/zmm/practice/version2/json/drawPointpairs/Err"

    pdal_batch_process(idir_png, idir_ppair, "text", ".txt", odir_img, odir_writer, "text", ".txt", json_path, "")
