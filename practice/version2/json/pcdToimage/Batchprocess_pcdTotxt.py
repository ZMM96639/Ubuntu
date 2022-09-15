# -*- coding: utf-8 -*-
import os
import subprocess


def get_suffix_files_in_dir(idir, ext):
    result_files = []
    for relative_path in os.listdir(idir):
        name, extension = os.path.splitext(relative_path)
        if extension == ext:
            result_file = os.path.join(idir, relative_path)
            result_files.append(result_file)

    # print("/*************/")
    # print(result_files)
    # print("/*************/")
    # print(names)
    # print("/*************/")

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
def pdal_batch_process(idir, reader_name, iext, odir, writer_name, oext, json_path, options):
    input_files = get_suffix_files_in_dir(idir, iext)

    for input_file in input_files:
        input_dir, input_relative = os.path.split(input_file)
        file_name, input_extension = os.path.splitext(input_relative)

        output_relative = file_name + oext
        output_file = os.path.join(odir, output_relative)

        print(input_file)
        print(output_file)
        command = " pdal pipeline -i " + json_path + " --readers." + reader_name + \
                  ".filename=" + input_file + " --writers." + writer_name + ".filename=" + \
                  output_file + " " + options

        print(command)
        execute_command(command)


if __name__ == '__main__':
    json_path = "/home/zmm/practice/version2/json/pcdTotxt.json"
    idir = "/home/zmm/practice/version2/json/pcdToimage/origin"
    odir = "/home/zmm/practice/version2/json/pcdToimage/pointsCloud"

    # oext must be equal to the suffix defined in the json_path file
    pdal_batch_process(idir, "pcd", ".pcd", odir, "text", ".txt", json_path, "")
