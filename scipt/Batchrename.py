import os


def rename_file(path):
    fileList = os.listdir(path)

    for file in fileList:
        used_filename, extension = os.path.splitext(file)
        new_file = "pitch_roll_yaw_thetax_thetay_thetaz_" + used_filename + extension

        file = path + file
        new_file = path + new_file
        os.rename(file, new_file)


if __name__ == '__main__':
    path = os.getcwd()
    filepath = os.path.join(path, "result/")
    print(filepath)
    rename_file(filepath)
