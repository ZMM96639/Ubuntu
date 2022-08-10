import os

import pdal

# embed in main will happen error
# if __name__ == '__main_':

path = "/home/zmm/practice/version2/json/drawPointpairs/Err"

array = []
for num in range(1, 12):
    file = str(num) + ".txt"
    filepath = os.path.join(path, file)
    pipeline = pdal.Reader(filename=filepath).pipeline()
    pipeline.execute()
    arr = pipeline.arrays[0]
    array.append(arr)

print(array)


