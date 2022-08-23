import imageio

filenames = []
for i in range(1, 11):
    filename = "/home/zmm/practice/version2/json/drawPointpairs/result/" + str(i) + ".png"
    filenames.append(filename)

gif_images = []
for path in filenames:
    gif_images.append(imageio.v2.imread(path))
imageio.mimsave("/home/zmm/practice/version2/json/drawPointpairs/result/drawpointpairs.gif", gif_images, fps=2)
