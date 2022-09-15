import imageio

filenames = []
for i in range(1, 11):
    filename = "/home/zmm/practice/version2/json/pcdToimage/result/" + str(i) + ".png"
    filenames.append(filename)

gif_images = []
for path in filenames:
    gif_images.append(imageio.v2.imread(path))
imageio.mimsave("/home/zmm/practice/version2/json/pcdToimage/result/pcdToImage.gif", gif_images, fps=2)
