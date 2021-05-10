#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>

using namespace cv;
int main(int argc, char **argv){
    Mat scrImg = imread(argv[1], 0);
    imshow("1.picture", scrImg);
    Mat cornerStrength;
    cornerHarris(scrImg, cornerStrength,2,3,0.01);
    Mat harrisCorner;
    threshold(cornerStrength, harrisCorner,0.00001,255,THRESH_BINARY);
    imshow("2.picture", harrisCorner);
    waitKey(0);
    return 0;
}