#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>

using namespace cv;
int main(int argc, char **argv){
    Mat scrImg = imread(argv[2], 0);
    namedWindow("1.picture",WINDOW_KEEPRATIO);
    imshow("1.picture", scrImg);
    Mat cornerStrength;
    cornerHarris(scrImg, cornerStrength,2,3,0.01);
    Mat harrisCorner;
    threshold(cornerStrength, harrisCorner,0.0000001 ,255,THRESH_BINARY);
    namedWindow("2.picture",WINDOW_KEEPRATIO);
    imshow("2.picture", harrisCorner);
    waitKey(0);
    return 0;
}