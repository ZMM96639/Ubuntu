#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace std;
using namespace cv;
#define WINDOW_NAME "[Shi_Timasi]"
Mat g_srcImg, g_grayImg;
int g_maxCornerNumber = 33;
int g_maxTrackbarNumber = 520;
RNG g_rng(12345);

void on_GoodFeaturesToTrack(int, void *){
    if( g_maxCornerNumber < 1){
        g_maxCornerNumber = 1;
    }
    vector<Point2f> corners;
    double qualityLevel = 0.01;
    double minDistance = 10;
    int blockSize = 3;
    double k = 0.04;
    Mat copy = g_srcImg.clone();
    goodFeaturesToTrack( g_grayImg,corners,g_maxCornerNumber,qualityLevel,minDistance,Mat(),blockSize, false,k);
    cout << "> the number of detect: "<<corners.size()<<endl;
    int r = 4;
    for(unsigned int i = 0; i < corners.size();i++){
        circle(copy,corners[i],r,Scalar(g_rng.uniform(0,255),g_rng.uniform(0,255),g_rng.uniform(0,255)),-1,8,0);
    }
    imshow(WINDOW_NAME,copy);
}
int main(int argc, char **argv){
    g_srcImg = imread(argv[1],1);
    cvtColor(g_srcImg,g_grayImg,COLOR_BGR2GRAY);
    namedWindow(WINDOW_NAME,WINDOW_KEEPRATIO);
    createTrackbar("max corners :",WINDOW_NAME,&g_maxCornerNumber,g_maxTrackbarNumber,on_GoodFeaturesToTrack);
    on_GoodFeaturesToTrack(0,0);
    waitKey(0);
    return 0;
}