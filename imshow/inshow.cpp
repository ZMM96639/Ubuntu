//
// Created by zmm on 5/5/21.
//
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
using namespace cv;

int main(int argc,char **argv){
    Mat girl = imread("/home/meimiao/Pictures/1.jpg");
    namedWindow("1picture");
    imshow("1picture",girl);
    Mat img = imread("/home/meimiao/Pictures/2.jpg",199);
    Mat logo = imread("2_logo.jpg");
    namedWindow("2.jpg");
    imshow("2.jpg",img);
    namedWindow("3logo");
    imshow("3logo", logo);
    Mat imgROI;
//first way
imgROI = image(Rect(800,350,logo.cols,logo.rows));
//second way
imgROI = image(Rang(350,350+logo.rows),Rang(800,800+logo.cols));
    addWeighted(imgROI,0.5,logo,0.3,0,imgROI);
    namedWindow("4picture");
    imshow("4picture",img);
    waitKey();
    return 0;

}