//
// Created by zmm on 5/5/21.
//
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
using namespace cv;

int main(int argc,char **argv){
    Mat girl = imread("1.jpg");
    namedWindow("1picture");
    imshow("1picture",girl);
    Mat img = imread("2.jpg",199);
    Mat logo = imread("2_logo.jpg");
    namedWindow("2.jpg");
    imshow("2.jpg",img);
    namedWindow("3logo");
    imshow("3logo", logo);
    Mat imgeROI;
}