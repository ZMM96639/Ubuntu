//
// Created by zmm on 5/5/21.
//
#include <iostream>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>

using namespace std;
using namespace cv;

int main(int argc, char **argv) {
//    Mat girl = imread(argv[1]);
//    namedWindow("1picture", WINDOW_AUTOSIZE);
//    imshow("1picture", girl);

    Mat img = imread(argv[2]);
    Mat logo = imread(argv[3]);
//    namedWindow("2.jpg", WINDOW_AUTOSIZE);
//    imshow("2.jpg", img);
//
//    namedWindow("3logo");
//    imshow("3logo", logo);

    Mat imgROI;
//first way
    cout << "hello" << endl;
    imgROI = img(Rect(800, 350, logo.cols, logo.rows));
    cout << "hello" << endl;
//second way
//    imgROI = img(Range(350, 350 + logo.rows), Range(800, 800 + logo.cols));
    addWeighted(imgROI, 0.5, logo, 0.3, 0, imgROI);
    namedWindow("4picture");
    imshow("4picture", img);
    waitKey();

    return 0;
}
