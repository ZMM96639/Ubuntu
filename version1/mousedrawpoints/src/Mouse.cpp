#include "Mouse.h"
#include <iostream>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

// vector<cv::Point> Mouse::ppoints;// comment the code will happen error.

void Mouse::onMouse(int event, int x, int y, int flags, void *param)
{
    Mat *im = reinterpret_cast<Mat *>(param);
    switch (event)
    {
    case 1:
        std::cout << "at(" << x << "," << y << ")value is:"
                  << static_cast<int>(im->at<uchar>(cv::Point(x, y))) << std::endl;

        ppoints.push_back(cv::Point(x, y));

        break;

    case 2:
        std::cout << "input(x,y)" << endl;
        std::cout << "x =" << endl;
        cin >> x;
        std::cout << "y =" << endl;
        cin >> y;
        std::cout << "at(" << x << "," << y << ")value is:"
                  << static_cast<int>(im->at<uchar>(cv::Point(x, y))) << std::endl;
        break;
    }
}