#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <istream>

using namespace cv;
using namespace std;
#define WINDOW_NAME1 "program1"
#define WINDOW_NAME2 "program2"
Mat g_srcImg, g_srcImg1, g_grayImg;
int thresh = 30;
int max_thresh = 175;

void on_CornerHarris(int, void *);

int main(int argc, char **argv) {
    cout << "hello" << endl;
    g_srcImg = imread(argv[1], 1);

    if (!g_srcImg.data) {
        print("error, please check out the path");
        return false;
    }
    imshow("origin picture", g_srcImg);
    g_srcImg1 = g_srcImg.clone();
    cvtColor(g_srcImg1, g_grayImg, COLOR_BGR2GRAY);
    namedWindow(WINDOW_NAME1, WINDOW_AUTOSIZE);
    createTrackbar("thresh:", WINDOW_NAME1, &thresh, max_thresh, on_CornerHarris);
    on_CornerHarris(0, 0);
    waitKey(0);
    return 0;
}

void on_CornerHarris(int, void *) {
    Mat dstImg;
    Mat normImg;
    Mat scaledImg;
    dstImg = Mat::zeros(g_srcImg.size(), CV_32FC1);
    g_srcImg1 = g_srcImg.clone();
    cornerHarris(g_grayImg, dstImg, 2, 3, 0.04, BORDER_DEFAULT);
    normalize(dstImg, normImg, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
    convertScaleAbs(normImg, scaledImg);
    for (int j = 0; j < normImg.rows; j++) {
        for (int i = 0; i < normImg.cols; i++) {
            if ((int) normImg.at<float>(j, i) > thresh + 80) {
                circle(g_srcImg1, Point(i, j), 5, Scalar(10, 10, 255), 2, 8, 0);
                circle(scaledImg, Point(i, j), 5, Scalar(0, 10, 255), 2, 8, 0);
            }
        }
    }
    imshow(WINDOW_NAME1, g_srcImg1);
    imshow(WINDOW_NAME2, scaledImg);
}