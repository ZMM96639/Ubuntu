//
// Created by zmm on 2021/5/1.
#include <opencv4/opencv2/highgui.hpp>
#include <climits>

using namespace cv;

void createAlphaMat(Mat &mat) {
    for (int i = 0; i < mat.rows; ++i) {
        for (int j = 0; j < mat.cols; ++j) {
            Vec4b &rgba = mat.at<Vec4b>(i, j);
            rgba[0] = UCHAR_MAX;
            rgba[1] = saturate_cast<u_char>((float(mat.cols - j)) / ((float) mat.cols) * UCHAR_MAX);
            rgba[2] = saturate_cast<u_char>((float(mat.rows - i)) / ((float) mat.rows) * UCHAR_MAX);
            rgba[3] = saturate_cast<u_char>(0.5 * (rgba[1] + rgba[2]));
        }
    }
}


