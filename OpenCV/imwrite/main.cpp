#include <iostream>
#include <opencv4/opencv2/highgui.hpp>
#include <vector>
#include "lib.h"

using namespace cv;
using namespace std;

int main(int argc, char **argv) {

    Mat mat(480, 640, CV_8UC4);
    createAlphaMat(mat);
    vector<int> compression_params;
    compression_params.push_back(IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
    try {
        imwrite("Alpg = ha.png", mat, compression_params);
        imshow("PNG", mat);
        fprintf(stdout, "picture");
        waitKey(0);
    }
    catch (runtime_error &ex) {
        fprintf(stderr, "Trans_error: %s\n", ex.what());
        return 1;
    }
    return 0;
}
