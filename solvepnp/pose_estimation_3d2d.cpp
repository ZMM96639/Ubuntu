#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <sophus/se3.hpp>
#include <chrono>

using namespace std;
using namespace cv;

int main(int argc, char **argv) {
    vector<Point3f> pts_3d, camera;
    vector<Point2f> pts_2d;
    int i = 0, k = 0, l = 0;
    int Arsize = 702;
    ifstream infile_1, infile_2, infile_3;
    string s, m, h;
    vector<string> v1, v2, v3;
    double data_1[Arsize][3], data_2[Arsize][2], data_3[3][3];

    infile_1.open(argv[1]);
    if (!infile_1.is_open()) {
        cout << "please check out the path of file" << endl;
    }

    //3D
    while (getline(infile_1, s)) {
        v1.push_back(s);
    }
    cout << "3D_coordinary:\n";
    for (; i < Arsize; i++) {
        string tmp;
        tmp = accumulate(v1[i].begin(), v1[i].end(), tmp);
        char *s_input = (char *) tmp.c_str();
        const char *split = ",";
        char *p = strtok(s_input, split);
        double a;
        int j = 0;
        while (p != NULL) {
            sscanf(p, "%lf", &a);
            p = strtok(NULL, split);
            data_1[i][j] = a;
            j++;
        }
        pts_3d.push_back(Point3f(data_1[i][0], data_1[i][1], data_1[i][2]));
    }
    cout << pts_3d << endl;

    //pixel
    infile_2.open(argv[2]);
    if (!infile_2.is_open()) {
        cout << "please check out the path of file" << endl;
    }
    while (getline(infile_2, m)) {
        v2.push_back(m);
    }
    cout << "\npixel_coordinary:\n";
    for (; k < Arsize; k++) {
        string tmp;
        tmp = accumulate(v2[k].begin(), v2[k].end(), tmp);
        char *s_input = (char *) tmp.c_str();
        const char *split = ",";
        char *p = strtok(s_input, split);
        double a;
        int j = 0;
        while (p != NULL) {
            sscanf(p, "%lf", &a);
            p = strtok(NULL, split);
            data_2[k][j] = a;
            j++;
        }
        pts_2d.push_back(Point2f(data_2[k][0], data_2[k][1]));
    }
    cout << pts_2d << endl;

    infile_3.open(argv[3]);
    if (!infile_3.is_open()) {
        cout << "please check out the path of file" << endl;
    }

    //im
    while (getline(infile_3, h)) {
        v3.push_back(h);
    }
    for (; l < 3; l++) {
        string tmp;
        tmp = accumulate(v3[l].begin(), v3[l].end(), tmp);
        char *s_input = (char *) tmp.c_str();
        const char *split = ",";
        char *p = strtok(s_input, split);
        double a;
        int j = 0;
        while (p != NULL) {
            sscanf(p, "%lf", &a);
            p = strtok(NULL, split);
            data_3[l][j] = a;
            j++;
        }
    }

    Mat K = (Mat_<double>(3, 3)
            << data_3[0][0], data_3[0][1], data_3[0][2], data_3[1][0], data_3[1][1], data_3[1][2], 0, 0, 1);
    Mat r, t;
    solvePnP(pts_3d, pts_2d, K, Mat(), r, t, false); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
    Mat R;
    cv::Rodrigues(r, R); // r为旋转向量形式，用Rodrigues公式转换为矩阵
    cout << "R=" << endl << R << endl;
    cout << "t=" << endl << t << endl;
}