#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <sophus/se3.hpp>
#include <chrono>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    assert(argc > 1);
    string path = argv[1];

    if (path.back() != '/')
    {
        path.append("/");
    }

    string pointPath[2];
    for (int i = 0; i < 2; i++)
    {
        pointPath[i] = path + "point.txt";
    }

    vector<vector<Point3d>> wp;
    vector<vector<Point2d>> cp;

    
    Mat K;

    Mat r, t;
    solvePnP(wp, cp, K, Mat(), r, t, false); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
    Mat R;
    cv::Rodrigues(r, R); // r为旋转向量形式，用Rodrigues公式转换为矩阵
    cout << "R =" << endl
         << R << endl;
    cout << "t =" << endl
         << t << endl;
}