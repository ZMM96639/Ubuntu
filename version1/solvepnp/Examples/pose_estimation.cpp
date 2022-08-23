#include <iostream>
#include <fstream>
#include <opencv2/calib3d/calib3d.hpp>

#include "fileLoad.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    string path = "/home/zmm/practice/version1/solvepnp/data";

    if (path.back() != '/')
    {
        path.append("/");
    }

    string pointPath[3];

    for (int i = 0; i < 2; i++)
    {
        if (i == 0)
        {
            pointPath[i] = path + "pppoints.txt";
        }
        else
        {
            pointPath[i] = path + "wpoints.txt";
        }
    }

    vector<double> K_elem;
    vector<Point2d> pixelPoints;
    vector<Point3d> worldPoints;

    pixelPoints.reserve(44);
    worldPoints.reserve(44);

    Fileload::dataLoad(pointPath[0], pixelPoints);
    Fileload::dataLoad(pointPath[1], worldPoints);

    Mat K = (Mat_<double>(3, 3) << 1463.712, 0, 633.992, 0, 1465.228, 422.927, 0, 0, 1);

    Mat R, r, t;

    string outpath = path + "result.txt";
    fstream fout(outpath, ios::out);

    // solvePnP(worldPoints, pixelPoints, K, Mat(), r, t, false, SOLVEPNP_ITERATIVE);
    solvePnPRansac(worldPoints, pixelPoints, K, Mat(), r, t, false);

    cv::Rodrigues(r, R);

    fout << "Rotation:" << endl;
    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < 3; c++)
        {
            fout << R.at<double>(r, c) << " ";
        }
        fout << endl;
    }

    fout << "rve:" << endl;
    for (int n_r = 0; n_r < 3; n_r++)
    {
        fout << r.at<double>(n_r) << " ";
    }
    fout << endl;

    fout << "tve: " << endl;
    for (int n_t = 0; n_t < 3; n_t++)
    {
        fout << t.at<double>(n_t) << " ";
    }
    fout << endl;

    return 0;
}