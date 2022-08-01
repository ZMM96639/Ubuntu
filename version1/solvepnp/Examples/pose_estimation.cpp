#include <iostream>
#include <fstream>
#include <opencv2/calib3d/calib3d.hpp>

#include "fileLoad.h"
#include "BundleAdjustment.hpp"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    string path = "/home/zmm/practice/solvepnp/data";

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
    pointPath[2] = path + "intrinsic.xml";

    vector<double> K_elem;
    vector<Point2d> pixelPoints;
    vector<Point3d> worldPoints;

    pixelPoints.reserve(44);
    worldPoints.reserve(44);

    Fileload::dataLoad(pointPath[0], pixelPoints);
    Fileload::dataLoad(pointPath[1], worldPoints);

    /*     cout << "ppoints:" << endl;
        cout << pixelPoints << endl;

        cout << "wpoints:" << endl;
        cout << worldPoints << endl;

        Fileload::dataLoad(pointPath[2], K_elem);

        Mat K = (Mat_<float>(3, 3) << K_elem[0], K_elem[1], K_elem[2],
                    K_elem[3], K_elem[4], K_elem[5],
                    K_elem[6], K_elem[7], K_elem[8]);
     */

    Mat K = (Mat_<double>(3, 3) << 1463.712, 0, 633.992, 0, 1465.228, 422.927, 0, 0, 1);

    // Mat R[11], r[11], t[11];
    Mat R, r, t;

    string outpath = path + "result5.txt";
    fstream fout(outpath, ios::out);

    // solvePnP(worldPoints, pixelPoints, K, Mat(), r, t, false, SOLVEPNP_ITERATIVE);
    solvePnPRansac(worldPoints, pixelPoints, K, Mat(), r, t, false);

    cv::Rodrigues(r, R);

    BundleAdjustment::VecVector3d pt3d;
    BundleAdjustment::VecVector2d pt2d;

    for (int i = 0; i < worldPoints.size(); ++i)
    {
        pt3d.push_back(Eigen::Vector3d(worldPoints[i].x, worldPoints[i].y, worldPoints[i].z));
        pt2d.push_back(Eigen::Vector3d(pixelPoints[i].x, pixelPoints[i].y));
    }

    Sophus::SE3d pose;
    BundleAdjustment::optimilizationUsingG2O(pt3d, pt2d, K, pose, 10);

    // cout << "R:"<< endl;
    // cout << R << endl;

    // cout << "r: " << r << endl;
    // cout << "t: " << t << endl;

    // fout << "R:" << endl;

    // for (int r = 0; r < 3; r++)
    // {
    //     for (int c = 0; c < 3; c++)
    //     {
    //         fout << R.at<double>(r, c) << " ";
    //     }
    //     fout << endl;
    // }

    // fout << "r:" << endl;
    // for (int n_r = 0; n_r < 3; n_r++)
    // {
    //     fout << r.at<double>(n_r) << " ";
    // }
    // fout << endl;

    // fout << "t: " << endl;
    // for (int n_t = 0; n_t < 3; n_t++)
    // {
    //     fout << t.at<double>(n_t) << " ";
    // }
    // fout << endl;

    return 0;
}