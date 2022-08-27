#include <iostream>
#include <fstream>

#include "BundleAdjustment.hpp"
#include "Loadfile.hpp"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    string path = "/home/zmm/VScodeProjects/study/BA/camra_extrinsic/data";
    if (path.back() != '/')
    {
        path.append("/");
    }

    string filepath[2];

    for (int i = 0; i < 2; ++i)
    {
        if (i == 0)
            filepath[i] = path + "3D/scene1.txt";
        else
            filepath[i] = path + "pixel/scene1.txt";
    }

    vector<Point3d> pts_3d;
    vector<Point2d> pts_2d;

    Loadfile::dataImput(filepath[0], pts_3d);

    Loadfile::dataImput(filepath[1], pts_2d);


    // cout << pts_3d.size() << endl;
    // cout << pts_2d.size() << endl;


    // for (auto &val : pts_3d)
    // {
    //     cout << val << endl;
    // }

    // cout << "/***************/" << endl;
    // for (auto &val : pts_2d)
    // {
    //     cout << val << endl;
    // }


    Mat K = (Mat_<double>(3, 3) << 2864.389051, 0.132770, 1306.081919, 0, 2864.682667, 1042.003042, 0, 0, 1);

    BundleAdjustment::VecVector3d pts_3d_eigen;
    BundleAdjustment::VecVector2d pts_2d_eigen;

    for (size_t n = 0; n < pts_3d.size(); ++n)
    {
        pts_3d_eigen.push_back(Eigen::Vector3d(pts_3d[n].x, pts_3d[n].y, pts_3d[n].z));
        pts_2d_eigen.push_back(Eigen::Vector2d(pts_2d[n].x, pts_2d[n].y));
    }

    cout << "calling bundle adjustment by gauss newton" << endl;

    Sophus::SE3d pose_gn;
    BundleAdjustment::bundleAdjustmentGaussNewton(pts_3d_eigen, pts_2d_eigen, K, pose_gn);

    cout << "calling bundle adjustment by g2o" << endl;

    Sophus::SE3d pose_g2o;
    BundleAdjustment::bundleAdjustmentG2O(pts_3d_eigen, pts_2d_eigen, K, pose_g2o);

    return 0;
}
