#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <opencv2/calib3d/calib3d.hpp>
#include <eigen3/Eigen/Core>
#include <vector>
#include <pangolin/pangolin.h>
#include <unistd.h>
#include <stdlib.h>
using namespace std;
using namespace Eigen;
using namespace cv;

cv::Mat image1,image2;
cv::Mat disparity_SGBM,disparity;
cv::Ptr<cv::StereoSGBM> sgbm=cv::StereoSGBM::create(0,96,9,8*9*9,32*9*9,1,63,10,100,32);
static void SGBMCallback(int pose,void* data);
void SGBMStart();
void showPointCloud(const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud);

int main(int argc, char **argv)
{
    cv::Mat image_left,image_right;
    image_left=cv::imread(argv[1],cv::IMREAD_UNCHANGED);
    image_right=cv::imread(argv[2],cv::IMREAD_UNCHANGED);
    if (image_left.data==nullptr){
        std::cout << "file" << argv[1]<<"Failed to load image"<< std::endl;
        return 1;
    }
    if (image_right.data==nullptr){
        std::cout << "file" << argv[2]<<"Failed to load image"<< std::endl;
        return 1;
    }
    cv::FileStorage fsSettings(argv[3],cv::FileStorage::READ);   //read EoRoC.yaml and get undistort image.
    if(!fsSettings.isOpened()){
        std::cout<<"ERROR: Wrong path to settings"<<endl;
        return -1;
    }
    cv::Mat K_l,K_r,P_l,P_r,R_l,R_r,D_l,D_r;
    fsSettings["LEFT.K"]>>K_l;
    fsSettings["RIGHT.K"] >> K_r;
    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;
    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;
    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;
    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];
    double fx=fsSettings["Camera.fx"];
    double fy=fsSettings["Camera.fy"];
    double cx=fsSettings["Camera.cx"];
    double cy=fsSettings["Camera.cy"];
    double b=fsSettings["Camera.bf"];
    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() || rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return -1;
    }
    cv::Mat M1l,M2l,M1r,M2r;
    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
    cv::Mat imLeftRect,imRightRect;
    cv::remap(image_left,imLeftRect,M1l,M2l,cv::INTER_LINEAR);
    cv::remap(image_right,imRightRect,M1r,M2r,cv::INTER_LINEAR);
    //cv::imshow("undistort_left",imLeftRect);
    //cv::imshow("undistort_right",imRightRect);          //show the undistort image(imLeftRect,imRightRect).
    //cv::waitKey(0);
    image1=imLeftRect.clone();
    image2=imRightRect.clone();
    cv::namedWindow("SGBM_disparity");
    SGBMStart();
    int flag=1;
    while (flag!=32)
    {
        sgbm->compute(image1,image2,disparity_SGBM);
        disparity_SGBM.convertTo(disparity,CV_32F,1.0/16.0f);
        cv::imshow("SGBM_disparity",disparity/96.0);
        flag=cv::waitKey(200);
    }
    cv::imshow("disparity",disparity/96.0);
    cv::waitKey(0);

    vector<Vector4d,Eigen::aligned_allocator<Vector4d>> pointcloud;

    for (int v=0;v<imLeftRect.rows;v++)
        for (int u=0;u<imLeftRect.cols;u++)
        {
            if (disparity.at<float>(v, u) <= 0.0 || disparity.at<float>(v, u) >= 96.0) continue;

            Vector4d point(0,0,0,imLeftRect.at<uchar>(v,u)/255.0);
            double x=(u-cx)/fx;
            double y=(v-cy)/fy;
            double depth = fx * b / (disparity.at<float>(v, u));
            point[0] = x * depth;
            point[1] = y * depth;
            point[2] = depth;
            pointcloud.push_back(point);
        }
    showPointCloud(pointcloud);
        return 0;
}

void showPointCloud(const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud)
{
    if (pointcloud.empty())
    {
        cerr << "Point cloud is empty!" << endl;
        return;
    }
    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
            );
    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: pointcloud) {
            glColor3f(p[3], p[3], p[3]);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}

void SGBMCallback(int pose,void* data)
{
    int SGBMnum=2;
    int blocksize=cv::getTrackbarPos("blocksize","SGBM_disparity");
    if (blocksize%2==0)
    {
        blocksize=blocksize+1;
    }
    if (blocksize<=5)
    {
        blocksize=5;
    }

    int numdisparity=cv::getTrackbarPos("numdisparity","SGBM_disparity");
    if (numdisparity%16!=0)
    {
        numdisparity=(numdisparity/16+1)*16;
    }

    sgbm->setBlockSize(blocksize);
    sgbm->setNumDisparities(numdisparity);
    sgbm->setSpeckleWindowSize(cv::getTrackbarPos("speckleWindowSize","SGBM_disparity"));
    sgbm->setSpeckleRange(cv::getTrackbarPos("speckleRange","SGBM_disparity"));
    sgbm->setUniquenessRatio(cv::getTrackbarPos("uniquenessRatio","SGBM_disparity"));
    sgbm->setDisp12MaxDiff(cv::getTrackbarPos("disp12MaxDiff","SGBM_disparity"));
    //sgbm->setpreFilterCap(cv::getTrackbarPos("preFilterCap","SGBM_disparity"));
    sgbm->setP1(600);
    sgbm->setP2(2400);
    sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
}
void SGBMStart()
{
    int minDisparity=0;
    int SGBMnum=2;
    int numdisparity=SGBMnum*16;
    int blocksize=5;
    int P1=600;
    int P2=2400;
    int disp12MaxDiff=1;
    int preFilterCap=63;
    int uniquenessRatio=6;
    int speckleWindowSize=60;
    int speckleRange=32;
    cv::createTrackbar("blocksize","SGBM_disparity",&blocksize,21,SGBMCallback);
    cv::createTrackbar("numdisparity","SGBM_disparity",&numdisparity,112,SGBMCallback);
    cv::createTrackbar("disp12MaxDiff","SGBM_disparity",&disp12MaxDiff,200,SGBMCallback);
    cv::createTrackbar("uniquenessRatio","SGBM_disparity",&uniquenessRatio,50,SGBMCallback);
    //cv::createTrackbar("preFilterCap","SGBM_disparity",&preFilterCap,64,SGBMCallback);
    cv::createTrackbar("speckleWindowSize","SGBM_disparity",&speckleWindowSize,200,SGBMCallback);
    cv::createTrackbar("speckleRange","SGBM_disparity",&speckleRange,40,SGBMCallback);
    SGBMCallback(blocksize,0);
    SGBMCallback(numdisparity,0);
    SGBMCallback(disp12MaxDiff,0);
    SGBMCallback(uniquenessRatio,0);
    SGBMCallback(speckleWindowSize,0);
    SGBMCallback(speckleRange,0);
    //SGBMCallback(preFilterCap,0);
}
