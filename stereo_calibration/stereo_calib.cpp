#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {
    string image_path = argv[1];
    vector<string> imageList[2];
    for (int i = 1; i <= 2; i++) {
        imageList[0].push_back(image_path + "left/" + to_string(i) + ".jpg");
        imageList[1].push_back(image_path + "right/" + to_string(i) + ".jpg");
    }

//    for (auto val:imageList[0]) { cout << val << endl; }

    //导入已标定结果
    vector<Mat> calib;
    calib.push_back((Mat_<float>(3, 3) <<
                                       3.2686099702554066e+02, 0, 6.3470432320051611e+02, 0, 3.2547238025065536e+02, 3.6356740662493496e+02, 0, 0, 1));
    calib.push_back((Mat_<float>(4, 1)
            << 7.4338510524433035e-02, -3.1460866361061567e-02, 2.9333195424307262e-02, -2.8386034907638838e-03));
    calib.push_back((Mat_<float>(3, 3) <<
                                       3.2721576451011396e+02, 0, 6.3913853333291217e+02, 0, 3.2602306388215783e+02, 3.6206074309792928e+02, 0, 0, 1));
    calib.push_back((Mat_<float>(4, 1)
            << 6.9703462974376143e-02, -1.8738660202294380e-02, 1.6583385365775284e-02, -4.0568375742832501e-03));

    Mat cameraMatrix[2], distCoeffs[2], R, T;
    cameraMatrix[0] = calib[0];
    distCoeffs[0] = calib[1];
    cameraMatrix[1] = calib[2];
    distCoeffs[1] = calib[3];

    int n_images = imageList[0].size();
    float squareSize = strtof("50", nullptr);

    int cols = strtol("7", nullptr, 10);
    int rows = strtol("6", nullptr, 10);
    Size boardSize(cols, rows), imageSize;
    vector<vector<Point2f> > imagePoints[2];
    vector<vector<Point3f> > objectPoints;
    objectPoints.resize(n_images);

//  世界坐标点
    for (int i = 0; i < n_images; i++) {
        for (int j = 0; j < boardSize.height; j++)
            for (int k = 0; k < boardSize.width; k++)
                objectPoints[i].push_back(Point3f(k * squareSize, j * squareSize, 0));
    }

    /*
        for (auto val:objectPoints) {
            cout << val << endl;
        }
    */

//    检测角点
    Mat img;
    for (int t = 0; t < 2; t++) {
        for (int m = 0; m < n_images; m++) {
            img = imread(imageList[t][m]);
            vector<Point2f> corners;
            cv::Mat gray, thresh_img;
            cvtColor(img, gray, COLOR_BGR2GRAY);
            cv::adaptiveThreshold(gray, thresh_img, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 121,
                                  20);//CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY

            bool patternfound = findChessboardCorners(thresh_img, boardSize, corners,
                                                      CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

            if (!patternfound) {
                cout << "Image: " << imageList[t][m] << " cannot be found any corners" << endl;
            }
            if (patternfound) {
                cornerSubPix(thresh_img, corners, cv::Size(9, 9), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

                /*
                //可视化
                cv::Mat imageTemp = img.clone();
                for (int num = 0; num < corners.size(); num++) {
                    circle(imageTemp, corners[num], 5, cv::Scalar(0, 0, 255), -1, 8, 0);
                }
                imshow("circle", imageTemp);
                waitKey(10);
                 */

                imagePoints[t].push_back(corners);
            }
        }
    }

    /*
    for (auto val:imagePoints[0]) {
        cout << val << endl;
    }
     */
    imageSize = img.size();
    double rms = fisheye::stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1], cameraMatrix[0],
                                          distCoeffs[0], cameraMatrix[1], distCoeffs[1], imageSize, R, T);

    cout << "Extrinsic error (RMS):" << rms << endl;
    cout << "Extrinsic Matrix:\nRotation Matrix:\n" << R << "\nTranslation Matrix:\n" << T << endl;

    Mat R1, R2, P1, P2, Q;
    fisheye::stereoRectify(cameraMatrix[0], distCoeffs[0], cameraMatrix[1], distCoeffs[1], imageSize,
                           R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY);

    cout << "R1:\n" << R1 << endl;
    cout << "R2:\n" << R2 << endl;
    cout << "P1:\n" << P1 << endl;
    cout << "P2:\n" << P2 << endl;
    cout << "Q:\n" << Q << endl;
    cout << "baseline: " << 1 / Q.at<double>(3, 2) << endl;

    Mat map[2][2];
    fisheye::initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, map[0][0], map[0][1]);
    fisheye::initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, map[1][0], map[1][1]);


    Mat imgLr, imgRr;
    Mat imgL = imread(imageList[0][0]);
    Mat imgR = imread(imageList[1][0]);

    imshow("imgL", imgL);
    imshow("imgR", imgR);

    remap(imgL, imgLr, map[0][0], map[0][1], INTER_AREA);//左校正
    remap(imgR, imgRr, map[1][0], map[1][1], INTER_AREA);//右校正

    imshow("imgLr", imgLr);
    imshow("imgRr", imgRr);

    int mindisparity = 1; //最小视差  搜索起点
    int ndisparities = 10;  //视差搜索范围
    int SADWindowSize = 0.1;   //SAD的窗口尺寸

    //SGBM
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(mindisparity, ndisparities, SADWindowSize);
    int L1 = 8 * imgLr.channels() * SADWindowSize * SADWindowSize;
    int L2 = 32 * imgRr.channels() * SADWindowSize * SADWindowSize;
    sgbm->setP1(L1);
    sgbm->setP2(L2);
    sgbm->setPreFilterCap(63); //剔除无误匹配参数
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleRange(32); // 视差连通条件
    sgbm->setSpeckleWindowSize(100); // 视差连续性中像素点的个数判断 判断噪点
    sgbm->setDisp12MaxDiff(1); //左右一致性检测最大容许误差阈值。int 类型
    Mat disp, disp_color;
    sgbm->compute(imgLr, imgRr, disp);
    Mat disp8U = Mat(disp.size(), CV_8UC3);
    disp.convertTo(disp8U, 255 / (512 * 16.));
    applyColorMap(disp8U, disp_color, cv::COLORMAP_JET);
    imshow("disparity", disp_color);
    waitKey(0);
    return 0;
}