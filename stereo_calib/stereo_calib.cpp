#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>


using namespace std;
using namespace cv;

//void insertDepth32f(cv::Mat& depth);

float fx = 718.856;
float baseline = 158.727;

int main(int argc, char *argv[]) {
    string image_path = argv[1];
    vector<string> imageList[2];
    for (int i = 1; i <= 2; i++) {
        imageList[0].push_back(image_path + "outimage_1/" + to_string(i) + ".jpg");
        imageList[1].push_back(image_path + "outimage_2/" + to_string(i) + ".jpg");
    }
    vector<Mat> calib;
    calib.push_back((Mat_<float>(3, 3) <<
    307.8753, 0.0000, 645.0605,
    0.0000, 308.9312, 362.3371,
    0, 0, 1));
    calib.push_back((Mat_<float>(4, 1) << 0.1031, -0.0522, 0.0686, -0.0229));
    calib.push_back((Mat_<float>(3, 3) <<
    318.5684, 0.0000, 639.1335,
    0.0000, 319.3746, 361.0389,
    0, 0, 1));
    calib.push_back((Mat_<float>(4, 1) << 0.0714, 0.0575, -0.1849, 0.1768));
    calib.push_back((Mat_<float>(3, 3) <<
    0.9835, -0.0058, 0.1810,
    0.0064, 1.0000, -0.0028,
    -0.1810, 0.0039, 0.9835));
    calib.push_back((Mat_<float>(3, 1) << -150.9182, -1.9951, 49.1336));

    Mat cameraMatrix[2], distCoeffs[2], rvecs[2], tvecs[2], R, T;
    cameraMatrix[0] = calib[0];
    distCoeffs[0] = calib[1];
    cameraMatrix[1] = calib[2];
    distCoeffs[1] = calib[3];
    /*
    R = calib[4];
    T = calib[5];
    */
    int n_images = imageList[0].size();
    float squareSize = strtof("50", nullptr);

    int cols = strtol("7", nullptr, 10);
    int rows = strtol("6", nullptr, 10);
    Size boardSize(cols, rows), imageSize;
    vector<vector<Point2f> > imagePoints[2];
    vector<vector<Point3f> > objectPoints;
    objectPoints.resize(n_images);

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

    Mat img;
    for (int t = 0; t < 2; t++) {
        for (int m = 0; m < n_images; m++) {
            img = imread(imageList[t][m]);
            /*
            imshow("img", img);
            waitKey(0);
             */
            vector<Point2f> corners;
            cv::Mat gray, thresh_img;
            cvtColor(img, gray, COLOR_BGR2GRAY);
            cv::adaptiveThreshold(gray, thresh_img, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 121,
                                  20);//CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY

                                  findChessboardCorners(thresh_img, boardSize, corners,
                                                        CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
                                  imagePoints[t].push_back(corners);

        }
    }
    /*
    for (auto val:imagePoints[0]) {
        cout << val << endl;
    }
     */
    imageSize = img.size();
    double rms = fisheye::stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1], cameraMatrix[0], distCoeffs[0],
                                          cameraMatrix[1], distCoeffs[1], imageSize, R, T);
    cout << "Extrinsic error (RMS):" << rms << endl;
    cout << "Extrinsic Matrix:\nRotation Matrix:\n" << R << "\nTranslation Matrix:\n" << T << endl;
}
//    Mat R1, R2, P1, P2, Q;
//    fisheye::stereoRectify(cameraMatrix[0], distCoeffs[0], cameraMatrix[1], distCoeffs[1], imageSize,
//                           R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, imageSize);
//    cout << "P1:\n" << P1 << endl;
//    cout << "P2:\n" << P2 << endl;
//    cout << "Q:\n" << Q << endl;
//    cout << "baseline: " << 1 / Q.at<double>(3, 2) << endl;
//
//    int mindisparity = 0; //最小视差  搜索起点
//    int ndisparities = 128;  //视差搜索范围
//    int SADWindowSize = 13;   //SAD的窗口尺寸
//SGBM
//    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(mindisparity, ndisparities, SADWindowSize);
//    int P1 = 4 * imgL.channels() * SADWindowSize * SADWindowSize;
//    int P2 = 32 * imgR.channels() * SADWindowSize * SADWindowSize;
//    sgbm->setP1(P1);
//    sgbm->setP2(P2);
//    sgbm->setPreFilterCap(63); //剔除无误匹配参数
//    sgbm->setUniquenessRatio(10);
//    sgbm->setSpeckleRange(32); // 视差连通条件
//    sgbm->setSpeckleWindowSize(100); // 视差连续性中像素点的个数判断 判断噪点
//    sgbm->setDisp12MaxDiff(1); //左右一致性检测最大容许误差阈值。int 类型
//    //sgbm->setMode(cv::StereoSGBM::MODE_HH);
//    cv::Mat disp32F;
//    sgbm->compute(imgL, imgR, disp32F);
//    disp32F.convertTo(disp32F, CV_32F, 1.0 / 16);
//    //float* inData = disp32F.ptr<float>(284);
//    //cout << float(inData[322]) << endl;
//    //insertDepth32f(disp32F);
//    Mat disp8U = Mat(disp32F.rows, disp32F.cols, CV_8UC1);
//    //normalize(disp8U, disp32F, 0, 255, NORM_MINMAX, CV_8UC1);
//    disp32F.convertTo(disp8U, CV_8UC1);
//    imshow("left", imgL);
//    imshow("right", imgR);
//    imshow("disparity", disp8U);
//    waitKey(0);
//    return 0;

//    cv::Mat depthMap = cv::Mat::zeros(disp32F.size(), CV_32FC1);
//    int height = disp32F.rows;
//    int width = disp32F.cols;
//    for(int k = 0;k < height; k++)
//    {
//        const float* inData = disp32F.ptr<float>(k);
//        float* outData = depthMap.ptr<float>(k);
//        for(int i = 0; i < width; i++)
//        {
//            if(!inData[i]) continue;
//            outData[i] = float(fx *baseline / inData[i]);
//        }
//    }
//    FileStorage fswrite("test.xml", FileStorage::WRITE);// 新建文件，覆盖掉已有文件
//    fswrite << "src1" << depthMap;
//    fswrite.release();
//    Mat depthMap8U = Mat(depthMap.rows, depthMap.cols, CV_8UC1);
//    normalize(depthMap, depthMap8U, 0, 255, NORM_MINMAX, CV_8U);
//    imshow("depth:8U",depthMap8U);
//    waitKey(0);
//    return 0;
//}
//
//void insertDepth32f(cv::Mat& depth)
//{
//    const int width = depth.cols;
//    const int height = depth.rows;
//    float* data = (float*)depth.data;
//    cv::Mat integralMap = cv::Mat::zeros(height, width, CV_64F);
//    cv::Mat ptsMap = cv::Mat::zeros(height, width, CV_32S);
//    double* integral = (double*)integralMap.data;
//    int* ptsIntegral = (int*)ptsMap.data;
//    memset(integral, 0, sizeof(double) * width * height);
//    memset(ptsIntegral, 0, sizeof(int) * width * height);
//    for (int i = 0; i < height; ++i)
//    {
//        int id1 = i * width;
//        for (int j = 0; j < width; ++j)
//        {
//            int id2 = id1 + j;
//            if (data[id2] > 1e-3)
//            {
//                integral[id2] = data[id2];
//                ptsIntegral[id2] = 1;
//            }
//        }
//    }
//    // 积分区间
//    for (int i = 0; i < height; ++i)
//    {
//        int id1 = i * width;
//        for (int j = 1; j < width; ++j)
//        {
//            int id2 = id1 + j;
//            integral[id2] += integral[id2 - 1];
//            ptsIntegral[id2] += ptsIntegral[id2 - 1];
//        }
//    }
//    for (int i = 1; i < height; ++i)
//    {
//        int id1 = i * width;
//        for (int j = 0; j < width; ++j)
//        {
//            int id2 = id1 + j;
//            integral[id2] += integral[id2 - width];
//            ptsIntegral[id2] += ptsIntegral[id2 - width];
//        }
//    }
//    int wnd;
//    double dWnd = 2;
//    while (dWnd > 1)
//    {
//        wnd = int(dWnd);
//        dWnd /= 2;
//        for (int i = 0; i < height; ++i)
//        {
//            int id1 = i * width;
//            for (int j = 0; j < width; ++j)
//            {
//                int id2 = id1 + j;
//                int left = j - wnd - 1;
//                int right = j + wnd;
//                int top = i - wnd - 1;
//                int bot = i + wnd;
//                left = max(0, left);
//                right = min(right, width - 1);
//                top = max(0, top);
//                bot = min(bot, height - 1);
//                int dx = right - left;
//                int dy = (bot - top) * width;
//                int idLeftTop = top * width + left;
//                int idRightTop = idLeftTop + dx;
//                int idLeftBot = idLeftTop + dy;
//                int idRightBot = idLeftBot + dx;
//                int ptsCnt = ptsIntegral[idRightBot] + ptsIntegral[idLeftTop] - (ptsIntegral[idLeftBot] + ptsIntegral[idRightTop]);
//                double sumGray = integral[idRightBot] + integral[idLeftTop] - (integral[idLeftBot] + integral[idRightTop]);
//                if (ptsCnt <= 0)
//                {
//                    continue;
//                }
//                data[id2] = float(sumGray / ptsCnt);
//            }
//        }
//        int s = wnd / 2 * 2 + 1;
//        if (s > 201)
//        {
//            s = 201;
//        }
//        //cv::GaussianBlur(depth, depth, cv::Size(s, s), s, s);
//    }

