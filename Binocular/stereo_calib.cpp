#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {

    assert(argc > 1);
    string image_path = argv[1];
    vector<string> imageList[2];

    for (int i = 1; i <= 44; i++) {
        imageList[0].push_back(image_path + "left/" + to_string(i) + ".jpg");
        imageList[1].push_back(image_path + "right/" + to_string(i) + ".jpg");
    }

//    for (auto val:imageList[1]) { cout << val << endl; }

    int n_images = imageList[0].size();
    int squareSize = 50; //Square length 50mm
    Size boardSize = Size(7, 6);  // Chessboard Size (7, 6)

    Size imageSize;
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
    cout << "delimiter" << endl;
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

            bool patternFound = findChessboardCorners(thresh_img, boardSize, corners,
                                                      CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

            if (!patternFound) {
                cout << "Image: " << imageList[t][m] << " cannot be found any corners" << endl;
            }
            if (patternFound) {
                cornerSubPix(thresh_img, corners, cv::Size(9, 9), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

/*
                //可视化
                cv::Mat imageTemp = img.clone();
                for (int num = 0; num < corners.size(); num++) {
                    circle(imageTemp, corners[num], 5, cv::Scalar(0, 0, 255), -1, 8, 0);
                }
                imshow("circle", imageTemp);
                waitKey(0);
*/
                imagePoints[t].push_back(corners);
            }
        }
    }

    int flags = 0;
    vector<Mat> calib;
    imageSize = img.size();
    vector<Vec3d> rotation_vectors[2], translation_vectors[2];
    Mat cameraMatrix[2], disCoeffs[2], R, T;

    flags |= cv::fisheye::CALIB_USE_INTRINSIC_GUESS;
    flags |= fisheye::CALIB_RECOMPUTE_EXTRINSIC;
    flags |= fisheye::CALIB_CHECK_COND;
    flags |= cv::fisheye::CALIB_FIX_SKEW;
    flags |= cv::fisheye::CALIB_FIX_PRINCIPAL_POINT;

/*
    for (auto val:objectPoints) {
        cout << val << endl;
    }
    cout << "delimiter" << endl;
    for (auto val2:imagePoints[0]) {
        cout << val2 << endl;
    }
*/
//导入已标定结果

    calib.push_back((Mat_<float>(3, 3) <<
                                       3.0865261620931369e+02, 0, 6.4383621266107968e+02, 0, 3.0732649800176790e+02, 3.5931135783971865e+02, 0, 0, 1));
    calib.push_back((Mat_<float>(4, 1)
            << 9.4044237923283328e-02, -4.1811048590483469e-03, 6.3758560334369937e-03, -2.6516749249909989e-03));

    for (int p = 0; p < 2; p++) {

        cameraMatrix[p] = calib[0];
        disCoeffs[p] = calib[1];

        fisheye::calibrate(objectPoints, imagePoints[p], imageSize, cameraMatrix[p], disCoeffs[p], rotation_vectors[p],
                           translation_vectors[p], flags, cv::TermCriteria(3, 100, 1e-6));
        /*
        FileStorage fs(image_path + "Intrinsic" + to_string(p) + ".xml", 1);
        fs << "Intrinsic_Matrix" << "[";
        for (size_t r = 0; r < 3; r++) {
            for (size_t c = 0; c < 3; c++) {
                fs << cameraMatrix[p].at<double>(r, c);
            }
        }
        fs << "]";
        fs << "discoeffs" << "[";
        for (size_t c = 0; c < 4; c++) {
            fs << disCoeffs[p].at<double>(0, c);
        }
        fs << "]";
        fs.release();
         */
    }

    cout << "Intrinsic and Distortion Matrix: " << endl;
    cout << cameraMatrix[0] << endl;
    cout << disCoeffs[0] << endl;
    cout << cameraMatrix[1] << endl;
    cout << disCoeffs[1] << endl;
    cout << "delimiter" << endl;
    cout << cameraMatrix[0].at<double>(0) << endl;
    cout << disCoeffs[0].at<double>(0, 0) << endl;

    /*
    for (auto val1: rotation_vectors[k]) { cout << val1 << endl; }
    for (auto val2: translation_vectors[k]) { cout << val2 << endl; }

}*/
/*
    double rms = fisheye::stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1], cameraMatrix[0],
                                          disCoeffs[0], cameraMatrix[1], disCoeffs[1], imageSize, R, T);

    cout << "Extrinsic error (RMS):" << rms << endl;
    cout << "Extrinsic Matrix:\nRotation Matrix:\n" << R << "\nTranslation Matrix:\n" << T << endl;

    Mat R1, R2, P1, P2, Q;
    fisheye::stereoRectify(cameraMatrix[0], disCoeffs[0], cameraMatrix[1], disCoeffs[1], imageSize,
                           R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY);

    cout << "R1:\n" << R1 << endl;
    cout << "R2:\n" << R2 << endl;
    cout << "P1:\n" << P1 << endl;
    cout << "P2:\n" << P2 << endl;
    cout << "Q:\n" << Q << endl;
    cout << "baseline: " << 1 / Q.at<double>(3, 2) << endl;

    Mat map[2][2];
    fisheye::initUndistortRectifyMap(cameraMatrix[0], disCoeffs[0], R1, P1, imageSize, CV_16SC2, map[0][0], map[0][1]);
    fisheye::initUndistortRectifyMap(cameraMatrix[1], disCoeffs[1], R2, P2, imageSize, CV_16SC2, map[1][0], map[1][1]);
    Mat imgLr, imgRr;

    Mat imgL = imread(imageList[0][0]);
    Mat imgR = imread(imageList[1][0]);


    imshow("imgL", imgL);
    imshow("imgR", imgR);


    remap(imgL, imgLr, map[0][0], map[0][1], INTER_AREA);//左校正
    remap(imgR, imgRr, map[1][0], map[1][1], INTER_AREA);//右校正

    imshow("imgLr", imgLr);
    imshow("imgRr", imgRr);
    waitKey(0);

    imwrite("../left.jpg", imgLr);
    imwrite("../right.jpg", imgRr);
*/
    return 0;
}