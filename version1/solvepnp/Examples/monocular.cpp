//
// Created by zmm on 2021/12/28.
//
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {

    // assert(argc > 1);
    string img_path = "/home/zmm/practice/solvepnp/data";

    if (img_path.back() != '/') {
        img_path.append("/");
    }

    vector<string> imgList;
    size_t num = 11;
    for (int i = 1; i <= num; i++) {
        imgList.push_back(img_path + to_string(i) + ".png");
    }


    size_t n_img = imgList.size();
    size_t squareSize = 81;
    Size boardSize = Size(9, 8);
    Size imgSize;
    vector<vector<Point2f>> imgPoints;
    vector<vector<Point3f>> objectPoints;
    objectPoints.resize(n_img);

    
    for (int i = 0; i < n_img; i++) {
        for (int j = 0; j < boardSize.height; j++) {
            for (int k = 0; k < boardSize.width; k++) {
                objectPoints[i].push_back(Point3f(k * squareSize, j * squareSize, 0));
            }
        }
    }

    Mat img;
    for (int n = 0; n < n_img; n++) {
        img = imread(imgList[n]);
        vector<Point2f> corners;
        Mat gray, thresh_img;
        cvtColor(img, gray, COLOR_BGR2GRAY);
        adaptiveThreshold(gray, thresh_img, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 121, 20);
        bool patterFound = findChessboardCorners(thresh_img, boardSize, corners,
                                                 CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        if (!patterFound) {
            cout << "image: " << imgList[n] << "Cannot be found any corners" << endl;
        } else {
            cornerSubPix(thresh_img, corners, Size(5, 5), Size(-1, -1),
                         TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.01));
            Mat imgTmp = img.clone();
            for (int n_corners = 0; n_corners < corners.size(); n_corners++) {
                circle(imgTmp, corners[n_corners], 5, Scalar(0, 0, 255), -1, 8, 0);
            }
            imshow("corners", imgTmp);
            waitKey(10);
            imgPoints.push_back(corners);
        }
    }

//    cout<< imgPoints[0][0][0] << endl;

    destroyAllWindows();
    int flags = 0;
    vector<Mat> calib;
    imgSize = img.size();
    vector<Vec3d> rotation_vectors, translation_vectors;
    Mat cameraMatrix, disCoeffs, R, T, E, F;

 
    flags |= CALIB_ZERO_TANGENT_DIST;
    flags |= CALIB_FIX_PRINCIPAL_POINT;

    double error = calibrateCamera(objectPoints, imgPoints, imgSize, cameraMatrix, disCoeffs,
                                   rotation_vectors,
                                   translation_vectors, flags, TermCriteria(3, 100, 1e-6));

    cout << "RMS re-projection error: " << error << endl;
    cout << "camera" << ":\n" << "Intrinsic Matrix:\n" << cameraMatrix << endl;
    cout << "Discoeffs:\n" << disCoeffs << endl;

    FileStorage fs(img_path + "Intrinsic" + ".xml", 1);
    fs << "Intrinsic_Matrix" << "[";
    for (int h = 0; h < 3; h++) {
        for (int p = 0; p < 3; p++) {
            fs << cameraMatrix.at<double>(h, p);
        }
    }
    fs << "]";
    fs << "discoeffs" << "[";
    for (int i = 0; i < 2; i++) {
        fs << disCoeffs.at<double>(0, i);
    }
    fs << "]";
    fs.release();

}





