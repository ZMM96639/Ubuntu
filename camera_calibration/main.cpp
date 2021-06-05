#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/calib3d/calib3d.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <vector>

using namespace std;
using namespace cv;
int CHECKERBOARD[2]{6, 9};

int main() {
    vector<vector<Point3f>> objpoints;
    vector<vector<Point2f>> imgpoint;
    vector<Point3f> objp;
    for (int i = 0; i < CHECKERBOARD[1]; i++) {
        for (int j = 0; j < CHECKERBOARD[0]; j++) {
            objp.push_back(Point3f(j, i, 0));
        }
    }

    vector<String> img;
    string path = "../1.BMP";
    Mat t;
    t = imread(path);
    namedWindow(path,WINDOW_KEEPRATIO);
    imshow("origin",t);
    glob(path, img);
    Mat frame, temp, gray;
    frame = imread(img[0]);
    cvtColor(frame,temp,COLOR_BGR2GRAY);
    namedWindow("img1",WINDOW_FREERATIO);
    imshow("img1",temp);
    equalizeHist(temp,gray);
    namedWindow("img2",WINDOW_FREERATIO);
    imshow("img2",gray);
//    vector<Point2f> corner_pts;
//    bool success;
//    for (int k = 0; k < img.size(); k++) {
//        frame = imread(img[k]);
//        cvtColor(frame, temp, COLOR_BGR2GRAY);
//        equalizeHist(temp, gray);
//        success = findChessboardCorners(gray, Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts,
//                                        CALIB_CB_NORMALIZE_IMAGE);
//        cout << corner_pts << endl;
//        cout << "success:" << success << endl;
//        if (success) {
//            TermCriteria criteria(cv::TermCriteria::EPS, 30, 0.001);
//            cornerSubPix(gray, corner_pts, Size(11, 11), Size(-1, -1), criteria);
//            drawChessboardCorners(frame, Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);
//            objpoints.push_back(objp);
//            cout << "corner_pts: " << corner_pts.front() << endl;
//            imgpoint.push_back(corner_pts);
//        }
//        namedWindow("img",WINDOW_KEEPRATIO);
//        imshow("img", frame);
//        waitKey(0);
//    }
//    destroyAllWindows();
//
//    cout << "objpoints: " << objpoints.size() << endl;
//
//
//    Mat camerMatrix, distCoeffs, R, T;
//    calibrateCamera(objpoints, imgpoint, Size(gray.rows, gray.cols), camerMatrix, distCoeffs, R, T);
//    cout << "camerMatrix : " << camerMatrix << endl;
//    cout << "distCoffs : " << distCoeffs << endl;
//    cout << "Rotation vector : " << R << endl;
//    cout << "Translation vector : " << T << endl;
    return 0;
}