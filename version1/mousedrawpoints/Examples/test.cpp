#include <iostream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>

#include "Mouse.h"

using namespace cv;
using namespace std;

int main(int argc, char *argv[])
{
    string path = "/home/zmm/practice/version1/solvepnp/data";

    if (path.back() != '/')
    {
        path.append("/");
    }

    for (int i = 1; i < 12; i++)
    {

        string img_path = path + to_string(i) + ".png";
        Mat img = imread(img_path);

        namedWindow("origin image", WINDOW_KEEPRATIO);
        setMouseCallback("origin image", Mouse::onMouse, &img);
        imshow("origin image", img);

        waitKey(0);
    }

    fstream fout(path + "ppppoints.txt",ios::out);

    for(int i = 0; i < 11; i++){
        fout << Mouse::ppoints[i].x << " " << Mouse::ppoints[i].y << endl;
    }
    fout.close();

    for(auto val:Mouse::ppoints){
        cout << val << endl;
    }
     
    return 0;
}
