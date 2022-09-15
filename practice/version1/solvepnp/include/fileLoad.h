#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

class Fileload
{
public:
    Fileload();

    ~Fileload();

    static void dataLoad(const string &strFile, vector<Point3d> &data);

    static void dataLoad(const string &strFile, vector<Point2d> &data);

    static void dataLoad(const string &strFile, vector<double> &data);

private:
};