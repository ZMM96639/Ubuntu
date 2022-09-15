#include "fileLoad.h"
#include <fstream>
#include <sstream>

Fileload::Fileload()
{
    cout << "Fileload construction loading" << endl;
}

Fileload::~Fileload()
{
    cout << "Fileload destruction loading" << endl;
}

void Fileload::dataLoad(const string &strFile, vector<Point3d> &data)
{
    ifstream f(strFile.c_str());
    while (!f.eof())
    {
        string s;
        getline(f, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            Point3d tmp;
            ss >> tmp.x >> tmp.y >> tmp.z;
            if (tmp.x == 0 && tmp.y == 0 && tmp.z == 0)
            {
                continue;
            }
            data.push_back(tmp);
        }
    }
}

void Fileload::dataLoad(const string &strFile, vector<Point2d> &data)
{
    ifstream f(strFile.c_str());
    while (!f.eof())
    {
        string s;
        getline(f, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            Point2d tmp;
            ss >> tmp.x >> tmp.y;
            if (tmp.x == 0 && tmp.y == 0)
            {
                continue;
            }
            data.push_back(tmp);
        }
    }
}

void Fileload::dataLoad(const string &strFile, vector<double> &data)
{
    FileStorage fs(strFile, 0);
    if (fs.isOpened())
    {
        fs["Intrinsic_Matrix"] >> data;
        fs.release();
    }
}
