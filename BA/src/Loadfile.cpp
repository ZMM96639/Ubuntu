
#include <iostream>
#include <fstream>
#include <sstream>

#include "Loadfile.hpp"

Loadfile::Loadfile(const std::string &path) : m_filepath(path) {}

Loadfile::~Loadfile() {}

void Loadfile::dataImput(const std::string &filepath, std::vector<cv::Point3f> &p3d)
{
    std::ifstream f(filepath.c_str(), std::ios::in);

    while (!f.eof())
    {
        std::string s;
        getline(f, s);

        if (!s.empty())
        {
            std::stringstream ss;
            ss << s;

            cv::Point3f tmp;
            ss >> tmp.x >> tmp.y >> tmp.z;
            p3d.push_back(tmp);
        }
        
    }
}

void Loadfile::dataImput(const std::string &filepath, std::vector<cv::Point2f> &p2d)
{
    std::ifstream f(filepath.c_str(), std::ios::in);
    while (!f.eof())
    {
        std::string s;
        getline(f, s);

        if (!s.empty())
        {
            std::stringstream ss;
            ss << s;

            cv::Point2f tmp;
            ss >> tmp.x >> tmp.y;
            p2d.push_back(tmp);
        }
        
    }
}