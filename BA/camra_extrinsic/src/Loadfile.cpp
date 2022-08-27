
#include <iostream>
#include <fstream>
#include <sstream>
#include <sys/stat.h>

#include "Loadfile.hpp"

Loadfile::Loadfile(const std::string &path) : m_filepath(path) {}

void Loadfile::dataImput(const std::string &filepath, std::vector<cv::Point3d> &p3d)
{
    if (!fileExist(filepath))
    {
        std::cout << "The loading of file is not exists!" << std::endl;
    }

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

void Loadfile::dataImput(const std::string &filepath, std::vector<cv::Point2d> &p2d)
{
    if (!fileExist(filepath))
    {
        std::cout << "The loading of file is not exists!" << std::endl;
    }

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

bool Loadfile::fileExist(const std::string &path)
{
    struct stat buffer;
    return (stat(path.c_str(), &buffer) == 0);
}