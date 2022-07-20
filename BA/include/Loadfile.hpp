

#pragma once

#include <opencv2/core/core.hpp>

class Loadfile
{
public:
    Loadfile(const std::string &path);
    ~Loadfile();

public:
    static void dataImput(const std::string &filepath, std::vector<cv::Point3f> &p3d);
    static void dataImput(const std::string &filepath, std::vector<cv::Point2f> &p2d);

private:
    Loadfile &operator=(const Loadfile &) = delete;
    Loadfile(const Loadfile &) = delete;

private:
    const std::string &m_filepath;
};