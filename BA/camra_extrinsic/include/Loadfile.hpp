

#pragma once

#include <opencv2/core/core.hpp>

class Loadfile
{
public:
    Loadfile(const std::string &path);
    ~Loadfile() = default;

public:
    static void dataImput(const std::string &filepath, std::vector<cv::Point3d> &p3d);
    static void dataImput(const std::string &filepath, std::vector<cv::Point2d> &p2d);

    static bool fileExist(const std::string &path);

private:
    Loadfile &operator=(const Loadfile &) = delete;
    Loadfile(const Loadfile &) = delete;

private:
    const std::string &m_filepath;
};