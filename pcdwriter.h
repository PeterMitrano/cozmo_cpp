#pragma once

#include <iostream>
#include <aruco/aruco.h>
#include <opencv2/core.hpp>
#include <map>

void savePCDFile(const std::string &fpath, const aruco::MarkerMap &ms, std::map<int, cv::Mat> frame_pose_map);

void savePCDFile(const std::string &fpath,
                 const std::vector<aruco::MarkerMap> &ms,
                 std::map<int, cv::Mat> frame_pose_map);
