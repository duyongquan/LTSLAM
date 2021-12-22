//
// Created by quan on 2021/12/13.
//

#include "glog/logging.h"
#include "gflags/gflags.h"

#include <iostream>
#include <string>

// bool slam_version = false;
DEFINE_bool(slam_version, false, "Current soft version");

void ShowVersion()
{
    std::cout << "slam tutorial: v0.0.1"
              << std::endl;
}

// glfag 替代品
// cli11 .h文件
// option
// boost
// linux : ls grep pwd
int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_slam_version) {
        ShowVersion();
    }

    return 0;
}