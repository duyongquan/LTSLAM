//
// Created by quan on 2021/12/13.
//

#include "glog/logging.h"
#include "gflags/gflags.h"

DEFINE_bool(output_file, false, "Log to output files");

namespace slam {
namespace example {

void RunLogToScreen()
{
    LOG(INFO) << "info test";
    LOG(WARNING) << "warning test";
    LOG(ERROR) << "error test";
//    LOG(FATAL) << "fatal test";
}

void RunLogToFile(const std::string& filename)
{
    FLAGS_logtostderr = false;
    FLAGS_log_dir = "./";
    google::SetStderrLogging(google::GLOG_INFO);
//    google::SetLogFilenameExtension("log_");
//    FLAGS_colorlogtostderr = true;  // Set log color
    FLAGS_logbufsecs = 0;  // Set log output speed(s)
    FLAGS_max_log_size = 1024;  // Set max log file size
    FLAGS_stop_logging_if_full_disk = true;  // If disk is full
    LOG(INFO) << "Learn SLAM for robot users";
    LOG(INFO) << "info test" << "hello log!";  //输出一个Info日志
    LOG(WARNING) << "warning test";  //输出一个Warning日志
    LOG(ERROR) << "error test";  //输出一个Error日志
}


void RunCondition()
{
    int num_cookies = 20;
    LOG_IF(INFO, num_cookies > 10) << "Got lots of cookies";  //当条件满足时输出日志

    //google::COUNTER 记录该语句被执行次数，从1开始，在第一次运行输出日志之后，每隔 10 次再输出一次日志信息
    LOG_EVERY_N(INFO, 10)  << "Got the "
                             << google::COUNTER << "th cookie";

    //上述两者的结合，不过要注意，是先每隔 10 次去判断条件是否满足，如果滞则输出日志；而不是当满足某条件的情况下，每隔 10 次输出一次日志信息
    int size = 2048;
    LOG_IF_EVERY_N(INFO, (size > 1024), 10) << "Got the " << google::COUNTER << "th big cookie";

    //当此语句执行的前 20 次都输出日志，然后不再输出
    LOG_FIRST_N(INFO, 1) << "Got the " << google::COUNTER << "th cookie";
}

}
}

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true); // gflag

    FLAGS_logtostderr = true;      //设置日志消息是否转到标准输出而不是日志文件
    FLAGS_alsologtostderr = true;  //设置日志消息除了日志文件之外是否去标准输出
    FLAGS_colorlogtostderr = true;  //设置记录到标准输出的颜色消息（如果终端支持）
    FLAGS_log_prefix = true;  //设置日志前缀是否应该添加到每行输出
    FLAGS_logbufsecs = 0;  //设置可以缓冲日志的最大秒数，0指实时输出
    FLAGS_max_log_size = 10;  //设置最大日志文件大小（以MB为单位）
    FLAGS_stop_logging_if_full_disk = true;  //设置是否在磁盘已满时避免日志记录到磁盘

    LOG(INFO) << "Run glog example...";

    if (FLAGS_output_file) {
        std::string log_filename = "glog_example.log";
        slam::example::RunLogToFile(log_filename);
    } else {
        slam::example::RunLogToScreen();
    }

    for (int i = 0; i < 100; i++) {
        slam::example::RunCondition();
    }

    google::ShutdownGoogleLogging(); // glog
    return 0;
}