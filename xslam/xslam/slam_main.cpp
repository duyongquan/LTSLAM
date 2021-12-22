#include <iostream>
#include "glog/logging.h"


namespace protools {
    
void Run(int argc, char **argv)
{
    LOG(INFO) << "Hello World";
}


} // namespace protools

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    protools::Run(argc, argv);
    google::ShutdownGoogleLogging();

    return EXIT_SUCCESS;
}