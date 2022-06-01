#include <iostream>
#include <string>

#include "glog/logging.h"

namespace tutorial {
namespace cpp {
namespace {




void RunCase1()
{
    
    int ret = system(::std::string("lst -l").c_str());

    if (ret == 0) {

    }
    LOG(INFO) << "ret = " << ret ;
}

void Run()
{
    RunCase1();
}


} // namespace
} // namespace cpp
} // namespace tutorial

int main(int argc, char **argv)
{
    LOG(INFO) << "Start tutorial cpp tutorial_cpp_003_simple_non_type_class_template_main ... ";
    tutorial::cpp::Run();
    return 0;
}