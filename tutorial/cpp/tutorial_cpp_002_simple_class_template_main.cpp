#include <iostream>
#include <string>

#include "glog/logging.h"


namespace tutorial {
namespace cpp {
namespace {

void RunCase1()
{
    
}

// Two-Phase Translation
void RunCase2()
{

}

void Run()
{
    RunCase1();
    RunCase2();
}

} // namespace
} // namespace cpp
} // namespace tutorial

int main(int argc, char **argv)
{
    LOG(INFO) << "Start tutorial_cpp_002_simple_class_template_main ... ";
    tutorial::cpp::Run();
    return 0;
}