#include "xslam/vins/common/logging.h"

namespace xslam {
namespace common {

void InitializeGlog(char** argv) 
{
   google::InitGoogleLogging(argv[0]);
}

const char* __GetConstFileBaseName(const char* file) 
{
    const char* base = strrchr(file, '/');
    if (!base) 
    {
        base = strrchr(file, '\\');
    }
    return base ? (base + 1) : file;
}

bool __CheckOptionImpl(const char* file, const int line, const bool result,
                       const char* expr_str) 
{
    if (result) 
    {
        return true;
    } 
    else 
    {
        std::cerr << StringPrintf("[%s:%d] Check failed: %s",
                                  __GetConstFileBaseName(file), line, expr_str)
                  << std::endl;
        return false;
    }
}

}  // namespace common
}  // namespace xslam
