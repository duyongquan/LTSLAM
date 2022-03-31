#include "xslam/vins/common/logging.h"

namespace xslam {
namespace vins {
namespace common {

void InitializeGlog(char** argv) 
{
   google::InitGoogleLogging(argv[0]);

    FLAGS_logtostderr      = true;  // 设置日志消息是否转到标准输出而不是日志文件
    FLAGS_alsologtostderr  = true;  // 设置日志消息除了日志文件之外是否去标准输出
    FLAGS_colorlogtostderr = true;  // 设置记录到标准输出的颜色消息（如果终端支持）
    FLAGS_log_prefix       = true;  // 设置日志前缀是否应该添加到每行输出
    FLAGS_logbufsecs       = 0;    // 设置可以缓冲日志的最大秒数，0指实时输出
    FLAGS_stop_logging_if_full_disk = true;  //设置是否在磁盘已满时避免日志记录到磁盘
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
}  // namespace vins
}  // namespace xslam
