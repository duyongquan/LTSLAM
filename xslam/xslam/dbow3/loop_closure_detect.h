#ifndef XSLAM_DBOW3_LOOP_CLOSURE_DETECT_H
#define XSLAM_DBOW3_LOOP_CLOSURE_DETECT_H

#include <string>
#include <vector>

namespace xslam {
namespace dbow3 {

class LoopClosureDetect
{
public:
    void RunDemo(const std::vector<std::string>& paths);
};

} // namespace dbow3
} // namespace xslam

#endif //XSLAM_DBOW3_LOOP_CLOSURE_DETECT_H