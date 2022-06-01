#ifndef XSLAM_PANGOLIN_HELLO_PANGOLIN_H
#define XSLAM_PANGOLIN_HELLO_PANGOLIN_H

#include "glog/logging.h"

#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/handler/handler.h>
#include <pangolin/gl/gldraw.h>

#include <string>

namespace xslam {
namespace pangolin {

    
class HelloPangolin
{
public:
    void RunDemo();
};


} // namespace pangolin
} // namespace xslam

#endif // XSLAM_PANGOLIN_HELLO_PANGOLIN_H