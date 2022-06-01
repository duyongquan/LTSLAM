#ifndef XSLAM_PANGOLIN_SIMPLE_MULTI_DISPLAY_H
#define XSLAM_PANGOLIN_SIMPLE_MULTI_DISPLAY_H

#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/handler/handler.h>
#include <pangolin/gl/gldraw.h>

#include "glog/logging.h"
#include <string>

namespace xslam {
namespace pangolin {

    
class SimpleMultiDisplay
{
public:
    void RunDemo();

private:
    void setImageData(unsigned char * imageArray, int size);
};


} // namespace pangolin
} // namespace xslam

#endif // XSLAM_PANGOLIN_SIMPLE_MULTI_DISPLAY_H
