#ifndef XSLAM_PANGOLIN_SIMPLE_DISPLAY_H
#define XSLAM_PANGOLIN_SIMPLE_DISPLAY_H

#include "glog/logging.h"

#include <pangolin/var/var.h>
#include <pangolin/var/varextra.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/display/widgets.h>
#include <pangolin/display/default_font.h>
#include <pangolin/handler/handler.h>

#include <string>

namespace xslam {
namespace pangolin {

    
class SimpleDisplay
{
public:
    void RunDemo();
};


} // namespace pangolin
} // namespace xslam

#endif // XSLAM_PANGOLIN_SIMPLE_DISPLAY_H



