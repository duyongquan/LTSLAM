#ifndef XSLAM_LIBQGLVIEWER_SIMPLE_VIEWER_H
#define XSLAM_LIBQGLVIEWER_SIMPLE_VIEWER_H

#include <QGLViewer/qglviewer.h>
#include "glog/logging.h"

namespace xslam {
namespace libQGLViewer {

class Viewer : public QGLViewer 
{
protected:
    virtual void draw();
    virtual void init();
    virtual QString helpString() const;
};

class SimpleViewer
{
public:
    void RunDemo();
};

} // namespace libQGLViewer
} // namespace xslam

#endif //XSLAM_LIBQGLVIEWER_SIMPLE_VIEWER_H

