#ifndef XSLAM_LIBQGLVIEWER_MULTI_VIEW_H
#define XSLAM_LIBQGLVIEWER_MULTI_VIEW_H

#include <QGLViewer/qglviewer.h>
#include "glog/logging.h"

namespace xslam {
namespace libQGLViewer {

class Scene 
{
public:
    void draw() const;
};

class Viewer : public QGLViewer 
{
public:
    Viewer(const Scene *const s, int type, QWidget *parent);

protected:
    virtual void draw();

private:
    const Scene *const scene_;
};

class MultiViewer
{
public:
    void RunDemo();
};

} // namespace libQGLViewer
} // namespace xslam

#endif // XSLAM_LIBQGLVIEWER_MULTI_VIEW_H

