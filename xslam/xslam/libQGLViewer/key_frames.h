#ifndef XSLAM_LIBQGLVIEWER_KEY_FRAMES_H
#define XSLAM_LIBQGLVIEWER_KEY_FRAMES_H

#include <QGLViewer/qglviewer.h>
#include "glog/logging.h"

namespace xslam {
namespace libQGLViewer {

class Viewer : public QGLViewer 
{
public:
    Viewer();

protected:
    virtual void draw();
    virtual void keyPressEvent(QKeyEvent *e);
    virtual QString helpString() const;

private:
    qglviewer::ManipulatedFrame **keyFrame_;
    qglviewer::KeyFrameInterpolator kfi_;
    const int nbKeyFrames;
    int currentKF_;
};

class KeyFrames
{
public:
    void RunDemo();
};

} // namespace libQGLViewer
} // namespace xslam

#endif // XSLAM_LIBQGLVIEWER_KEY_FRAMES_H

