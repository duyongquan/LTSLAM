#include "xslam/libQGLViewer/key_frames.h"

#include <QGLViewer/manipulatedFrame.h>
#include <QKeyEvent>

#include <qapplication.h>
#include <qsplitter.h>

using namespace qglviewer;
using namespace std;

namespace xslam {
namespace libQGLViewer {

Viewer::Viewer() : nbKeyFrames(4) 
{
  restoreStateFromFile();

  // myFrame is the Frame that will be interpolated.
  Frame *myFrame = new Frame();

  // Set myFrame as the KeyFrameInterpolator interpolated Frame.
  kfi_.setFrame(myFrame);
  kfi_.setLoopInterpolation();

  // An array of manipulated (key) frames.
  keyFrame_ = new ManipulatedFrame *[nbKeyFrames];

  // Create an initial path
  for (int i = 0; i < nbKeyFrames; i++) {
    keyFrame_[i] = new ManipulatedFrame();
    keyFrame_[i]->setPosition(-1.0 + 2.0 * i / (nbKeyFrames - 1), 0.0, 0.0);
    kfi_.addKeyFrame(keyFrame_[i]);
  }

  currentKF_ = 0;
  setManipulatedFrame(keyFrame_[currentKF_]);

  // Enable direct frame manipulation when the mouse hovers.
  setMouseTracking(true);

  setKeyDescription(Qt::Key_Plus, "Increases interpolation speed");
  setKeyDescription(Qt::Key_Minus, "Decreases interpolation speed");
  setKeyDescription(Qt::Key_Left, "Selects previous key frame");
  setKeyDescription(Qt::Key_Right, "Selects next key frame");
  setKeyDescription(Qt::Key_Return, "Starts/stops interpolation");

  help();

  connect(&kfi_, SIGNAL(interpolated()), SLOT(update()));
  kfi_.startInterpolation();
}

QString Viewer::helpString() const 
{
  QString text("<h2>K e y F r a m e s</h2>");
  text += "A <i>KeyFrameInterpolator</i> holds an interpolated path defined by "
          "key frames. ";
  text += "It can then smoothly make its associed frame follow that path. Key "
          "frames can interactively be manipulated, even ";
  text += "during interpolation.<br><br>";
  text += "Note that the camera holds 12 such keyFrameInterpolators, binded to "
          "F1-F12. Press <b>Alt+Fx</b> to define new key ";
  text += "frames, and then press <b>Fx</b> to make the camera follow the "
          "path. Press <b>C</b> to visualize these paths.<br><br>";
  text += "<b>+/-</b> changes the interpolation speed. Negative values are "
          "allowed.<br><br>";
  text += "<b>Return</b> starts-stops the interpolation.<br><br>";
  text += "Use the left and right arrows to change the manipulated KeyFrame. ";
  text += "Press <b>Control</b> to move it or simply hover over it.";
  return text;
}

void Viewer::keyPressEvent(QKeyEvent *e) 
{
  switch (e->key()) {
  case Qt::Key_Left:
    currentKF_ = (currentKF_ + nbKeyFrames - 1) % nbKeyFrames;
    setManipulatedFrame(keyFrame_[currentKF_]);
    update();
    break;
  case Qt::Key_Right:
    currentKF_ = (currentKF_ + 1) % nbKeyFrames;
    setManipulatedFrame(keyFrame_[currentKF_]);
    update();
    break;
  case Qt::Key_Return:
    kfi_.toggleInterpolation();
    break;
  case Qt::Key_Plus:
    kfi_.setInterpolationSpeed(kfi_.interpolationSpeed() + 0.25);
    break;
  case Qt::Key_Minus:
    kfi_.setInterpolationSpeed(kfi_.interpolationSpeed() - 0.25);
    break;
  // case Qt::Key_C :
  // kfi_.setClosedPath(!kfi_.closedPath());
  // break;
  default:
    QGLViewer::keyPressEvent(e);
  }
}

void Viewer::draw() 
{
    // Draw interpolated frame
    glPushMatrix();
    glMultMatrixd(kfi_.frame()->matrix());
    drawAxis(0.3f);
    glPopMatrix();

    kfi_.drawPath(5, 10);

    for (int i = 0; i < nbKeyFrames; ++i) 
    {
        glPushMatrix();
        glMultMatrixd(kfi_.keyFrame(i).matrix());

        if ((i == currentKF_) || (keyFrame_[i]->grabsMouse()))
            drawAxis(0.4f);
        else
            drawAxis(0.2f);

        glPopMatrix();
    }
}

void KeyFrames::RunDemo()
{
    int argc = 1;
    char **argv = NULL;
    QApplication application(argc, argv);

    Viewer viewer;
    viewer.setWindowTitle("keyFrames");
    viewer.show();
    application.exec();
}

} // namespace libQGLViewer
} // namespace xslam
