///Define the class name.
#ifndef NAOCAM_NAOCAM_H
#define NAOCAM_NAOCAM_H

#include <boost/shared_ptr.hpp>
#include <alcommon/almodule.h>
#include <alvision/alimage.h>

#include <string>
#include <iostream>
#include <ctype.h>
#include <algorithm> // for copy
#include <iterator> // for ostream_iterator
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>

///Include Aldebaran libraries to create a proxy between Nao and PC.
#include <alvalue/alvalue.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>

///Include Aldebaran libraries to suscribe the desire target.
#include <alvision/alvisiondefinitions.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <alproxies/alledsproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/alvideodeviceproxy.h>

///Include opencv libraries for image processing.
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv/cv.h"
#include "opencv/highgui.h"

///Define namespace cv and std.
namespace cv{}
using namespace std;
using namespace cv;
namespace AL
{
  class ALBroker;
}

class NaoCam : public AL::ALModule
{
///Define public methods.
///init() should be always be present.
  public:
    NaoCam(boost::shared_ptr<AL::ALBroker> broker, const string& name);
    virtual ~NaoCam();
    void init();
    void moveHead(float move, string joint);
    void usingLEDS(float duration);
    void move_to(float theta);
    //void trackingObject();
    void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status);
    void featureDetection(Mat img_1, vector<Point2f>& points1);

///Define private variables.
  private:
    AL::ALVideoDeviceProxy fVideoProxy;
    std::string fGVMId;
    AL::ALMotionProxy mProxy;
    AL::ALTextToSpeechProxy aProxy;
    AL::ALLedsProxy leds;
    AL::ALRobotPostureProxy posture;
    AL::ALImage* fImagePointer;
};

#endif  // NAOCAM_NAOCAM_H
