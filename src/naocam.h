///Define the class name.
#ifndef NAOCAM_NAOCAM_H
#define NAOCAM_NAOCAM_H

///Include Aldebaran libraries to create a proxy between Nao and PC.

#include <boost/shared_ptr.hpp>
#include <alcommon/almodule.h>
#include <alvision/alimage.h>
#include <signal.h>
#include <alcommon/albrokermanager.h>
#include <alcommon/altoolsmain.h>
#include <alvalue/alvalue.h>
#include <alcommon/albroker.h>

#include <string>
#include <iostream>
#include <ctype.h>
#include <algorithm> // for copy
#include <iterator> // for ostream_iterator
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <unistd.h>

///Include Aldebaran libraries to suscribe the desire target.
#include <alvision/alvisiondefinitions.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <alproxies/alledsproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/alvideodeviceproxy.h>

///Include opencv libraries for image processing.
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

///Define namespace cv and std.
namespace cv{}
using namespace std;
using namespace cv;
namespace AL
{
  class ALBroker;
}
using namespace AL;

#define MAX_FRAME 1000
#define MIN_NUM_FEAT 2000

class NaoCam : public ALModule
{
///Define public methods.
///init() should be always be present.
  public:
    NaoCam(boost::shared_ptr<ALBroker> broker, const string& name);
    virtual ~NaoCam();
    void init();
    void moveHead(float move, string joint);
    void usingLEDS(float duration);
    void move_to(float theta);
    void trackingObject();
    int getFrame(void);
    int streamCamera(void);
    int calibrateCamera(void);
    void readParameters(void);
    void undistortFrame(void);
    void featureDetection(Mat img_1, vector<Point2f>& points1);
    void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status);
    double getAbsoluteScale(void);
    int startVO(void);

///Define private variables.
  private:
    ALVideoDeviceProxy fVideoProxy;
    string fGVMId;
    ALMotionProxy mProxy;
    ALTextToSpeechProxy aProxy;
    ALLedsProxy leds;
    ALRobotPostureProxy posture;
    ALImage* fImagePointer;
    ALValue alFRAME;
    Mat cvFRAME;
    Mat cvFRAME_UND;
    Mat intrMAT=Mat(3, 3, CV_64F);;
    Mat distMAT;
    int pxx,pxy;
    double focal = 562.5;
    double cx=324;
    double cy=189;
    int key;
    vector<float> prevPOSITION;
    vector<float> instPOSITION;
};

#endif  // NAOCAM_NAOCAM_H
