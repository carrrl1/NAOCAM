///Include the header file
#include "naocam.h"

///The constructor initialize the broker and the objects.
NaoCam::NaoCam( boost::shared_ptr<AL::ALBroker> broker, const string& name):
	AL::ALModule(broker, name),
    fVideoProxy(AL::ALVideoDeviceProxy(broker)),
    mProxy(AL::ALMotionProxy(broker)),
    aProxy(AL::ALTextToSpeechProxy(broker)),
    leds(AL::ALLedsProxy(broker)),
    posture(AL::ALRobotPostureProxy(broker)),
    fGVMId("GVM")
{
  setModuleDescription("This module identify and tracks an 2D object using SURF and SIFT algorithms.");
}

///The destructor unsubscribes the proxies.
NaoCam::~NaoCam()
{
	fVideoProxy.unsubscribe(fGVMId);
	mProxy.rest();
	delete fImagePointer;
}


void NaoCam::init()
{
///Run the desire methods
	try
	{
/*
Parameters of .suscribe:
	*Name – Name of the subscribing module.
	*resolution – Resolution requested.
		**AL::kQQVGA image of 160*120px.
		**AL::kQVGA image of 320*240px.
		**AL::kVGA image of 640*480px
		**AL::k4VGA image of 1280*960px.
	*colorSpace – Colorspace requested.
	*fps – Fps (frames per second) requested to the video source. The OV7670 VGA camera can only run at 30fps.
*/
	fGVMId = fVideoProxy.subscribe(fGVMId, AL::kQVGA, AL::kRGBColorSpace, 30);
	usingLEDS(3.0f);
	posture.goToPosture("StandInit", 0.5f);
  //trackingObject();
  }

///If the fVideoProxy can't suscribe, then exit with status 1.
	catch (const AL::ALError& e)
	{
    cerr << "Caught exception: \n" << e.what() << endl;
	::exit(1);
	}
}

///Using LEDs.
void NaoCam::usingLEDS(float duration)
{
	leds.rasta(duration);
}

///Method for moving NAO's head, receives an angle (float) and the direction of movement (string).
///The string to receive can be either "HeadYaw" for X movement or "HeadPitch" for Y movement.
void NaoCam::moveHead(float move, string joint)
{

///The name of the joint to be moved.
  const AL::ALValue jointName = joint;

///Make sure the head is stiff to be able to move it. To do so, make the stiffness go to the maximum in one second.
///Target stiffness.
		AL::ALValue stiffness = 1.0f;
///Time (in seconds) to reach the target.
    AL::ALValue time = 1.0f;
///Call the stiffness interpolation method.
    mProxy.stiffnessInterpolation(jointName, stiffness, time);

///Set the target angle list, in radians.
    AL::ALValue targetAngles = AL::ALValue::array(move);
///Set the corresponding time lists, in seconds.
    AL::ALValue targetTimes = AL::ALValue::array(1.5f);
///Specify that the desired angles are absolute.
    bool isAbsolute = false;

///Call the angle interpolation method. The joint will reach the desired angles at the desired times.
    mProxy.angleInterpolation(jointName, targetAngles, targetTimes, isAbsolute);

///Remove the stiffness on the head.
    stiffness = 0.0f;
    time = 1.0f;
    mProxy.stiffnessInterpolation(jointName, stiffness, time);

}

void NaoCam::featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status)	{

///This function automatically gets rid of points for which tracking fails

  vector<float> err;
  Size winSize=Size(21,21);
  TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);

  calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);

  ///getting rid of points for which the KLT tracking failed or those who have gone outside the frame
  int indexCorrection = 0;
  for( int i=0; i<status.size(); i++)
     {  Point2f pt = points2.at(i- indexCorrection);
     	if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	{
     		  if((pt.x<0)||(pt.y<0))	{
     		  	status.at(i) = 0;
     		  }
     		  points1.erase (points1.begin() + (i - indexCorrection));
     		  points2.erase (points2.begin() + (i - indexCorrection));
     		  indexCorrection++;
     	}
     }
}


void NaoCam::featureDetection(Mat img_1, vector<Point2f>& points1)	{
	///uses FAST as of now, modify parameters as necessary
  vector<KeyPoint> keypoints_1;
  int fast_threshold = 20;
  bool nonmaxSuppression = true;
  FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
  KeyPoint::convert(keypoints_1, points1, vector<int>());
}
