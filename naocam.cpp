/**
 * #Noobs
 */
 
///Include the header file
#include "naocam.h"

///Include Aldebaran libraries to create a proxy between Nao and PC.
#include <alvalue/alvalue.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>

///Include opencv libraries for image processing.
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"

///Define namespace cv and std.
namespace cv{}
using namespace std;
using namespace cv;

///Include Aldebaran libraries to suscribe the desire target.
#include <alvision/alvisiondefinitions.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <alproxies/alledsproxy.h>
#include <alproxies/alrobotpostureproxy.h>


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
  	trackingObject();
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

void NaoCam::trackingObject() 
{
///Load image of the object to be recognized and load to grayscale.
	const Mat img_object = imread("img.jpg", CV_LOAD_IMAGE_GRAYSCALE);
///Define an Mat objet with the dimensions of the NAO's camera requested resolution.
	Mat img_scene_color = Mat(Size(320, 240), CV_8UC3);
///Define Mat objet to contain scene image.
	Mat img_scene;
///Create a OpenCV window to display the images.
  	namedWindow("images");

///Main loop. Exit when pressing ESC.*/
while ((char) waitKey(30) != 27){
/*Retrieve an image from the camera.
    * The image is returned in the form of a container object, with the
    * following fields:
    * 0 = width
    * 1 = height
    * 2 = number of layers
    * 3 = colors space index (see alvisiondefinitions.h)
    * 4 = time stamp (seconds)
    * 5 = time stamp (micro seconds)
    * 6 = image buffer (size of width * height * number of layers)
*/
	AL::ALValue img = fVideoProxy.getImageRemote(fGVMId);

/// Access the image buffer (6th field) and assign it to the opencv image container.
    img_scene_color.data = (uchar*) img[6].GetBinary();
/* Convert coloured scene image to grayscale scene image.
	This is performed to have better matches*/ 
    cvtColor(img_scene_color, img_scene, CV_BGR2GRAY);
  
///*Detect the keypoints using SURF Detector
/// Set minHessian for the SurfFeatureDetector.
  	int minHessian = 400;
	SurfFeatureDetector detector( minHessian );
/// Set keypoint vectors for the image and the scene.
	vector<KeyPoint> keypoints_object, keypoints_scene;
	detector.detect( img_object, keypoints_object );
	detector.detect( img_scene, keypoints_scene );

///*Calculate descriptors (feature vectors).
	SurfDescriptorExtractor extractor;
///Set descriptors vectors for the image and the scene.
	Mat descriptors_object, descriptors_scene;
	extractor.compute( img_object, keypoints_object, descriptors_object );
	extractor.compute( img_scene, keypoints_scene, descriptors_scene );
  
///*Matching descriptor vectors using FLANN matcher.
	FlannBasedMatcher matcher;
	vector< DMatch > matches;
	matcher.match( descriptors_object, descriptors_scene, matches );

///Calculate max and min distances between keypoints
	double max_distance = 0; 
	double min_distance = 100;
	
	for( int i = 0; i < descriptors_object.rows; i++ )
	{ double distance = matches[i].distance;
    	if( distance < min_distance ) min_distance = distance;
		if( distance > max_distance ) max_distance = distance;
	}

///Print max and min distances.
	cout<<"Max distance: "<<max_distance<<endl;
	cout<<"Max distance: "<<min_distance<<endl;

///Draw only matches whose distance is less than 3*min_dist.
  	vector<DMatch> good_matches;

	for( int i = 0; i < descriptors_object.rows; i++ )
	{ 
		if( matches[i].distance < 3*min_distance )
     	{
			good_matches.push_back(matches[i]); 
		}
	}

///Print how many good matches where found.
    cout<<"--Good Matches: "<<good_matches.size()<<endl;

	Mat img_matches;
///Draw a line between matches.
	drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
				good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
				vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

///Localize the object.
	vector<Point2f> object;
	vector<Point2f> scene;

	for(int i=0; i<good_matches.size(); i++)
	{
///Get the keypoints from the good matches
    	object.push_back(keypoints_object[good_matches[i].queryIdx].pt );
    	scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt );
	}

	Mat H = findHomography(object, scene, CV_RANSAC);

///Get the corners from the object to be detected and the center of the object.
	vector<Point2f> obj_corners(5);
	obj_corners[0] = cvPoint(0,0);
	obj_corners[1] = cvPoint(img_object.cols, 0);
	obj_corners[2] = cvPoint(img_object.cols, img_object.rows); 
	obj_corners[3] = cvPoint(0, img_object.rows);
///Get the center of the object
	obj_corners[4] = cvPoint(img_object.cols/2,img_object.rows/2 ); 
	vector<Point2f> scene_corners(5);

///Adapt the image from the object to the scene.
	perspectiveTransform( obj_corners, scene_corners, H);

///Draw lines between the corners (the mapped object in the scene) and draw a circle in the center of the object.
  	line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 2);
 	line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 2);
 	line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
	line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
///Draw the center of the object.
	circle( img_matches, scene_corners[4] + Point2f( img_object.cols, 0), 3, Scalar( 255, 0, 0), -1 ); 
///Draw the center of the scene.
	circle( img_matches, Point2f(img_scene.cols/2, img_scene.rows/2) + Point2f( img_object.cols, 0), 3 , Scalar( 0, 0, 255), -1);  
  
///Print position of the center of the object.
	cout<<"--X position: "<<scene_corners[4].x<<endl;
	cout<<"--Y position: "<<scene_corners[4].y<<endl;

///Show detected matches
	imshow( "Object Detection", img_matches);
  
///Getting distance froms object to center.
	float movex = img_scene.cols/2-scene_corners[4].x;
	float movey = -img_scene.rows/2+scene_corners[4].y;

/*Moving head to object
	0.85f defines the transformation between distance between pixels and radians.
	Also works with 0.9f.
*/
	if(abs(movex)>1) {
		cout<<"--Moving in x: "<<movex<<endl;
		moveHead(movex*0.85f/img_scene.cols, "HeadYaw");
	}
	if(abs(movey)>1) {
		cout<<"--Moving in y: "<<movey<<endl;
		moveHead(movey*0.85f/img_scene.rows, "HeadPitch");
	}
	
/*If the distance between the center of the object and the center of the scene is less than
	30 pixels, then the object is detected, so say "I found the object"
*/ 

	if((abs(movex)<30)&&(abs(movey)<30))
	{
		usingLEDS(3.0f);
		aProxy.say("I found the object");
		cout<<"--Moving: "<<movex<<endl;
	}


	
/*Tells to ALVideoDevice that it can give back the image buffer to the
driver. Optional after a getImageRemote but MANDATORY after a getImageLocal.*/
	fVideoProxy.releaseImage(fGVMId);
  }
  destroyAllWindows();
///Do not forget to release the image. 
}