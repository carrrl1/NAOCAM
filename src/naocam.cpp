///Include the header file
#include "naocam.h"

///The constructor initialize the broker and the objects.
NaoCam::NaoCam( boost::shared_ptr<AL::ALBroker> broker, const string& name):
		ALModule(broker, name),
    fVideoProxy(ALVideoDeviceProxy(broker)),
    mProxy(ALMotionProxy(broker)),
    aProxy(ALTextToSpeechProxy(broker)),
    leds(ALLedsProxy(broker)),
    posture(ALRobotPostureProxy(broker)),
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
	int resmode;
	cout << "\n Enter the desire resolution. \n1. kQQVGA image of 160*120px \n2. kQVGA image of 320*240px \n3. kVGA image of 640*480px \n4. k4VGA image of 1280*960px" << endl;
	cin >> resmode;
	switch (resmode) {
		case 1:
			cout << "Using kQQVGA configuration";
			pxx=160;
			pxy=120;
			fGVMId = fVideoProxy.subscribe("test", kQQVGA, kBGRColorSpace, 30);
			break;
		case 2:
			cout << "Using kQVGA configuration";
			pxx=320;
			pxy=240;
			fGVMId = fVideoProxy.subscribe("test", kQVGA, kBGRColorSpace, 30);
			break;
		case 3:
			cout << "Using kVGA configuration";
			pxx=640;
			pxy=480;
			fGVMId = fVideoProxy.subscribe("test", kVGA, kBGRColorSpace, 30);
			break;
		case 4:
			cout << "Using k4VGA configuration";
			pxx=1280;
			pxy=960;
			fGVMId = fVideoProxy.subscribe("test", k4VGA, kBGRColorSpace, 30);
			break;
		default:
			cout << "Using default configuration kQVGA";
			pxx=320;
			pxy=240;
			fGVMId = fVideoProxy.subscribe("test", kQVGA, kBGRColorSpace, 30);
			break;
		}

		///Define a Mat object with the dimensions of the NAO's camera requested resolution.
		cvFRAME = Mat(Size(pxx, pxy), CV_8UC3);

	//usingLEDS(3.0f);
	//posture.goToPosture("StandInit", 0.5f);

	///Select the desire mode.
	int mode;
	cout << "\n Enter the desire mode: \n1. Stream from Nao camera. \n2. Calibrate Nao camera. \n3. Start Visual Odometry" << endl;
	cin >> mode;
	switch (mode) {
		case 1:
			streamCamera();
			break;
		case 2:
			calibrateCamera();
			break;
		case 3:
			startVO();
			break;
		default:
			cout << "No mode selected!" <<endl;
			cout << "Ending..." <<endl;
			break;
	}

  //trackingObject();
  }

///If the fVideoProxy can't suscribe, then exit with status 1.
	catch (const ALError& e)
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
  const ALValue jointName = joint;

///Make sure the head is stiff to be able to move it. To do so, make the stiffness go to the maximum in one second.
///Target stiffness.
    ALValue stiffness = 1.0f;
///Time (in seconds) to reach the target.
    ALValue time = 1.0f;
///Call the stiffness interpolation method.
    mProxy.stiffnessInterpolation(jointName, stiffness, time);

///Set the target angle list, in radians.
    ALValue targetAngles = ALValue::array(move);
///Set the corresponding time lists, in seconds.
    ALValue targetTimes = ALValue::array(1.5f);
///Specify that the desired angles are absolute.
    bool isAbsolute = false;

///Call the angle interpolation method. The joint will reach the desired angles at the desired times.
    mProxy.angleInterpolation(jointName, targetAngles, targetTimes, isAbsolute);

///Remove the stiffness on the head.
    stiffness = 0.0f;
    time = 1.0f;
    mProxy.stiffnessInterpolation(jointName, stiffness, time);

}

int NaoCam::getFrame(void){
//Function to retrieve a frame from the Nao camera.

	/** Retrieve an image from the camera.
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
	alFRAME = fVideoProxy.getImageRemote(fGVMId);

	/** Access the image buffer (6th field) and assign it to the opencv image
	* container. */
	cvFRAME.data = (uchar*) alFRAME[6].GetBinary();

	/** Tells to ALVideoDevice that it can give back the image buffer to the
	* driver. Optional after a getImageRemote but MANDATORY after a getImageLocal.*/
	fVideoProxy.releaseImage(fGVMId);
	return 0;
}

int NaoCam::streamCamera(void){
	cout << "Streaming..." << endl;

	/** Create a OpenCV window to display the images. */
	namedWindow("NAO CAMERA STREAMING", WINDOW_AUTOSIZE );

	/** Main loop. Exit when pressing ESC.*/
	while ((char) waitKey(30) != 27){
		getFrame();
		/** Display the stream image on screen.*/
		imshow("NAO CAMERA STREAMING", cvFRAME);
	}
	return 0;
}

int NaoCam::calibrateCamera(void){
	///This function calibrates the camera using a Chessboard.
	cout << "CALIBRATION MODE" << endl;
	Mat calibFRAME;
	Mat calibFRAMEGRAY;

	int successes=0;
	int snapshots = 0;
  int horizontal_chess_corners;
  int vertical_chess_corners;
	///Each board cell is 0.03 m
	double sizeof_cell=0.03;

	vector<Point2f> corners;
	vector<Point3f> chessboard_points;
	vector<vector<Point3f> > appended_points;
	vector<vector<Point2f> > detected_points;

	///Ask the user to imput the chessboard features.
	cout<<"Enter number of corners along width: "<<endl;
	cin>>horizontal_chess_corners;

	cout<< "Enter number of corners along height: " <<endl;
	cin>>vertical_chess_corners;

	cout<< "Enter the number of snaps to take:" <<endl;
	cin>>snapshots;

	cout<< "Starting Chessboard detection..." <<endl;
	int total_cells = horizontal_chess_corners * vertical_chess_corners;
	Size chessboard_size = Size(horizontal_chess_corners, vertical_chess_corners);

	///Fill the vector coordinates of each corner of the chessboard
	for(int j=0;j<total_cells;j++) chessboard_points.push_back(Point3f(j/horizontal_chess_corners*sizeof_cell, j%horizontal_chess_corners*sizeof_cell, 0.0f));

	namedWindow("Chessboard Frame", WINDOW_AUTOSIZE );

	cout<< "Press spacebar to take a snap..." <<endl;
	while(successes<snapshots) {
		getFrame();
		//calibFRAME=cvFRAME;
		cvtColor(cvFRAME, calibFRAMEGRAY, CV_BGR2GRAY);
		///Try to find a chessboard in the frame.
		bool found = findChessboardCorners(calibFRAMEGRAY, chessboard_size, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		if(found) {
			///If the chessboard is found draw the detected corners in the frame
			cornerSubPix(calibFRAMEGRAY, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(calibFRAMEGRAY, chessboard_size, corners, found);
		}

		imshow("Chessboard Frame",calibFRAMEGRAY);

		//getFrame();
		//calibFRAME=cvFRAME;

		///Take a snap if spacebar is presed.
		key = waitKey(1);
		if(key==27) return 0;
		if(key==' ' || key==10 && found!=0) {
			detected_points.push_back(corners);
			appended_points.push_back(chessboard_points);
			cout << "Snap taken!" << endl;
			successes++;
			if(successes>=snapshots) break;
		}
	}

	destroyWindow("Chessboard Frame");
	//intrinsic was CV_32FC1
	Mat intrinsic = Mat(3, 3, CV_64F);
	Mat distortion;
	vector<Mat> rvecs;
	vector<Mat> tvecs;

	intrinsic.at<double>(0,0) = 1;
	intrinsic.at<double>(1,1) = 1;

	///Calibrate the camera with the detected corners to calculate the intrisic parameter matrix and distortion coefficients matrix.
	::calibrateCamera(appended_points, detected_points, cvFRAME.size(), intrinsic, distortion, rvecs, tvecs);

	cout<<"Intrinsic matrix: \n"<<intrinsic<<endl;
	cout<<"Intrinsic parameter fx: "<<intrinsic.at<double>(0,0)<<endl;
	cout<<"Intrinsic parameter fy: "<<intrinsic.at<double>(1,1)<<endl;
	cout<<"Intrinsic parameter cx: "<<intrinsic.at<double>(0,2)<<endl;
	cout<<"Intrinsic parameter cy: "<<intrinsic.at<double>(1,2)<<endl;
	cout<<intrinsic.at<double>(2,2)<<endl;

	///Save the calibration into a .yml file.
	cout<<"Saving intrisic camera matriz ..."<<endl;
	FileStorage fsw("CALIBRATION.YML", FileStorage::WRITE);
	fsw<< "CameraIntrinsicMatrix" <<intrinsic;
	cout<<"Saving distortion coefficients matriz ..."<<endl;
	fsw<< "DistortionCoefficientsMatrix" <<distortion;
	fsw.release();

	namedWindow("Distorted Frame", WINDOW_AUTOSIZE );
	namedWindow("Undistorted Frame", WINDOW_AUTOSIZE );

	///Show the image undistorted
	Mat undistorted_frame;
	while(true) {
		getFrame();
		undistort(cvFRAME, undistorted_frame, intrinsic, distortion);
		imshow("Distorted Frame", cvFRAME);
		imshow("Undistorted Frame", undistorted_frame);

		key = waitKey(1);
		if(key==27) break;
	}

	destroyAllWindows();
	return 0;
}

void NaoCam::readParameters(void){
	///This function reads the intrinsic and extrinsic parameters for a saved camara calibration.
	if(ifstream("CALIBRATION.YML")){
		Mat intrinsic=Mat(3, 3, CV_64F);
		FileStorage fsr("CALIBRATION.YML", FileStorage::READ);
		fsr["CameraIntrinsicMatrix"]>>intrMAT;
		cout<<"Intrinsic matrix: \n"<<intrMAT<<endl;
		cout<<"Intrinsic parameter fx: "<<intrMAT.at<double>(0,0)<<endl;
		cout<<"Intrinsic parameter fy: "<<intrMAT.at<double>(1,1)<<endl;
		cout<<"Intrinsic parameter cx: "<<intrMAT.at<double>(0,2)<<endl;
		cout<<"Intrinsic parameter cy: "<<intrMAT.at<double>(1,2)<<endl;
		focal=intrMAT.at<double>(0,0);
		cx=intrMAT.at<double>(0,2);
		cy=intrMAT.at<double>(1,2);
		fsr["DistortionCoefficientsMatrix"]>>distMAT;
		fsr.release();
	}
	else{
		cout<<"You have to calibrate the camera!"<<endl;
		cout<<"Using default calibration..."<<endl;
	}
}

void NaoCam::undistortFrame(void){
	///Method to get frame and undistort it.
		getFrame();
		undistort(cvFRAME, cvFRAME_UND, intrMAT, distMAT);
}

void NaoCam::featureDetection(Mat img_1, vector<Point2f>& points1)	{
	///uses FAST as of now, modify parameters as necessary
  vector<KeyPoint> keypoints_1;
  int fast_threshold = 20;
  bool nonmaxSuppression = true;
  FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
  KeyPoint::convert(keypoints_1, points1, vector<int>());


	///*Detect the keypoints using SURF Detector
	/// Set minHessian for the SurfFeatureDetector.
  //int minHessian = 400;
	//SurfFeatureDetector detector( minHessian );
	/// Set keypoint vectors for the image and the scene.
	//detector.detect( img_1, keypoints_1);
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

double NaoCam::getAbsoluteScale(void)	{

	///Function to get the absolute scale using the inertial module.
	double x=instPOSITION.at(0);
	double x_prev=prevPOSITION.at(0);

	double y=instPOSITION.at(1);
	double y_prev=prevPOSITION.at(1);

	double z=instPOSITION.at(2);
	double z_prev=prevPOSITION.at(2);

	//cout<<"Position in x "<<x<<endl;
	//cout<<"Previous position in x "<<x_prev<<endl;

	double scale_calculation=sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev));

	//prevPOSITION.at(0)=instPOSITION.at(0);
	//prevPOSITION.at(1)=instPOSITION.at(1);
	//prevPOSITION.at(2)=instPOSITION.at(2);
	prevPOSITION=instPOSITION;

  return scale_calculation;
}


int NaoCam::startVO(void)	{
	//Fame P or previous or past frame and Frame I for instant or current frame.
  Mat FRAME_P, FRAME_I;
	vector<Point2f> FEATURES_P;
	vector<Point2f> FEATURES_I;

	//Pose and the essential matrix
	Mat E, R, R_f, t, t_f, mask;
	bool init=true;
	double scale = 1.0;

	int zoom=60;

	//Intrinsic parameters of the camera:
	readParameters();
  Point2d pp(cx, cy);

	///Files to save the results.
  ofstream myfile1;
  myfile1.open ("TRAJECTORY.dat");
	ofstream myfile2;
  myfile2.open ("INERTIAL_TRAJECTORY.dat");

  char coordinates[100];
	char inertial_coordinates[100];

  int fontFace = FONT_HERSHEY_PLAIN;
  double fontScale = 1;
  int thickness = 1;
  Point textOrg(10, 50);

  ///Read the first frame and get the position of the nao.
	undistortFrame();
	///Wait the nao to change
	usleep(1000000);

  ///We use grayscale images for better features detection.
  cvtColor(cvFRAME_UND, FRAME_P, COLOR_BGR2GRAY);
	featureDetection(FRAME_P, FEATURES_P);

  namedWindow( "Nao camera", WINDOW_AUTOSIZE ); /// Create a window for display.
  namedWindow( "Trajectory", WINDOW_AUTOSIZE ); /// Create a window for display.
	namedWindow( "Inertial unit trajectory", WINDOW_AUTOSIZE ); /// Create a window for display.

	///Matrix containing the trajectory.
  Mat trajectory = Mat(600, 600, CV_8UC3,Scalar(255,255,255));
	Mat inertial_trajectory = Mat(600, 600, CV_8UC3,Scalar(255,255,255));

	while(true){
		///Get the current frame.
		instPOSITION=mProxy.getPosition("CameraTop", 1, true);
		undistortFrame();
  	cvtColor(cvFRAME_UND, FRAME_I, COLOR_BGR2GRAY);

  	vector<uchar> status;
  	featureTracking(FRAME_P, FRAME_I, FEATURES_P, FEATURES_I, status);

		///Get the essential matrix.
  	E = findEssentialMat(FEATURES_I, FEATURES_P, focal, pp, RANSAC, 0.999, 1.0, mask);
  	recoverPose(E, FEATURES_I, FEATURES_P, R, t, focal, pp, mask);

		///If is the first frame intialize R_t and t_f.
		if(init)
		{
			R_f=R.clone();
			t_f=t.clone();
			prevPOSITION=instPOSITION;
			init=false;
		}
		else{
				scale = getAbsoluteScale();
				cout<<"The absolute scale is "<<scale<<endl;
			if ((scale>0)&&(t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {
				R_f = R*R_f;
				t_f = t_f + scale*(R_f*t);
	   	}
		else {
			 //cout << "Scale below 0, or incorrect translation" << endl;
			}
		}

  /// A redetection is triggered in case the number of features being trakced go below a particular threshold
 	  //if (FEATURES_P.size() < MIN_NUM_FEAT)	{
 		  featureDetection(FRAME_P, FEATURES_P);
      featureTracking(FRAME_P,FRAME_I,FEATURES_P,FEATURES_I, status);
 	  //}

    FRAME_P = FRAME_I.clone();
    FEATURES_P = FEATURES_I;

    circle(trajectory, Point(int(t_f.at<double>(2)*zoom) + 300, int(t_f.at<double>(0)*zoom) + 300) ,1, CV_RGB(0,50,190), 2);
		circle(inertial_trajectory, Point(int(instPOSITION.at(0)*20)+300, int(instPOSITION.at(1)*20)+300) ,1, CV_RGB(0,50,190), 2);

    rectangle( trajectory, Point(10, 30), Point(550, 50), CV_RGB(255,255,255), CV_FILLED);
		rectangle( inertial_trajectory, Point(10, 30), Point(550, 50), CV_RGB(255,255,255), CV_FILLED);

    sprintf(coordinates, "Coordinates: x = %02f y = %02f z = %02f", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
    putText(trajectory, coordinates, textOrg, fontFace, fontScale, Scalar::all(0), thickness, 8);

		sprintf(inertial_coordinates, "Coordinates: x = %02f y = %02f z = %02f", instPOSITION.at(0), instPOSITION.at(1), instPOSITION.at(1));
    putText(inertial_trajectory, inertial_coordinates, textOrg, fontFace, fontScale, Scalar::all(0), thickness, 8);

    imshow( "Nao camera", FRAME_I);
    imshow( "Odometry trajectory", trajectory );
		imshow( "Inertial unit trajectory", inertial_trajectory );

		myfile1<<t_f.at<double>(2)<<"\t"<<t_f.at<double>(0)<<"\t"<<t_f.at<double>(1)<<endl;
		myfile2<<instPOSITION.at(0)<<"\t"<<instPOSITION.at(1)<<"\t"<<instPOSITION.at(2)<<endl;

		key = waitKey(1);
		if(key==27) break;
  }

	cout<<"Saving trajectories into images..."<<endl;
	imwrite("TRAJECTORY.PNG",trajectory);
	imwrite("INERTIAL_TRAJECTORY.PNG",inertial_trajectory);
	myfile1.close();
	myfile2.close();

  //clock_t end = clock();
  //double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  //cout << "Total time taken: " << elapsed_secs << "s" << endl;
  return 0;
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

	//Ptr<SURF> detector = SURF::create( minHessian );
/// Set keypoint vectors for the image and the scene.
	vector<KeyPoint> keypoints_object, keypoints_scene;
	//detector->detect( img_object, keypoints_object );
	//detector->detect( img_scene, keypoints_scene );

///*Calculate descriptors (feature vectors).
	//Ptr<SURF> extractor = SURF::create();
///Set descriptors vectors for the image and the scene.
	Mat descriptors_object, descriptors_scene;
	//extractor->compute( img_object, keypoints_object, descriptors_object );
	//extractor->compute( img_scene, keypoints_scene, descriptors_scene );

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
