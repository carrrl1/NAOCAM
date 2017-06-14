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
	cout << "\n Enter the desire mode. \n1. Stream from Nao camera. \n2. Calibrate Nao camera. \n3. Start Visual Odometry" << endl;
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
	namedWindow("NAO CAMERA STREAMING");

	/** Main loop. Exit when pressing ESC.*/
	while ((char) waitKey(30) != 27){
		getFrame();
		/** Display the stream image on screen.*/
		imshow("images", cvFRAME);
	}
	return 0;
}

int NaoCam::calibrateCamera(void){
	cout << "Calibrating..." << endl;
	int numBoards = 0;
  int numCornersHor;
  int numCornersVer;

	cout<<"Enter number of corners along width: "<<endl;
	cin>>numCornersHor;

	cout<< "Enter number of corners along height: " <<endl;
	cin>>numCornersVer;

	cout<< "Enter number of boards:" <<endl;
	cin>>numBoards;

	int numSquares = numCornersHor * numCornersVer;
	Size board_sz = Size(numCornersHor, numCornersVer);
	vector<vector<Point3f> > object_points;
	vector<vector<Point2f> > image_points;

	vector<Point2f> corners;
	int successes=0;

	vector<Point3f> obj;
	for(int j=0;j<numSquares;j++) obj.push_back(Point3f(j/numCornersHor, j%numCornersHor, 0.0f));

	Mat calibFRAME;
	Mat calibFRAMEGRAY;

	namedWindow("Chessboard Frame", WINDOW_AUTOSIZE );

	while(successes<numBoards) {
		getFrame();
		calibFRAME=cvFRAME;
		cvtColor(calibFRAME, calibFRAMEGRAY, CV_BGR2GRAY);
		bool found = findChessboardCorners(calibFRAME, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		if(found) {
			cornerSubPix(calibFRAMEGRAY, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(calibFRAMEGRAY, board_sz, corners, found);
		}

		imshow("Chessboard Frame",calibFRAMEGRAY);

		getFrame();
		calibFRAME=cvFRAME;

		int key = waitKey(1);
		if(key==27) return 0;
		if(key==' ' && found!=0) {
			image_points.push_back(corners);
			object_points.push_back(obj);
			cout << "Snap stored!" << endl;
			successes++;
			if(successes>=numBoards) break;
		}
	}

	Mat intrinsic = Mat(3, 3, CV_32FC1);
	Mat distCoeffs;
	vector<Mat> rvecs;
	vector<Mat> tvecs;

	intrinsic.ptr<float>(0)[0] = 1;
	intrinsic.ptr<float>(1)[1] = 1;

	::calibrateCamera(object_points, image_points, calibFRAME.size(), intrinsic, distCoeffs, rvecs, tvecs);

	namedWindow("Distorted Frame", WINDOW_AUTOSIZE );
	namedWindow("Undistorted Frame", WINDOW_AUTOSIZE );

	Mat imageUndistorted;
	while(1) {
		getFrame();
		calibFRAME=cvFRAME;
		undistort(calibFRAME, imageUndistorted, intrinsic, distCoeffs);
		imshow("Distorted Frame", calibFRAME);
		imshow("Undistorted Frame", imageUndistorted);
		waitKey(1);
	}

	return 0;
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
  //FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
  //KeyPoint::convert(keypoints_1, points1, vector<int>());
}

double NaoCam::getAbsoluteScale(int frame_id, int sequence_id, double z_cal)	{

  string line;
  int i = 0;
  ifstream myfile ("/home/avisingh/Datasets/KITTI_VO/00.txt");
  double x =0, y=0, z = 0;
  double x_prev, y_prev, z_prev;
  if (myfile.is_open())
  {
    while (( getline (myfile,line) ) && (i<=frame_id))
    {
      z_prev = z;
      x_prev = x;
      y_prev = y;
      std::istringstream in(line);
      //cout << line << '\n';
      for (int j=0; j<12; j++)  {
        in >> z ;
        if (j==7) y=z;
        if (j==3)  x=z;
      }

      i++;
    }
    myfile.close();
  }

  else {
    cout << "Unable to open file";
    return 0;
  }

  return sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev)) ;

}


int NaoCam::startVO(void)	{
	//Fame P or previous or past frame and Frame I for instant or current frame.
  Mat FRAME_P, FRAME_I;
	vector<Point2f> FEATURES_P;
	vector<Point2f> FEATURES_I;

	//Pose and the essential matrix
	Mat E, R, R_f, t, t_f, mask;
	bool init=true;
	double scale = 1.00;

	//Intrinsic parameters of the camera:
	//Fix
  double focal = 718.8560;
  Point2d pp(607.1928, 185.2157);

  ofstream myfile;
  myfile.open ("results.txt");

  char text[100];
  int fontFace = FONT_HERSHEY_PLAIN;
  double fontScale = 1;
  int thickness = 1;
  Point textOrg(10, 50);

  //Read the first frame
	getFrame();
  //We use grayscale images for better features detection.
  cvtColor(cvFRAME, FRAME_P, COLOR_BGR2GRAY);
	featureDetection(FRAME_P, FEATURES_P);

  clock_t begin = clock();

  namedWindow( "Nao camera", WINDOW_AUTOSIZE );// Create a window for display.
  namedWindow( "Trajectory", WINDOW_AUTOSIZE );// Create a window for display.

  Mat traj = Mat::zeros(600, 600, CV_8UC3);

  for(int numFrame=0; numFrame < MAX_FRAME; numFrame++)	{
		//Get the current frame.
		getFrame();
  	cvtColor(cvFRAME, FRAME_I, COLOR_BGR2GRAY);

  	vector<uchar> status;
  	featureTracking(FRAME_P, FRAME_I, FEATURES_P, FEATURES_I, status);

  	E = findEssentialMat(FEATURES_I, FEATURES_P, focal, pp, RANSAC, 0.999, 1.0, mask);
  	recoverPose(E, FEATURES_I, FEATURES_P, R, t, focal, pp, mask);

    Mat prevPts(2,FEATURES_P.size(), CV_64F), currPts(2,FEATURES_I.size(), CV_64F);


   for(int i=0;i<FEATURES_P.size();i++)	{
		 //this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
  		prevPts.at<double>(0,i) = FEATURES_P.at(i).x;
  		prevPts.at<double>(1,i) = FEATURES_P.at(i).y;

  		currPts.at<double>(0,i) = FEATURES_I.at(i).x;
  		currPts.at<double>(1,i) = FEATURES_I.at(i).y;
    }

  	scale = getAbsoluteScale(numFrame, 0, t.at<double>(2));

		if(init==true)
		{
			R_f=R.clone();
			t_f=t.clone();
			init=false;
		}
		else{
			if ((scale>0.1)&&(t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {
	      t_f = t_f + scale*(R_f*t);
	      R_f = R*R_f;
	    }
			else {
			 //cout << "scale below 0.1, or incorrect translation" << endl;
			}
		}

  // A redetection is triggered in case the number of feautres being trakced go below a particular threshold
 	  if (FEATURES_P.size() < MIN_NUM_FEAT)	{
 		  featureDetection(FRAME_P, FEATURES_P);
      featureTracking(FRAME_P,FRAME_I,FEATURES_P,FEATURES_I, status);
 	  }

    FRAME_P = FRAME_I.clone();
    FEATURES_P = FEATURES_I;

    int x = int(t_f.at<double>(0)) + 300;
    int y = int(t_f.at<double>(2)) + 100;
    circle(traj, Point(x, y) ,1, CV_RGB(255,0,0), 2);

    rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
    sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
    putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);

    imshow( "Road facing camera", FRAME_I);
    imshow( "Trajectory", traj );

    waitKey(1);
  }

  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  cout << "Total time taken: " << elapsed_secs << "s" << endl;
  return 0;
}
