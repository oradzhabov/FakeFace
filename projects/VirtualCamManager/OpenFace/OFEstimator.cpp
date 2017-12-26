#include "OFEstimator.h"

// Libraries for landmark detection (includes CLNF and CLM modules)
#include "GazeEstimation.h"

#include <fstream>
#include <sstream>

// OpenCV includes
#include <opencv2/videoio/videoio.hpp>  // Video write
#include <opencv2/videoio/videoio_c.h>  // Video write
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Boost includes
#include <filesystem.hpp>
#include <filesystem/fstream.hpp>

#include <vector>


OFEstimator::OFEstimator()
	:IHeadEstimator(),
    det_parameters(NULL),
	clnf_model(NULL)
{
	vector<string> arguments;

	arguments.push_back ("AppName.exe");

	det_parameters = new LandmarkDetector::FaceModelParameters (arguments);

	// Some initial parameters that can be overriden from command line	
	vector<string> files, depth_directories, output_video_files, out_dummy;
	
	// By default try webcam 0
	int device = 0; // ros: was 0

	// Get the input output file parameters
	
	// Indicates that rotation should be with respect to world or camera coordinates
	bool u;
	string output_codec;
	LandmarkDetector::get_video_input_output_params(files, depth_directories, out_dummy, output_video_files, u, output_codec, arguments);
	
	// here we can see that _alt_tree more than 10% better than _alt.
	// http://stackoverflow.com/questions/4440283/how-to-choose-the-cascade-file-for-face-detection
	//  but does not work by some reason
	/*
	det_parameters->face_detector_location = "classifiers/haarcascade_frontalface_alt_tree.xml";
	det_parameters->curr_face_detector = LandmarkDetector::FaceModelParameters::HAAR_DETECTOR;
	*/

	// The modules that are being used for tracking
	clnf_model = new LandmarkDetector::CLNF(det_parameters->model_location);	

	// Grab camera parameters, if they are not defined (approximate values will be used)
//	float fx = 0, fy = 0, cx = 0, cy = 0;
	fx = 0;
	fy = 0;
	cx = 0;
	cy = 0;
	// Get camera parameters
//	LandmarkDetector::get_camera_params(device, fx, fy, cx, cy, arguments);

	// If cx (optical axis centre) is undefined will use the image size/2 as an estimate
	cx_undefined = false;
	fx_undefined = false;

	if (cx == 0 || cy == 0)
	{
		cx_undefined = true;
	}
	if (fx == 0 || fy == 0)
	{
		fx_undefined = true;
	}

	det_parameters->track_gaze = false;
}


OFEstimator::~OFEstimator()
{
	if (det_parameters)
		delete det_parameters;
	if (clnf_model)
		delete clnf_model;
}

void
OFEstimator::Initialize(const float & focalLength, const float & opticalCenterX, const float & opticalCenterY)
{
	cx = opticalCenterX;
	cy = opticalCenterY;

    cx_undefined = false;
    //
    //
    //
	fx = focalLength;
	fy = focalLength;

    fx_undefined = false;
}


// Visualising the results
void visualise_tracking(cv::Mat& captured_image, cv::Mat_<float>& depth_image, const LandmarkDetector::CLNF& face_model, const LandmarkDetector::FaceModelParameters& det_parameters, cv::Point3f gazeDirection0, cv::Point3f gazeDirection1, double fx, double fy, double cx, double cy)
{

	// Drawing the facial landmarks on the face and the bounding box around it if tracking is successful and initialised
	double detection_certainty = face_model.detection_certainty;
	bool detection_success = face_model.detection_success;

	double visualisation_boundary = 0.2;

	// Only draw if the reliability is reasonable, the value is slightly ad-hoc
	if (detection_certainty < visualisation_boundary)
	{
		LandmarkDetector::Draw(captured_image, face_model);

		double vis_certainty = detection_certainty;
		if (vis_certainty > 1)
			vis_certainty = 1;
		if (vis_certainty < -1)
			vis_certainty = -1;

		vis_certainty = (vis_certainty + 1) / (visualisation_boundary + 1);

		// A rough heuristic for box around the face width
		int thickness = (int)std::ceil(2.0* ((double)captured_image.cols) / 640.0);

		cv::Vec6d pose_estimate_to_draw = LandmarkDetector::GetCorrectedPoseWorld(face_model, fx, fy, cx, cy);

		// Draw it in reddish if uncertain, blueish if certain
		LandmarkDetector::DrawBox(captured_image, pose_estimate_to_draw, cv::Scalar((1 - vis_certainty)*255.0, 0, vis_certainty * 255), thickness, fx, fy, cx, cy);
		
		if (det_parameters.track_gaze && detection_success && face_model.eye_model)
		{
			FaceAnalysis::DrawGaze(captured_image, face_model, gazeDirection0, gazeDirection1, fx, fy, cx, cy);
		}
	}

	// Work out the framerate
	/*
	if (frame_count % 10 == 0)
	{
		double t1 = cv::getTickCount();
		fps_tracker = 10.0 / (double(t1 - t0) / cv::getTickFrequency());
		t0 = t1;
	}
	
	// Write out the framerate on the image before displaying it
	char fpsC[255];
	std::sprintf(fpsC, "%d", (int)fps_tracker);
	string fpsSt("FPS:");
	fpsSt += fpsC;
	cv::putText(captured_image, fpsSt, cv::Point(10, 20), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 0));
	*/
	/*
	if (!det_parameters.quiet_mode)
	{
		cv::namedWindow("tracking_result", 1);
		cv::imshow("tracking_result", captured_image);

		if (!depth_image.empty())
		{
			// Division needed for visualisation purposes
			imshow("depth", depth_image / 2000.0);
		}
	}
	*/
}

void
OFEstimator::update(const cv::InputArray image, const bool isFirstFrame)
{
	cv::Mat captured_image = image.getMat();
	if(captured_image.empty())
		return;

	// If optical centers are not defined just use center of image
	if (cx_undefined)
	{
		cx = captured_image.cols / 2.0f;
		cy = captured_image.rows / 2.0f;
	}
	// Use a rough guess-timate of focal length
	if (fx_undefined)
	{
		fx = 500 * (captured_image.cols / 640.0);
		fy = 500 * (captured_image.rows / 480.0);

		fx = (fx + fy) / 2.0;
		fy = fx;
	}		
	//
	// Reading the images
	cv::Mat_<float> depth_image;
	cv::Mat_<uchar> grayscale_image;

	if(captured_image.channels() == 3)
	{
		cv::cvtColor(captured_image, grayscale_image, CV_BGR2GRAY);				
	}
	else
	{
		grayscale_image = captured_image.clone();				
	}

	// The actual facial landmark detection / tracking
	bool detection_success = LandmarkDetector::DetectLandmarksInVideo(grayscale_image, depth_image, *clnf_model, *det_parameters);
			
	// Visualising the results
	// Drawing the facial landmarks on the face and the bounding box around it if tracking is successful and initialised
	double detection_certainty = clnf_model->detection_certainty;

	// Gaze tracking, absolute gaze direction
	cv::Point3f gazeDirection0(0, 0, -1);
	cv::Point3f gazeDirection1(0, 0, -1);

	if (det_parameters->track_gaze && detection_success && clnf_model->eye_model)
	{
		FaceAnalysis::EstimateGaze(*clnf_model, gazeDirection0, fx, fy, cx, cy, true);
		FaceAnalysis::EstimateGaze(*clnf_model, gazeDirection1, fx, fy, cx, cy, false);
	}

#ifdef HEAD_POSE_ESTIMATOR_DEBUG
	_debug = captured_image.clone();
#endif // HEAD_POSE_ESTIMATOR_DEBUG

	m_landmarks.clear();
	if (detection_success)
	{
		const int n = clnf_model->detected_landmarks.rows/2;
	
		dlib::rectangle				rect;
		std::vector<dlib::point>	pts(68);


		// Drawing feature points
		if(n == 68)
		{
			pts[0] = dlib::point(cvRound(clnf_model->detected_landmarks.at<double>(0)),
												cvRound(clnf_model->detected_landmarks.at<double>(0 + n)));
			rect = pts.front();

			for( int i = 1; i < n; ++i)
			{		
				pts[i] = dlib::point(cvRound(clnf_model->detected_landmarks.at<double>(i)),
									cvRound(clnf_model->detected_landmarks.at<double>(i + n)));

				rect += pts[i];
			}
			m_landmarks.push_back (dlib::full_object_detection (rect, pts));
		}
#ifdef HEAD_POSE_ESTIMATOR_DEBUG
		visualise_tracking (_debug, depth_image, *clnf_model, *det_parameters, gazeDirection0, gazeDirection1, fx, fy, cx, cy);
#endif // HEAD_POSE_ESTIMATOR_DEBUG
	}

	if (m_landmarks.empty())
	{
		// restart the tracker
		clnf_model->Reset();
	}
}

void
OFEstimator::drawMesh(size_t face_idx) const
{
#ifdef HEAD_POSE_ESTIMATOR_DEBUG
	if (m_landmarks.empty() == false)
	{
		for (size_t ti = 0; ti < m_FaceMesh.size(); ++ti)
		{
			const IHeadEstimator::sTriangle & t = m_FaceMesh[ti];
			for (int ei = 0; ei < 3; ++ei)
			{
				const int ei2 = (ei + 1) % 3;
				//
				const size_t v0 = t.vInd[ei];
				const size_t v1 = t.vInd[ei2];

				line(_debug, toCv(m_landmarks.front().part(v0)),
							toCv(m_landmarks.front().part(v1)), 
							cv::Scalar(0, 255, 0), 1, CV_AA);
			}
		}
	}
#endif // HEAD_POSE_ESTIMATOR_DEBUG
}

const size_t
OFEstimator::getShapesNb() const
{
	return m_landmarks.size();
}

const dlib::full_object_detection &
OFEstimator::getShape(const size_t & index) const
{
	return m_landmarks.front();
}
