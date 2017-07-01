#include <cmath>
#include <ctime>

#include <opencv2/calib3d/calib3d.hpp>

#ifdef HEAD_POSE_ESTIMATOR_DEBUG
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#endif

// Good explanation about Camera coordinates and other CS
// http://stackoverflow.com/questions/15727527/how-to-determine-world-coordinates-of-a-camera
//
// glsl Poisson Blending(seamlessCloning)
// https://www.shadertoy.com/view/4l3Xzl

#include "HeadPoseEstimator.h"


bool cvRectAreaComparer (cv::Rect & i,cv::Rect & j) { return (i.area() > j.area()); }

const double HeadPoseEstimator::TICK_FREQUENCY = cv::getTickFrequency();

HeadPoseEstimator::HeadPoseEstimator():
        IHeadEstimator(),
        focalLength(350),
        opticalCenterX(-1),
        opticalCenterY(-1),
        m_faceCascade(NULL),
        m_resizedWidth(320),
        m_foundFace(false),
        m_templateMatchingRunning(false),
        m_templateMatchingStartTime(0),
        m_templateMatchingCurrentTime(0),
        m_templateMatchingMaxDuration(3),
        kf(NULL),
        lastResult_kf(false),
        m_ticks_kf(0)
{
	Load("shape_predictor_68_face_landmarks.dat", 554, "haarcascade_frontalface_default.xml");
}

void
HeadPoseEstimator::Load(const std::string& face_detection_model, float focalLength_, const std::string cascadeFilePath) 
{
	this->focalLength = focalLength_;

    setFaceCascade(cascadeFilePath);

    // setup Kalman filter for face's rect
    int stateSize = 6;
    int measSize = 4;
    int contrSize = 0;
    kf = new cv::KalmanFilter(stateSize, measSize, contrSize, CV_32F);
    state.create(stateSize, 1, CV_32F);  // [x,y,v_x,v_y,w,h]
    meas.create(measSize, 1, CV_32F);    // [z_x,z_y,z_w,z_h]
    //cv::Mat procNoise(stateSize, 1, type)
    // [E_x,E_y,E_v_x,E_v_y,E_w,E_h]

    // Transition State Matrix A
    // Note: set dT at each processing step!
    // [ 1 0 dT 0  0 0 ]
    // [ 0 1 0  dT 0 0 ]
    // [ 0 0 1  0  0 0 ]
    // [ 0 0 0  1  0 0 ]
    // [ 0 0 0  0  1 0 ]
    // [ 0 0 0  0  0 1 ]
    cv::setIdentity(kf->transitionMatrix);

    // Measure Matrix H
    // [ 1 0 0 0 0 0 ]
    // [ 0 1 0 0 0 0 ]
    // [ 0 0 0 0 1 0 ]
    // [ 0 0 0 0 0 1 ]
    kf->measurementMatrix = cv::Mat::zeros(measSize, stateSize, CV_32F);
    kf->measurementMatrix.at<float>(0) = 1.0f;
    kf->measurementMatrix.at<float>(7) = 1.0f;
    kf->measurementMatrix.at<float>(16) = 1.0f;
    kf->measurementMatrix.at<float>(23) = 1.0f;

    // Process Noise Covariance Matrix Q
    // [ Ex   0   0     0     0    0  ]
    // [ 0    Ey  0     0     0    0  ]
    // [ 0    0   Ev_x  0     0    0  ]
    // [ 0    0   0     Ev_y  0    0  ]
    // [ 0    0   0     0     Ew   0  ]
    // [ 0    0   0     0     0    Eh ]
    //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
    kf->processNoiseCov.at<float>(0) = 1e-2; // 1e-2
    kf->processNoiseCov.at<float>(7) = 1e-2; // 1e-2
    kf->processNoiseCov.at<float>(14) = 5.0f; // 5.0f
    kf->processNoiseCov.at<float>(21) = 5.0f; // 5.0f
    kf->processNoiseCov.at<float>(28) = 1e-2; // 1e-2
    kf->processNoiseCov.at<float>(35) = 1e-2; // 1e-2

    // Measures Noise Covariance Matrix R
    cv::setIdentity(kf->measurementNoiseCov, cv::Scalar(1)); // 1e-1

    // Load face detection and pose estimation models.
    detector = dlib::get_frontal_face_detector();
    dlib::deserialize(face_detection_model) >> pose_model;

}

void
HeadPoseEstimator::Initialize(const float & focalLength_, const float & opticalCenterX_, const float & opticalCenterY_)
{
    this->focalLength = focalLength_;
    this->opticalCenterX = opticalCenterX_;
    this->opticalCenterY = opticalCenterY_;
}

HeadPoseEstimator::~HeadPoseEstimator()
{
    if (m_faceCascade != NULL)
        delete m_faceCascade;
    
    if (kf != NULL)
        delete kf;
}

void HeadPoseEstimator::setFaceCascade(const std::string cascadeFilePath)
{
    if (m_faceCascade == NULL) {
        m_faceCascade = new cv::CascadeClassifier(cascadeFilePath);
    }
    else {
        m_faceCascade->load(cascadeFilePath);
    }

    isHaarLoaded = true;
    if (m_faceCascade->empty()) {
        isHaarLoaded = false;
        std::cerr << "Error creating cascade classifier. Make sure the file" << std::endl
            << cascadeFilePath << " exists." << std::endl;
    }
}


cv::Mat HeadPoseEstimator::getFaceTemplate(const cv::Mat &frame, cv::Rect face)
{
    face.x += face.width / 4;
    face.y += face.height / 4;
    face.width /= 2;
    face.height /= 2;

    cv::Mat faceTemplate = frame(face).clone();
    return faceTemplate;
}

cv::Rect HeadPoseEstimator::doubleRectSize(const cv::Rect &inputRect, const cv::Rect &frameSize) const
{
    cv::Rect outputRect;
    // Double rect size
    outputRect.width = inputRect.width * 2;
    outputRect.height = inputRect.height * 2;

    // Center rect around original center
    outputRect.x = inputRect.x - inputRect.width / 2;
    outputRect.y = inputRect.y - inputRect.height / 2;

    // Handle edge cases
    if (outputRect.x < frameSize.x) {
        outputRect.width += outputRect.x;
        outputRect.x = frameSize.x;
    }
    if (outputRect.y < frameSize.y) {
        outputRect.height += outputRect.y;
        outputRect.y = frameSize.y;
    }

    if (outputRect.x + outputRect.width > frameSize.width) {
        outputRect.width = frameSize.width - outputRect.x;
    }
    if (outputRect.y + outputRect.height > frameSize.height) {
        outputRect.height = frameSize.height - outputRect.y;
    }

    return outputRect;
}

cv::Point HeadPoseEstimator::centerOfRect(const cv::Rect &rect) const
{
    return cv::Point(rect.x + rect.width / 2, rect.y + rect.height / 2);
}

void HeadPoseEstimator::detectFaceAllSizes(const cv::Mat &frame)
{
    m_foundFace = false;

    // Minimum face size is 1/5th of screen height
    // Maximum face size is 2/3rds of screen height
    m_faceCascade->detectMultiScale(frame, m_allFaces, 1.2, 24, 0,
                                    cv::Size(frame.rows / 5, frame.rows / 5),
                                    cv::Size(frame.rows * 2 / 3, frame.rows * 2 / 3));

    if (m_allFaces.empty())
        return;

    // sort rectangles by area descending
    if (m_allFaces.size() > 1)
        std::sort(m_allFaces.begin(), m_allFaces.end(), cvRectAreaComparer);

    m_foundFace = true;

    // Locate biggest face
    //m_trackedFace = biggestFace(m_allFaces);
    m_trackedFace = m_allFaces.front();

    // Copy face template
    m_faceTemplate = getFaceTemplate(frame, m_trackedFace);

    // Calculate roi
    m_faceRoi = doubleRectSize(m_trackedFace, cv::Rect(0, 0, frame.cols, frame.rows));

    // Update face position
    m_facePosition = centerOfRect(m_trackedFace);
}

void HeadPoseEstimator::detectFaceAroundRoi(const cv::Mat &frame)
{
    // Detect faces sized +/-20% off biggest face in previous search
    m_faceCascade->detectMultiScale(frame(m_faceRoi), m_allFaces, 1.1, 3, 0,
        cv::Size(m_trackedFace.width * 8 / 10, m_trackedFace.height * 8 / 10),
        cv::Size(m_trackedFace.width * 12 / 10, m_trackedFace.width * 12 / 10));

    if (m_allFaces.empty())
    {
        // Activate template matching if not already started and start timer
        m_templateMatchingRunning = true;
        if (m_templateMatchingStartTime == 0)
            m_templateMatchingStartTime = cv::getTickCount();
        return;
    }

    // sort rectangles by area descending
    if (m_allFaces.size() > 1)
        std::sort(m_allFaces.begin(), m_allFaces.end(), cvRectAreaComparer);


    // Turn off template matching if running and reset timer
    m_templateMatchingRunning = false;
    m_templateMatchingCurrentTime = m_templateMatchingStartTime = 0;

    // Get detected face
    //m_trackedFace = biggestFace(m_allFaces);
    m_trackedFace = m_allFaces.front();

    // Add roi offset to face
    m_trackedFace.x += m_faceRoi.x;
    m_trackedFace.y += m_faceRoi.y;

    // Get face template
    m_faceTemplate = getFaceTemplate(frame, m_trackedFace);

    // Calculate roi
    m_faceRoi = doubleRectSize(m_trackedFace, cv::Rect(0, 0, frame.cols, frame.rows));

    // Update face position
    m_facePosition = centerOfRect(m_trackedFace);
}

void HeadPoseEstimator::detectFacesTemplateMatching(const cv::Mat &frame)
{
    // Calculate duration of template matching
    m_templateMatchingCurrentTime = cv::getTickCount();
    double duration = (double)(m_templateMatchingCurrentTime - m_templateMatchingStartTime) / TICK_FREQUENCY;

    // If template matching lasts for more than 2 seconds face is possibly lost
    // so disable it and redetect using cascades
    if (duration > m_templateMatchingMaxDuration) {
        m_foundFace = false;
        m_templateMatchingRunning = false;
        m_templateMatchingStartTime = m_templateMatchingCurrentTime = 0;
		m_facePosition.x = m_facePosition.y = 0;
		m_trackedFace.x = m_trackedFace.y = m_trackedFace.width = m_trackedFace.height = 0;
		return;
    }

	// Edge case when face exits frame while 
	if (m_faceTemplate.rows * m_faceTemplate.cols == 0 || m_faceTemplate.rows <= 1 || m_faceTemplate.cols <= 1) {
		m_foundFace = false;
		m_templateMatchingRunning = false;
		m_templateMatchingStartTime = m_templateMatchingCurrentTime = 0;
		m_facePosition.x = m_facePosition.y = 0;
		m_trackedFace.x = m_trackedFace.y = m_trackedFace.width = m_trackedFace.height = 0;
		return;
	}

    // Template matching with last known face 
    //cv::matchTemplate(frame(m_faceRoi), m_faceTemplate, m_matchingResult, CV_TM_CCOEFF);
    cv::matchTemplate(frame(m_faceRoi), m_faceTemplate, m_matchingResult, CV_TM_SQDIFF_NORMED);
    cv::normalize(m_matchingResult, m_matchingResult, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
    double min, max;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc(m_matchingResult, &min, &max, &minLoc, &maxLoc);

    // Add roi offset to face position
    minLoc.x += m_faceRoi.x;
    minLoc.y += m_faceRoi.y;

    // Get detected face
    //m_trackedFace = cv::Rect(maxLoc.x, maxLoc.y, m_trackedFace.width, m_trackedFace.height);
    m_trackedFace = cv::Rect(minLoc.x, minLoc.y, m_faceTemplate.cols, m_faceTemplate.rows);
    m_trackedFace = doubleRectSize(m_trackedFace, cv::Rect(0, 0, frame.cols, frame.rows));

    // Get new face template
    m_faceTemplate = getFaceTemplate(frame, m_trackedFace);

    // Calculate face roi
    m_faceRoi = doubleRectSize(m_trackedFace, cv::Rect(0, 0, frame.cols, frame.rows));

    // Update face position
    m_facePosition = centerOfRect(m_trackedFace);
}

void HeadPoseEstimator::detectFaces(const cv::Mat &frame, const bool isFirstFrame)
{
    if (isHaarLoaded == false)
    {
        m_foundFace = false;
        return;
    }

    // Downscale frame to m_resizedWidth width - keep aspect ratio
    m_scale = (double) std::min(m_resizedWidth, frame.cols) / frame.cols;
    cv::Size resizedFrameSize = cv::Size((int)(m_scale*frame.cols), (int)(m_scale*frame.rows));

    cv::Mat resizedFrame;
    cv::resize(frame, resizedFrame, resizedFrameSize);

    // actually to be able use linear algorithm in Kalman Filter we need to use only one
    // of approaches for tracking and do not mix them.
    detectFaceAllSizes(resizedFrame); // Detect using cascades over whole image
    /*
    if (!m_foundFace || isFirstFrame)
        detectFaceAllSizes(resizedFrame); // Detect using cascades over whole image
    else {
        detectFaceAroundRoi(resizedFrame); // Detect using cascades only in ROI
        if (m_templateMatchingRunning && false) {
            detectFacesTemplateMatching(resizedFrame); // Detect using template matching
        }
    }
    */
}

void HeadPoseEstimator::update(const cv::InputArray _image, const bool isFirstFrame)
{
    const cv::Mat image = _image.getMat();

    if (opticalCenterX == -1) // not initialized yet
    {
        opticalCenterX = image.cols / 2;
        opticalCenterY = image.rows / 2;
#ifdef HEAD_POSE_ESTIMATOR_DEBUG
        std::cerr << "Setting the optical center to (" << opticalCenterX << ", " << opticalCenterY << ")" << std::endl;
#endif
    }

    current_image = dlib::cv_image<dlib::bgr_pixel>(image);
    /*
    dlib::array2d<dlib::rgb_pixel> img;
    dlib::assign_image(img, cv_image<bgr_pixel>(image));
    // Make the image larger so we can detect small faces.
    pyramid_up(img);
    */

    detectFaces(image, isFirstFrame);
    double precTick = m_ticks_kf;
    m_ticks_kf = (double) cv::getTickCount();
    double dT = (m_ticks_kf - precTick) / TICK_FREQUENCY; //seconds
    if (isFirstFrame) // initialize
    {
        dT = 0.1;
        lastResult_kf = false;
    }
    cv::Rect faceRect;
    if (m_foundFace)
    {
        cv::Rect predRect;
        if (lastResult_kf == true)
        {
            // >>>> Matrix A
            kf->transitionMatrix.at<float>(2) = dT;
            kf->transitionMatrix.at<float>(9) = dT;
            // <<<< Matrix A

            //cout << "dT:" << endl << dT << endl;

            state = kf->predict();
            //cout << "State post:" << endl << state << endl;

            
            predRect.width = state.at<float>(4);
            predRect.height = state.at<float>(5);
            predRect.x = state.at<float>(0) - predRect.width / 2;
            predRect.y = state.at<float>(1) - predRect.height / 2;
            //
            //
            //
            //predRect.x += predRect.width /16;
            //predRect.width -= predRect.width /16 * 2;
        }

        faceRect = m_trackedFace;
        faceRect.x = (int)(faceRect.x / m_scale);
        faceRect.y = (int)(faceRect.y / m_scale);
        faceRect.width = (int)(faceRect.width / m_scale);
        faceRect.height = (int)(faceRect.height / m_scale);
        

        faces.clear();
        if (lastResult_kf == true)
        {
            faces.push_back(dlib::rectangle(predRect.x,predRect.y, predRect.x+predRect.width, predRect.y+predRect.height));
        }
        else
        {
            faces.push_back(dlib::rectangle(faceRect.x,faceRect.y, faceRect.x+faceRect.width, faceRect.y+faceRect.height));
        }
    }
    else
    {
        //
        // todo: it coule be speed up if use face(landmarks or else) tracker rather detector.
        // read post and comments espacially
        // https://www.learnopencv.com/facial-landmark-detection/
        // http://www.learnopencv.com/object-tracking-using-opencv-cpp-python/
        //
        const double thresshold = 0.0; // ros: -0.5 allows more roughly (rotated) faces but with error
        faces = detector(current_image, thresshold); 

        if (faces.size() > 0)
        {
            faceRect.x = faces.front().left();
            faceRect.y = faces.front().top();
            faceRect.width = faces.front().right() - faces.front().left();
            faceRect.height = faces.front().bottom() - faces.front().top();
        }
    }

    if (faces.size() > 0)
    {
        meas.at<float>(0) = faceRect.x + faceRect.width / 2;
        meas.at<float>(1) = faceRect.y + faceRect.height / 2;
        meas.at<float>(2) = faceRect.width;
        meas.at<float>(3) = faceRect.height;

        if (lastResult_kf == false)
        {
            // >>>> Initialization
            kf->errorCovPre.at<float>(0) = 1; // px
            kf->errorCovPre.at<float>(7) = 1; // px
            kf->errorCovPre.at<float>(14) = 1;
            kf->errorCovPre.at<float>(21) = 1;
            kf->errorCovPre.at<float>(28) = 1; // px
            kf->errorCovPre.at<float>(35) = 1; // px

            state.at<float>(0) = meas.at<float>(0);
            state.at<float>(1) = meas.at<float>(1);
            state.at<float>(2) = 0;
            state.at<float>(3) = 0;
            state.at<float>(4) = meas.at<float>(2);
            state.at<float>(5) = meas.at<float>(3);
            // <<<< Initialization

            kf->statePost = state;
        }
        else
        {
            kf->correct(meas); // Kalman Correction
        }

        lastResult_kf = true;
    }
    else
    {
        lastResult_kf = false;
    }

    // Find the pose of each face.
    shapes.clear();

    std::vector<dlib::rectangle>::iterator  it = faces.begin();
    std::vector<dlib::rectangle>::iterator  ite = faces.end();
    for (; it != ite; ++it) {
        dlib::rectangle & face = *it;
        shapes.push_back(pose_model(current_image, face));
    }

#ifdef HEAD_POSE_ESTIMATOR_DEBUG
    // Draws the contours of the face and face features onto the image
    
    _debug = image.clone();

    auto color = cv::Scalar(0,128,128);
    auto color_blue = cv::Scalar(128,0,0);
    auto color_green = cv::Scalar(0,128,0);
    auto color_red = cv::Scalar(0,0,128);

    for (unsigned long i = 0; i < shapes.size(); ++i)
    {
        const dlib::full_object_detection & d = shapes[i];

        if (m_FaceMesh.empty())
        {
            for (unsigned long i = 1; i <= 16; ++i)
                line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, CV_AA); // left eye

            for (unsigned long i = 28; i <= 30; ++i)
                line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color_green, 2, CV_AA);

            for (unsigned long i = 18; i <= 21; ++i)
                line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, CV_AA);
            for (unsigned long i = 23; i <= 26; ++i)
                line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, CV_AA);
            for (unsigned long i = 31; i <= 35; ++i)
                line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, CV_AA);
            line(_debug, toCv(d.part(30)), toCv(d.part(35)), color, 2, CV_AA);

            for (unsigned long i = 37; i <= 41; ++i)
                line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color, 2, CV_AA); // right eye
            line(_debug, toCv(d.part(36)), toCv(d.part(41)), color, 2, CV_AA);

            for (unsigned long i = 43; i <= 47; ++i)
                line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color_blue, 2, CV_AA); // chest
            line(_debug, toCv(d.part(42)), toCv(d.part(47)), color, 2, CV_AA);

            for (unsigned long i = 49; i <= 59; ++i)
                line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color_red, 2, CV_AA);
            line(_debug, toCv(d.part(48)), toCv(d.part(59)), color_red, 2, CV_AA);

            for (unsigned long i = 61; i <= 67; ++i)
                line(_debug, toCv(d.part(i)), toCv(d.part(i-1)), color_red, 2, CV_AA);
            line(_debug, toCv(d.part(60)), toCv(d.part(67)), color_red, 2, CV_AA);

            /* ros: ros2
            for (_ULonglong i = 0; i < 68 ; i++) {
                putText(_debug, std::to_string(i), toCv(d.part(i)), FONT_HERSHEY_DUPLEX, 0.6, Scalar(255,255,255));
            }
            */
        }
        else
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

                    line(_debug, toCv(d.part(v0)), toCv(d.part(v1)), color_green, 1, CV_AA);
                }
            }
        }
    }
    
#endif
}

head_pose HeadPoseEstimator::calc_pose(const EstimatorSourceData & srcData) const
{
    cv::Mat         projectionMat       = cv::Mat::zeros(3,3,CV_32F);
    cv::Matx33f     projection          = projectionMat;
    //
    projection(0,0) = focalLength;
    projection(1,1) = focalLength;
    projection(0,2) = opticalCenterX;
    projection(1,2) = opticalCenterY;
    projection(2,2) = 1;
    //
    cv::Mat         rvec, tvec;
    cv::Matx33d     rotation;


    // Find the 3D pose of our head
#if 1 // This case does not control the quality, but fast and averagly not a bad result
    cv::solvePnP(srcData.getObjectPoints(), srcData.getImagePoints(),
                projection, cv::noArray(),
                rvec, tvec, false,
#if CV_VERSION_MAJOR >= 3
            cv::SOLVEPNP_ITERATIVE);
#else
            cv::ITERATIVE);
#endif

#else // 1 
    // Here quality could be controlled, but parameters(and their values) should be researched
    int iterationsCount = 1000;
    float reprojectionError = 20;//8.0
    int minInliersCount = 1000;
    // Find the 3D pose of our head
    cv::solvePnPRansac(head_points3df, detected_points2df,
            projection, cv::noArray(),
            rvec, tvec, false,

            iterationsCount,
            reprojectionError,
            minInliersCount,
            cv::noArray(),
#if CV_VERSION_MAJOR >= 3
            cv::SOLVEPNP_ITERATIVE);
#else
            cv::ITERATIVE);
#endif
#endif // 1
/*
    {
//        tvec.create(3, 1, CV_64F);

        size_t  i,j;
        std::vector<CvPoint3D32f> modelPoints;
        std::vector<CvPoint2D32f> srcImagePoints;

        for (i = 0; i < head_points3df.size(); ++i)
            modelPoints.push_back(cvPoint3D32f(head_points3df[i].x, head_points3df[i].y, head_points3df[i].z));

        for (i = 0; i < detected_points2df.size(); ++i)
            srcImagePoints.push_back( cvPoint2D32f( detected_points2df[i].x, detected_points2df[i].y ) );

        CvPOSITObject *positObject = cvCreatePOSITObject( &modelPoints[0], static_cast<int>(modelPoints.size()) );

        //Estimate the pose
        float * rotation_matrix = new float[9];
        float * translation_vector = new float[3];

        CvTermCriteria criteria = cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 100, 1.0e-4f);

        cvPOSIT( positObject, &srcImagePoints[0], focalLength, criteria, rotation_matrix, translation_vector );

        for (i = 0; i < 3; ++i)
        {
            for (j = 0; j < 3; ++j)
            {
                rotation(i,j) = rotation_matrix[i*3 + j];
            }
            tvec.at<double>(i) = translation_vector[i];
        }

        cvReleasePOSITObject(&positObject);
        delete rotation_matrix;
        delete translation_vector;
    }
    */

    Rodrigues(rvec, rotation);

    head_pose pose = head_pose(
        rotation(0,0),    rotation(0,1),    rotation(0,2),    tvec.at<double>(0)/1000,
        rotation(1,0),    rotation(1,1),    rotation(1,2),    tvec.at<double>(1)/1000,
        rotation(2,0),    rotation(2,1),    rotation(2,2),    tvec.at<double>(2)/1000,
                    0,                0,                0,                     1);

    return pose;
}

head_pose HeadPoseEstimator::calc_pose(size_t face_idx) const
{
    cv::Mat         projectionMat       = cv::Mat::zeros(3,3,CV_32F);
    cv::Matx33f     projection          = projectionMat;

    projection(0,0) = focalLength;
    projection(1,1) = focalLength;
    projection(0,2) = opticalCenterX;
    projection(1,2) = opticalCenterY;
    projection(2,2) = 1;

    std::vector<cv::Point3f> head_points3df;

    head_points3df.push_back(P3D_SELLION);
    head_points3df.push_back(P3D_RIGHT_EYE);
    head_points3df.push_back(P3D_LEFT_EYE);
    head_points3df.push_back(P3D_RIGHT_EAR);
    head_points3df.push_back(P3D_LEFT_EAR);
    head_points3df.push_back(P3D_MENTON);
    head_points3df.push_back(P3D_NOSE);
    head_points3df.push_back(P3D_STOMMION);

    std::vector<cv::Point2f> detected_points2df;

    detected_points2df.push_back(coordsOf(face_idx, SELLION));
    detected_points2df.push_back(coordsOf(face_idx, RIGHT_EYE));
    detected_points2df.push_back(coordsOf(face_idx, LEFT_EYE));
    detected_points2df.push_back(coordsOf(face_idx, RIGHT_SIDE));
    detected_points2df.push_back(coordsOf(face_idx, LEFT_SIDE));
    detected_points2df.push_back(coordsOf(face_idx, MENTON));
    detected_points2df.push_back(coordsOf(face_idx, NOSE));

    auto stomion = (coordsOf(face_idx, MOUTH_CENTER_TOP) + coordsOf(face_idx, MOUTH_CENTER_BOTTOM)) * 0.5;
    detected_points2df.push_back(stomion);

    cv::Mat rvec, tvec;
    cv::Matx33d rotation;


    // Find the 3D pose of our head
#if 1 // This case does not control the quality, but fast and averagly not a bad result
    cv::solvePnP(head_points3df, detected_points2df,
                projection, cv::noArray(),
                rvec, tvec, false,
#if CV_VERSION_MAJOR >= 3
            cv::SOLVEPNP_ITERATIVE);
#else
            cv::ITERATIVE);
#endif
#else // Here quality could be controlled, but parameters(and their values) should be researched
    int iterationsCount = 1000;
    float reprojectionError = 20;//8.0
    int minInliersCount = 1000;
    // Find the 3D pose of our head
    cv::solvePnPRansac(head_points3df, detected_points2df,
            projection, cv::noArray(),
            rvec, tvec, false,

            iterationsCount,
            reprojectionError,
            minInliersCount,
            cv::noArray(),
#if CV_VERSION_MAJOR >= 3
            cv::SOLVEPNP_ITERATIVE);
#else
            cv::ITERATIVE);
#endif
#endif // 1
/*
    {
//        tvec.create(3, 1, CV_64F);

        size_t  i,j;
        std::vector<CvPoint3D32f> modelPoints;
        std::vector<CvPoint2D32f> srcImagePoints;

        for (i = 0; i < head_points3df.size(); ++i)
            modelPoints.push_back(cvPoint3D32f(head_points3df[i].x, head_points3df[i].y, head_points3df[i].z));

        for (i = 0; i < detected_points2df.size(); ++i)
            srcImagePoints.push_back( cvPoint2D32f( detected_points2df[i].x, detected_points2df[i].y ) );

        CvPOSITObject *positObject = cvCreatePOSITObject( &modelPoints[0], static_cast<int>(modelPoints.size()) );

        //Estimate the pose
        float * rotation_matrix = new float[9];
        float * translation_vector = new float[3];

        CvTermCriteria criteria = cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 100, 1.0e-4f);

        cvPOSIT( positObject, &srcImagePoints[0], focalLength, criteria, rotation_matrix, translation_vector );

        for (i = 0; i < 3; ++i)
        {
            for (j = 0; j < 3; ++j)
            {
                rotation(i,j) = rotation_matrix[i*3 + j];
            }
            tvec.at<double>(i) = translation_vector[i];
        }

        cvReleasePOSITObject(&positObject);
        delete rotation_matrix;
        delete translation_vector;
    }
    */

    Rodrigues(rvec, rotation);

    head_pose pose = head_pose(
        rotation(0,0),    rotation(0,1),    rotation(0,2),    tvec.at<double>(0)/1000,
        rotation(1,0),    rotation(1,1),    rotation(1,2),    tvec.at<double>(1)/1000,
        rotation(2,0),    rotation(2,1),    rotation(2,2),    tvec.at<double>(2)/1000,
                    0,                0,                0,                     1);

#ifdef HEAD_POSE_ESTIMATOR_DEBUG
    /*
    {
        std::vector<cv::Point3f> axes;
        axes.push_back(cv::Point3f(0,0,0));
        axes.push_back(cv::Point3f(50,0,0));
        axes.push_back(cv::Point3f(0,50,0));
        axes.push_back(cv::Point3f(0,0,50));
        std::vector<cv::Point2f> projected_axes;

        for (size_t i = 0; i < axes.size(); ++i)
        {
            cv::Matx41d vec(axes[i].x, axes[i].y, axes[i].z, 1.0);
            cv::Matx41d vec_in_cam_cs = pose * vec;
            cv::Matx31f vec_in_cam_cs_f(vec_in_cam_cs(0), vec_in_cam_cs(1), vec_in_cam_cs(2));
            cv::Matx31f proj = projection * vec_in_cam_cs_f;
            //
            projected_axes.push_back(cv::Point2f(proj(0), proj(1)));
        }
        line(_debug, projected_axes[0], projected_axes[3], cv::Scalar(255,0,0),2,CV_AA);
        line(_debug, projected_axes[0], projected_axes[2], cv::Scalar(0,255,0),2,CV_AA);
        line(_debug, projected_axes[0], projected_axes[1], cv::Scalar(0,0,255),2,CV_AA);
    }
    */
    
    if (m_FaceMesh.empty())
    {
        std::vector<cv::Point2f> reprojected_points;

        // projects points from the model coordinate space to the image coordinates.
        // Also computes derivatives of the image coordinates w.r.t the intrinsic and extrinsic camera parameters
        cv::projectPoints(head_points3df, rvec, tvec, projection, cv::noArray(), reprojected_points);

    
        std::vector<cv::Point2f>::iterator it = reprojected_points.begin();
        std::vector<cv::Point2f>::iterator ite = reprojected_points.end();
        for (; it != ite; ++it) {
            cv::Point2f & point = *it;
            circle(_debug, point,2, cv::Scalar(0,255,255),2);
        }
    

        std::vector<cv::Point3f> axes;
        axes.push_back(cv::Point3f(0,0,0));
        axes.push_back(cv::Point3f(50,0,0));
        axes.push_back(cv::Point3f(0,50,0));
        axes.push_back(cv::Point3f(0,0,50));
        std::vector<cv::Point2f> projected_axes;

        // projects points from the model coordinate space to the image coordinates.
        // Also computes derivatives of the image coordinates w.r.t the intrinsic and extrinsic camera parameters
        cv::projectPoints(axes, rvec, tvec, projection, cv::noArray(), projected_axes);

        line(_debug, projected_axes[0], projected_axes[3], cv::Scalar(255,0,0),2,CV_AA);
        line(_debug, projected_axes[0], projected_axes[2], cv::Scalar(0,255,0),2,CV_AA);
        line(_debug, projected_axes[0], projected_axes[1], cv::Scalar(0,0,255),2,CV_AA);

        // Sellion position
        putText(_debug, "(" + std::to_string(_Longlong(pose(0,3) * 100)) + "cm, " + std::to_string(_Longlong(pose(1,3) * 100)) + "cm, " + std::to_string(_Longlong(pose(2,3) * 100)) + "cm)", coordsOf(face_idx, SELLION), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255),2);

        for (int i = 0; i < faces.size(); ++i)
        {
            const dlib::rectangle & r = faces[i];

            line(_debug, cv::Point2f(r.left(), r.top()), cv::Point2f(r.right(), r.top()), cv::Scalar(255,0,0),2,CV_AA);
            line(_debug, cv::Point2f(r.left(), r.bottom()), cv::Point2f(r.right(), r.bottom()), cv::Scalar(255,0,0),2,CV_AA);
            line(_debug, cv::Point2f(r.left(), r.top()), cv::Point2f(r.left(), r.bottom()), cv::Scalar(255,0,0),2,CV_AA);
            line(_debug, cv::Point2f(r.right(), r.top()), cv::Point2f(r.right(), r.bottom()), cv::Scalar(255,0,0),2,CV_AA);
        }
    }

#endif

    return pose;
}

const size_t
HeadPoseEstimator::getShapesNb() const
{
    return shapes.size();
}

const dlib::full_object_detection &
HeadPoseEstimator::getShape(const size_t & index) const
{
    return shapes[index];
}

std::vector<head_pose> HeadPoseEstimator::poses() const {
    
    std::vector<head_pose> res;

    for (auto i = 0; i < faces.size(); i++){
        res.push_back(calc_pose(i));
    }

    return res;

}

cv::Point2f HeadPoseEstimator::coordsOf(size_t face_idx, FACIAL_FEATURE feature) const
{
    return toCv(shapes[face_idx].part(feature));
}

// Finds the intersection of two lines, or returns false.
// The lines are defined by (o1, p1) and (o2, p2).
// taken from: http://stackoverflow.com/a/7448287/828379
bool HeadPoseEstimator::intersection(cv::Point2f o1, cv::Point2f p1, cv::Point2f o2, cv::Point2f p2,
                                      cv::Point2f &r,
                                      const float & eps) const
{
    cv::Point2f x = o2 - o1;
    cv::Point2f d1 = p1 - o1;
    cv::Point2f d2 = p2 - o2;

    float cross = d1.x*d2.y - d1.y*d2.x;
    if (abs(cross) < eps)
        return false;

    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    r = o1 + d1 * t1;
    return true;
}

