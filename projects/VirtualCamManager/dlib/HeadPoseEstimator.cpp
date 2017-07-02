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


HeadPoseEstimator::HeadPoseEstimator():
        IHeadEstimator(),
        focalLength(350),
        opticalCenterX(-1),
        opticalCenterY(-1)
{
	Load("shape_predictor_68_face_landmarks.dat", 554, "haarcascade_frontalface_default.xml");
}

void
HeadPoseEstimator::Load(const std::string& face_detection_model, float focalLength_, const std::string cascadeFilePath) 
{
	this->focalLength = focalLength_;

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

    //
    // todo: it coule be speed up if use face(landmarks or else) tracker rather detector.
    // read post and comments espacially
    // https://www.learnopencv.com/facial-landmark-detection/
    // http://www.learnopencv.com/object-tracking-using-opencv-cpp-python/
    //
    const double thresshold = 0.0; // ros: -0.5 allows more roughly (rotated) faces but with error
    faces = detector(current_image, thresshold); 

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


