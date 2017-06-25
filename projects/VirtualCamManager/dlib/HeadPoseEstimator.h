#ifndef __GUARD_HEADPOSE_ESTIMATOR_H__
#define __GUARD_HEADPOSE_ESTIMATOR_H__

#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/video/tracking.hpp>

#include <dlib/opencv.h>
#include <dlib/image_processing.h>
#include <dlib/image_processing/frontal_face_detector.h>

#include "EstimatorSourceData.h"
#include "../IHeadEstimator.h"

#include <vector>
#include <array>
#include <string>


// Anthropometric for male adult
// Relative position of various facial feature relative to sellion
// Values taken from https://en.wikipedia.org/wiki/Human_head
// X points forward
const static cv::Point3f P3D_SELLION(0., 0.,0.); // переносица
const static cv::Point3f P3D_RIGHT_EYE(-20., -65.5, -5.); // ros: was -20., -65.5,-5.
const static cv::Point3f P3D_LEFT_EYE(-20., 65.5, -5.); // ros: was -20., 65.5,-5.
const static cv::Point3f P3D_RIGHT_EAR(-100., -77.5,-6.);
const static cv::Point3f P3D_LEFT_EAR(-100., 77.5,-6.);
const static cv::Point3f P3D_NOSE(21.0, 0., -48.0);
const static cv::Point3f P3D_STOMMION(10.0, 0., -75.0);
const static cv::Point3f P3D_MENTON(0., 0.,-133.0); // подбородок



static const int MAX_FEATURES_TO_TRACK=100;

//std::string to_string(const int i);

// Interesting facial features with their landmark index
enum FACIAL_FEATURE {
    NOSE=30,
    RIGHT_EYE=36,
    LEFT_EYE=45,
    RIGHT_SIDE=0,
    LEFT_SIDE=16,
    EYEBROW_RIGHT=21,
    EYEBROW_LEFT=22,
    MOUTH_UP=51,
    MOUTH_DOWN=57,
    MOUTH_RIGHT=48,
    MOUTH_LEFT=54,
    SELLION=27,
    MOUTH_CENTER_TOP=62,
    MOUTH_CENTER_BOTTOM=66,
    MENTON=8
};



class HeadPoseEstimator : public IHeadEstimator
{
public:

	HeadPoseEstimator();

	virtual void Initialize(const float & focalLength, const float & opticalCenterX, const float & opticalCenterY);

    virtual ~HeadPoseEstimator();
    virtual void update(const cv::InputArray image, const bool isFirstFrame);

    virtual head_pose calc_pose(size_t face_idx) const;
    head_pose calc_pose(const EstimatorSourceData & ) const;

	virtual const size_t    getShapesNb() const;
    virtual const dlib::full_object_detection & getShape(const size_t & index) const;

    std::vector<head_pose> poses() const;

    float focalLength;
    float opticalCenterX;
    float opticalCenterY;


private:
    void Load(const std::string& face_detection_model,
						float focalLength,
                        const std::string cascadeFilePath);

    dlib::cv_image<dlib::bgr_pixel> current_image;

    dlib::frontal_face_detector detector;
    dlib::shape_predictor pose_model;

    std::vector<dlib::rectangle> faces;

    std::vector<dlib::full_object_detection> shapes;


    /** Return the point corresponding to the dictionary marker.
    */
    cv::Point2f coordsOf(size_t face_idx, FACIAL_FEATURE feature) const;

    /** Returns true if the lines intersect (and set r to the intersection
     *  coordinates), false otherwise.
     */
    bool intersection(cv::Point2f o1, cv::Point2f p1,
                      cv::Point2f o2, cv::Point2f p2,
                      cv::Point2f &r,
                      const float & eps = std::numeric_limits<float>::min()/*ros: was 1.e-8*/) const;

    // Haar cascade frontal face detection routines
    void setFaceCascade(const std::string cascadeFilePath);
    void detectFaces(const cv::Mat &frame, const bool isFirstFrame);
    void detectFaceAllSizes(const cv::Mat &frame);
    void detectFaceAroundRoi(const cv::Mat &frame);
    void detectFacesTemplateMatching(const cv::Mat &frame);
    cv::Mat HeadPoseEstimator::getFaceTemplate(const cv::Mat &frame, cv::Rect face);
    cv::Rect HeadPoseEstimator::doubleRectSize(const cv::Rect &inputRect, const cv::Rect &frameSize) const;
    cv::Point HeadPoseEstimator::centerOfRect(const cv::Rect &rect) const;
    static const double TICK_FREQUENCY;
    bool isHaarLoaded;
    double m_templateMatchingMaxDuration;
    cv::CascadeClassifier* m_faceCascade;
    cv::Mat m_matchingResult;
    std::vector<cv::Rect> m_allFaces;
    cv::Rect m_trackedFace;
    cv::Rect m_faceRoi;
    cv::Mat m_faceTemplate;
    cv::Point m_facePosition;
    int64 m_templateMatchingStartTime;
    int64 m_templateMatchingCurrentTime;
    int m_resizedWidth;
    double m_scale;
    bool m_foundFace;
    bool m_templateMatchingRunning;
    // >>>> Kalman Filter
    cv::KalmanFilter * kf;
    bool lastResult_kf;
    double m_ticks_kf;
    cv::Mat state;
    cv::Mat meas;
    // >>>>
    // https://sourceforge.net/p/dclib/discussion/442518/thread/a21dd8fc/?limit=25#824e
    // http://dlib.net/video_tracking_ex.cpp.html
    dlib::correlation_tracker tracker;
};


#endif // __GUARD_HEADPOSE_ESTIMATOR_H__