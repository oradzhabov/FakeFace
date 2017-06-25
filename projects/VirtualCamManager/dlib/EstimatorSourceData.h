#pragma once
#include <opencv2/core/core.hpp>
#include <dlib/opencv.h>
#include <dlib/image_processing.h>
#include <dlib/image_processing/frontal_face_detector.h>

#include <vector>


class EstimatorSourceData
{
    std::vector<cv::Point3f> head_points3df;
    std::vector<cv::Point2f> detected_points2df;

public:

    EstimatorSourceData();
    ~EstimatorSourceData();

    const cv::InputArray & getObjectPoints() const;
    const cv::InputArray & getImagePoints() const;

    static void createByRecognition (EstimatorSourceData & result, const dlib::full_object_detection & );
};

