#include "EstimatorSourceData.h"
#include "HeadPoseEstimator.h" // for constants


EstimatorSourceData::EstimatorSourceData()
{
}


EstimatorSourceData::~EstimatorSourceData()
{
}

const cv::InputArray &
EstimatorSourceData::getObjectPoints() const
{
    return head_points3df;
}

const cv::InputArray &
EstimatorSourceData::getImagePoints() const
{
    return detected_points2df;
}

void
EstimatorSourceData::createByRecognition(EstimatorSourceData & result, const dlib::full_object_detection & shape)
{
    result.head_points3df.clear();
    result.detected_points2df.clear();

    result.head_points3df.push_back(P3D_SELLION);
    result.head_points3df.push_back(P3D_RIGHT_EYE);
    result.head_points3df.push_back(P3D_LEFT_EYE);
    result.head_points3df.push_back(P3D_RIGHT_EAR);
    result.head_points3df.push_back(P3D_LEFT_EAR);
    result.head_points3df.push_back(P3D_MENTON);
    result.head_points3df.push_back(P3D_NOSE);
    result.head_points3df.push_back(P3D_STOMMION);

    result.detected_points2df.push_back(toCv(shape.part(SELLION)));
    result.detected_points2df.push_back(toCv(shape.part(RIGHT_EYE)));
    result.detected_points2df.push_back(toCv(shape.part(LEFT_EYE)));
    result.detected_points2df.push_back(toCv(shape.part(RIGHT_SIDE)));
    result.detected_points2df.push_back(toCv(shape.part(LEFT_SIDE)));
    result.detected_points2df.push_back(toCv(shape.part(MENTON)));
    result.detected_points2df.push_back(toCv(shape.part(NOSE)));

    auto stomion = (toCv(shape.part(MOUTH_CENTER_TOP)) + toCv(shape.part(MOUTH_CENTER_BOTTOM))) * 0.5;
    result.detected_points2df.push_back(stomion);
}