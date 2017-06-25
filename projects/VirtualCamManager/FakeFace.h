#pragma once

#ifdef HEAD_POSE_ESTIMATOR_DEBUG

#include <opencv2/core/core.hpp>
#include "IHeadEstimator.h"


class CFakeFace
{
    cv::Mat                                     m_imgOrig;
    cv::Mat                                     m_img32f;
    dlib::full_object_detection                 m_landmarkContainer;
public:
    CFakeFace();
    ~CFakeFace();

    const int   Initialize(IHeadEstimator * estimator, const char * pFaceImageFile);
    const bool  IsInitialized() const;
    const dlib::full_object_detection & GetLandmars() const;
//    const cv::Mat   GetImgOrig() const;
    const cv::Mat   GetImg32f() const;
};

// Warps and alpha blends triangular regions from img1 and img2 to img
void
morphTriangle(const cv::Mat & img1, const cv::Mat &img2, cv::Mat &img,
                std::vector<cv::Point2f> &t1,
                std::vector<cv::Point2f> &t2,
                std::vector<cv::Point2f> &t,
                const double & alpha);
void PoissonBlend(const cv::Mat & base, const cv::Mat & src, const cv::Mat & mask, cv::Mat & res1, const cv::Rect & roi);


#endif // HEAD_POSE_ESTIMATOR_DEBUG