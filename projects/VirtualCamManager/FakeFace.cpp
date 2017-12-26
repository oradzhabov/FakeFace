#ifdef HEAD_POSE_ESTIMATOR_DEBUG

#include "FakeFace.h"
#include <opencv2/opencv.hpp>

#ifdef USE_PPL
	#include <ppl.h>
#endif // USE_PPL

CFakeFace::CFakeFace()
{
}


CFakeFace::~CFakeFace()
{
}

const bool
CFakeFace::IsInitialized() const
{
    return m_landmarkContainer.num_parts() > 0;
}

const dlib::full_object_detection &
CFakeFace::GetLandmarks() const
{
    return m_landmarkContainer;
}

const cv::Mat &
CFakeFace::GetImg32f() const
{
    return m_img32f;
}

const int
CFakeFace::Initialize(IHeadEstimator * estimator, const char * pFaceImageFile)
{
    m_landmarkContainer = dlib::full_object_detection(); // clear

    if (pFaceImageFile != NULL)
        m_imgOrig = cv::imread(pFaceImageFile);

    if (m_imgOrig.empty())
        return -1;

	// cx = 640/2 = 320
	// cy = 480/2 = 240
	// fx = fy = cx/tan(60/2 * pi / 180) = 554.26
    float cx = m_imgOrig.cols / 2;
    float cy = m_imgOrig.rows / 2;
    float fx = cx/tan(60/2 * 3.14159265 / 180);
    estimator->Initialize(fx,cx,cy);
    estimator->update(m_imgOrig, true);


    // if any result has been obtained
    if (estimator->getShapesNb() > 0)
    {
        m_imgOrig.convertTo(m_img32f, CV_32F);

        m_landmarkContainer = estimator->getShape(0);

        return 0;
    }

    return -1;
}

// Apply affine transform calculated using srcTri and dstTri to src
void
applyAffineTransform(const cv::Mat &warpImage, const cv::Mat &src, std::vector<cv::Point2f> &srcTri, std::vector<cv::Point2f> &dstTri)
{
    
    // Given a pair of triangles, find the affine transform.
    cv::Mat warpMat = cv::getAffineTransform( srcTri, dstTri );
    
    // Apply the Affine Transform just found to the src image
    cv::warpAffine( src, warpImage, warpMat, warpImage.size(), cv::INTER_LINEAR, cv::BORDER_REFLECT_101);
}

// Warps and alpha blends triangular regions from img1 and img2 to img
void
morphTriangle(const cv::Mat & img1, const cv::Mat &img2, cv::Mat &img,
                std::vector<cv::Point2f> &t1,
                std::vector<cv::Point2f> &t2,
                std::vector<cv::Point2f> &t,
                const double & alpha)
{
    
    // Find bounding rectangle for each triangle
    cv::Rect r = cv::boundingRect(t);
    cv::Rect r1 = cv::boundingRect(t1);
    cv::Rect r2 = cv::boundingRect(t2);
    
    // Offset points by left top corner of the respective rectangles
    std::vector<cv::Point2f> t1Rect(3), t2Rect(3), tRect(3);
    std::vector<cv::Point> tRectInt(3);
    for(int i = 0; i < 3; i++)
    {
        tRect[i] = cv::Point2f( t[i].x - r.x, t[i].y - r.y);
        tRectInt[i] = cv::Point(t[i].x - r.x, t[i].y - r.y); // for fillConvexPoly
        
        t1Rect[i] = cv::Point2f( t1[i].x - r1.x, t1[i].y - r1.y);
        t2Rect[i] = cv::Point2f( t2[i].x - r2.x, t2[i].y - r2.y);
    }
    
    // Get mask by filling triangle
    cv::Mat mask = cv::Mat::zeros(r.height, r.width, CV_32FC3);
    cv::fillConvexPoly(mask, tRectInt, cv::Scalar(1.0, 1.0, 1.0), 16, 0);
    
    // Apply warpImage to small rectangular patches
    cv::Mat img1Rect, img2Rect;
    img1(r1).copyTo(img1Rect);
    img2(r2).copyTo(img2Rect);
    
    cv::Mat warpImage1 = cv::Mat::zeros(r.height, r.width, img1Rect.type());
    cv::Mat warpImage2 = cv::Mat::zeros(r.height, r.width, img2Rect.type());
    
    applyAffineTransform(warpImage1, img1Rect, t1Rect, tRect);
    applyAffineTransform(warpImage2, img2Rect, t2Rect, tRect);
    
    // Alpha blend rectangular patches
    cv::Mat imgRect = (1.0 - alpha) * warpImage1 + alpha * warpImage2;
    
    // Copy triangular region of the rectangular patch to the output image
    cv::multiply(imgRect,mask, imgRect);
    cv::multiply(img(r), cv::Scalar(1.0,1.0,1.0) - mask, img(r));
    img(r) = img(r) + imgRect;
}

// source:
// https://www.shadertoy.com/view/4l3Xzl#
//
void PoissonBlend(const cv::Mat & base, const cv::Mat & src, const cv::Mat & mask, cv::Mat & res1, const cv::Rect & roi)
{
    base.copyTo(res1);

    cv::Mat res2 = res1.clone();

    const int d = 3;

    for (int iter = 0; iter < 15/*odd !*/; ++iter)
    {
        cv::Mat & res_prev = (iter % 2) == 0 ? res2 : res1;
        cv::Mat & res = (iter % 2) == 0 ? res1 : res2;
        
#ifdef USE_PPL
        Concurrency::parallel_for ((int)(roi.y), (int)(roi.y + roi.height), [&](int i){
#else
        for(int i = roi.y; i < roi.y + roi.height; ++i){
#endif // USE_PPL
            cv::Point neighbours[4];
            for(int j = roi.x; j < roi.x + roi.width; ++j)
            {
                const cv::Point p (j,i);
                if(mask.at<cv::Vec3b>(p)[0] == 255)
                {
                    neighbours[0] = cv::Point(j-d,i);
                    neighbours[1] = cv::Point(j+d,i);
                    neighbours[2] = cv::Point(j,i-d);
                    neighbours[3] = cv::Point(j,i+d);

                    cv::Vec3f col = res_prev.at<cv::Vec3b>(p);
                    //cv::Vec3f col (0,0,0);
                    for (int n = 0; n < 4; ++n)
                    {
                        const cv::Point & q = neighbours[n];

                        if (mask.at<cv::Vec3b>(q)[0] == 255)
                        {
                            col += res_prev.at<cv::Vec3b>(q);
                            //
                            col += src.at<cv::Vec3b>(p);
                            col -= src.at<cv::Vec3b>(q);
                        }
                        else
                        {
                            col += base.at<cv::Vec3b>(q);
                        }
                    }
                    col /= float(4 + 1); // 4 neighbours and 1 from res_prev
                    res.at<cv::Vec3b>(p) = col;
                }
            }
#ifdef USE_PPL
        });
#else
        }
#endif // USE_PPL
    }
}

#endif // HEAD_POSE_ESTIMATOR_DEBUG