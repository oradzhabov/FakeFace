#ifndef __IHEAD_ESTIMATOR_H__
#define __IHEAD_ESTIMATOR_H__


#include <opencv2/core/core.hpp>
#include <dlib/image_processing/frontal_face_detector.h>

typedef cv::Matx44d head_pose;

class CFakeFace;
class IHeadEstimator
{
public:
#ifdef HEAD_POSE_ESTIMATOR_DEBUG
    struct sTriangle { size_t  vInd[3]; };
	typedef std::vector<IHeadEstimator::sTriangle>	TriMesh;

    mutable cv::Mat _debug;

#endif // HEAD_POSE_ESTIMATOR_DEBUG

    IHeadEstimator();

	virtual ~IHeadEstimator() {};

	virtual void Initialize(const float & focalLength, const float & opticalCenterX, const float & opticalCenterY) = 0;

	virtual void update(const cv::InputArray image, const bool isFirstFrame) = 0;

	virtual void drawMesh(size_t face_idx) const = 0;

	virtual const size_t    getShapesNb() const = 0;

	virtual const dlib::full_object_detection & getShape(const size_t & index) const = 0;

    /**
    * @brief Return triangles from fake face image
    * @detail If mesh had not been loaded before or CFakeFace object had not recognize face, returns empty array
    */
    std::vector<std::vector<cv::Point2f>> getTriangles(CFakeFace * pFakeFace) const;
    /**
    * @brief Return triangles from current found face image
    * @detail If mesh had not been loaded before or estimator had not recognize requsted shape, returns empty array
    */
    std::vector<std::vector<cv::Point2f>> getTriangles(const int shapeIndex, cv::Rect & estMeshRect) const;
    std::vector<std::vector<cv::Point2f>> getTriangles(const cv::Mat container) const;
protected:

#ifdef HEAD_POSE_ESTIMATOR_DEBUG
    IHeadEstimator::TriMesh  m_FaceMesh; // formed by landmarkes
private:
	const int readFaceMesh(const char * pFileName);
#endif // HEAD_POSE_ESTIMATOR_DEBUG
};

cv::Point2f toCv(const dlib::point& p);

#endif // __IHEAD_ESTIMATOR_H__