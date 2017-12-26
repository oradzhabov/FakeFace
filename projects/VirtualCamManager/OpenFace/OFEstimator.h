#pragma once

#include "../IHeadEstimator.h"

#include <LandmarkCoreIncludes.h>

#include <vector>


class OFEstimator : public IHeadEstimator
{
	float fx , fy , cx , cy;
	bool cx_undefined;
	bool fx_undefined;
	LandmarkDetector::FaceModelParameters * det_parameters;
	LandmarkDetector::CLNF * clnf_model;
	std::vector<dlib::full_object_detection>	m_landmarks;
public:
	OFEstimator();
	virtual ~OFEstimator();

	virtual void Initialize(const float & focalLength, const float & opticalCenterX, const float & opticalCenterY);

	virtual void update(const cv::InputArray image, const bool isFirstFrame);

	virtual void drawMesh(size_t face_idx) const;

	virtual const size_t    getShapesNb() const;

	virtual const dlib::full_object_detection & getShape(const size_t & index) const;
};

