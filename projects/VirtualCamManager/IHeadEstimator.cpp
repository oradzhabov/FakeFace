#include "IHeadEstimator.h"
#include "FakeFace.h"
#include <opencv2/imgproc.hpp>

#ifdef HEAD_POSE_ESTIMATOR_DEBUG

cv::Point2f toCv(const dlib::point& p)
{
    return cv::Point2f(p.x(), p.y());
}

IHeadEstimator::IHeadEstimator()
{
#ifdef HEAD_POSE_ESTIMATOR_DEBUG
    if (readFaceMesh("conn.txt") == false)
        std::cout << "File conn.txt with face-mesh had not been read" << std::endl;
    else
        std::cout << "File conn.txt with face-mesh has been read successfully" << std::endl;
#endif // HEAD_POSE_ESTIMATOR_DEBUG
}

std::vector<std::vector<cv::Point2f>>
IHeadEstimator::getTriangles(CFakeFace * pFakeFace) const
{
    std::vector<std::vector<cv::Point2f>>   result;
    const dlib::full_object_detection       & fakeShape = pFakeFace->GetLandmarks();
    const size_t                            numTris = m_FaceMesh.size();

    if (fakeShape.num_parts() > 0) {
        result.resize(numTris, std::vector<cv::Point2f>(3));

        for (size_t ti = 0; ti < numTris; ++ti)
        {
            const IHeadEstimator::sTriangle & tri = m_FaceMesh[ti];

            result[ti][0] = toCv(fakeShape.part(tri.vInd[0]));
            result[ti][1] = toCv(fakeShape.part(tri.vInd[1]));
            result[ti][2] = toCv(fakeShape.part(tri.vInd[2]));
        }
    }
    return result;
}

std::vector<std::vector<cv::Point2f>>
IHeadEstimator::getTriangles(const int shapeIndex, cv::Rect & estMeshRect) const
{
    std::vector<std::vector<cv::Point2f>>   result;
    const size_t                            numTris = m_FaceMesh.size();
    const dlib::full_object_detection       & estShape = getShape(shapeIndex);
    std::vector<cv::Point2f>                tri_all;

    if (estShape.num_parts() > 0) {
        result.resize(numTris, std::vector<cv::Point2f>(3));
        for (size_t ti = 0; ti < numTris; ++ti)
        {
            const IHeadEstimator::sTriangle & tri = m_FaceMesh[ti];

            result[ti][0] = toCv(estShape.part(tri.vInd[0]));
            result[ti][1] = toCv(estShape.part(tri.vInd[1]));
            result[ti][2] = toCv(estShape.part(tri.vInd[2]));

            // store it for future using
            tri_all.insert(tri_all.end(), result[ti].begin(), result[ti].end());
        }
    }

    estMeshRect = cv::boundingRect(tri_all);

    return result;
}

std::vector<std::vector<cv::Point2f>>
IHeadEstimator::getTriangles(const cv::Mat container) const
{
    std::vector<std::vector<cv::Point2f>>   result;
    const size_t                            numTris = m_FaceMesh.size();
    const int                               srcWidth = container.cols;
    const int                               srcHeight = container.rows;
    const int                               horizPartsNb = (numTris - numTris % 2) / 2 + numTris % 2;
    const int                               partWidth = (srcWidth - srcWidth % horizPartsNb) / horizPartsNb;

    result.resize(numTris, std::vector<cv::Point2f>(3));

    int ti = 0;
    for (size_t partInd = 0; partInd < horizPartsNb; partInd++)
    {
        result[ti][0] = cv::Point2f(partInd * partWidth, 0);
        result[ti][1] = cv::Point2f(partInd * partWidth + partWidth - 1, 0);
        result[ti][2] = cv::Point2f(partInd * partWidth, srcHeight - 1);
        ti++;

        if (ti < numTris)
        {
            result[ti][0] = cv::Point2f(partInd * partWidth + partWidth - 1, 0);
            result[ti][1] = cv::Point2f(partInd * partWidth + partWidth - 1, srcHeight - 1);
            result[ti][2] = cv::Point2f(partInd * partWidth, srcHeight - 1);
            ti++;
        }
    }

    return result;
}

const int
IHeadEstimator::readFaceMesh(const char * pFileName)
{
    std::ifstream   input(pFileName);
    std::string     line;
    int             result = false;

    m_FaceMesh.clear();

    if (input.is_open())
    {
        result = true;
        while (std::getline(input, line))
        {
            std::istringstream iss(line);
            char delimeter; // ","
            int a, b, c;
            if (!(iss >> a >> delimeter >> b >> delimeter >> c))
            {
                result = false;
                break; // error
            }
            IHeadEstimator::sTriangle   t;
            t.vInd[0] = a-1;
            t.vInd[1] = b-1;
            t.vInd[2] = c-1;
            m_FaceMesh.push_back(t);
        }
    }

    if (result == false)
        m_FaceMesh.clear();

    return result;
}

#endif // HEAD_POSE_ESTIMATOR_DEBUG
