#include "IHeadEstimator.h"

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

void
IHeadEstimator::GetTriangles(IHeadEstimator::TriMesh & result) const
{
    result = m_FaceMesh;
}

#endif // HEAD_POSE_ESTIMATOR_DEBUG
