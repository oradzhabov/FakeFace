#pragma once

#include <Windows.h>
#include <opencv2/core/core.hpp>
#include "FakeFace.h"

class IHeadEstimator;
class CFaceSwitcher {

public:
                        CFaceSwitcher(void);
                        ~CFaceSwitcher(void);

    void				InitEstimator(IHeadEstimator * newEstimator, const char * pFakeFaceFileName);
    void				SetNewFace(const char * pFakeFaceFileName);
    void				putToEstimator(BYTE * pData);
    BYTE *				getFromEstimator(void);

protected:

private:

    static unsigned int __stdcall estimatorFuncThread(void *p_this) {
        CFaceSwitcher* pObj = static_cast<CFaceSwitcher*>(p_this);
        return pObj->estimatorFunc(NULL);
    }
    unsigned  int		estimatorFunc(void*);

private:

    IHeadEstimator	*	m_pEstimator;
    HANDLE              m_estimatorThread;
    bool                m_isFirstEstimatorFrame;
    HANDLE              m_estInputFrameReadyEvent;
    CRITICAL_SECTION    m_crit_input;
    CRITICAL_SECTION    m_crit_estimator; // work with estimator should be wrapped by this crit section
    CRITICAL_SECTION    m_crit_output;
    bool                m_isEstimatorThreadShouldBeClose;
    cv::Mat             m_estInputFrame;
    cv::Mat             m_estOutputFrame;
    cv::Mat             m_estOutputFrameBackbuffer;
    bool                m_IsEstOutputFrameHasBeenUpdated;
    CFakeFace           m_fakeFace;
};