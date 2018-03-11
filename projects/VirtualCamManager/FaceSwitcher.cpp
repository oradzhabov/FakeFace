#include "FaceSwitcher.h"
#include "dlib/HeadPoseEstimator.h"
#include "OpenFace/OFEstimator.h"
#include <process.h>

unsigned  int __stdcall CFaceSwitcher::estimatorFunc(void*)
{
    cv::Mat                                     estInputFrame(480, 640, CV_8UC3);
    cv::Mat                                     estInputFrame32f(480, 640, CV_32FC3);
    cv::Mat                                     otherFace32fc3 = cv::Mat::zeros(480, 640, CV_32FC3);
    cv::Mat                                     imgMorph8uc3 = cv::Mat::zeros(480, 640, CV_8UC3);
    cv::Mat                                     mask(480, 640, CV_8UC3);
    std::vector<std::vector<cv::Point2f>>       otherFaceTriangles;
    //

    m_isFirstEstimatorFrame = true;

    while (m_isEstimatorThreadShouldBeClose == false)
    {
        // Optimal strategy: continue processing right after previous result has been provided to consumer.
        // To do it, event set to signal state only from \pull() and estimator have new results.
        // Tests demonstrates that such strategy have 2x free CPU than we set signal's state in \push().
        DWORD dwWaitResult = WaitForSingleObject(m_estInputFrameReadyEvent,  // event handle
                                                INFINITE);                  // indefinite wait
        if (dwWaitResult == WAIT_OBJECT_0) // signal
        {
            // Set to nonsignaled. Next loop will lock thread till new source will be prepared
            if (!ResetEvent(m_estInputFrameReadyEvent))
                printf("SetEvent failed (%d)\n", GetLastError());
        }

        EnterCriticalSection(&m_crit_input);
        estInputFrame = m_estInputFrame.clone();
        // todo: actually we could use follow line to avoid clone() but I'm not sure that .data will be good separated
        // in different threads
        // const cv::Mat   estInputFrame(480, 640, CV_8UC3, m_estInputFrame.data);
        LeaveCriticalSection(&m_crit_input);

        // wrap part of code where estimator should be in stabilized state by their crit-section
        EnterCriticalSection(&m_crit_estimator);

        if (m_isFirstEstimatorFrame == true)
            otherFaceTriangles = m_pEstimator->getTriangles(&m_fakeFace);

        m_pEstimator->update(estInputFrame, m_isFirstEstimatorFrame);

        m_isFirstEstimatorFrame = false;


        // Do not update result if we have no success
        if (m_pEstimator->getShapesNb() == 1)
        {
            cv::Rect                                estMeshRect;
            std::vector<std::vector<cv::Point2f>>   faceTriangles = m_pEstimator->getTriangles(0, estMeshRect);
            //
            // If we have mesh and fake face
            //
            if (!otherFaceTriangles.empty() && m_fakeFace.IsInitialized() && faceTriangles.size() == otherFaceTriangles.size())
            {
                estInputFrame.convertTo(estInputFrame32f, CV_32FC3);

                // fast way to clear image
                // http://answers.opencv.org/question/88254/most-efficient-way-to-clear-an-image-with-c-interface/
                otherFace32fc3 = cv::Mat::zeros(otherFace32fc3.size(), otherFace32fc3.type());
                //otherFace32fc3 = estInputFrame32f; // todo: actually here should be clone()

                /*
                at least some info from original will make result natural in lighting sense
                0:fakeFace, 1:original
                BUT: Poisson blending could make more natural result after.
                */
                const double morphFactor = 0.0;
                morphTriangles( m_fakeFace.GetImg32f(), estInputFrame32f, otherFace32fc3,
                                otherFaceTriangles, faceTriangles, faceTriangles,
                                morphFactor);

                mask = cv::Mat::zeros(estInputFrame.size(), estInputFrame.type());

                cv::Mat otherFace8uc3;
                otherFace32fc3.convertTo(otherFace8uc3, CV_8UC3);
                cv::threshold(otherFace8uc3, mask, 1, 255, cv::THRESH_BINARY);

                // Fast implementation of Poisson blend (not OpenCV's) negotiate to use lazy data preparation.
                // So it is not necessary to set \isLazyDataPrepared true because it does not effect to speed performance
                //
                // cv::Point   center = (estMeshRect.tl() + estMeshRect.br()) / 2;
                // cv::seamlessClone(temp, estInputFrame, mask, center, imgMorph8uc3, cv::NORMAL_CLONE);
                //
                PoissonBlend(estInputFrame, otherFace8uc3, mask, imgMorph8uc3, estMeshRect);

                //mask.convertTo (imgMorph8uc3, CV_8UC3);
                //otherFace32fc3.convertTo (imgMorph8uc3, CV_8UC3);
                //
                // Set to true for slow OpenCV's version of Poisson Blending.
                //
                // isLazyDataPrepared      = true;

                EnterCriticalSection(&m_crit_output);
                cv::flip(imgMorph8uc3, m_estOutputFrame, 0); // flip requires different holders
                m_IsEstOutputFrameHasBeenUpdated = true;
                LeaveCriticalSection(&m_crit_output);

            }
            else // If we have not mesh OR fake face
            {
                m_pEstimator->drawMesh(0); // to draw lines

                EnterCriticalSection(&m_crit_output);
                cv::flip(m_pEstimator->_debug, m_estOutputFrame, 0); // flip requires different holders
                m_IsEstOutputFrameHasBeenUpdated = true;
                LeaveCriticalSection(&m_crit_output);
            }
        }
        else
        {
            // To prevent infinite waiting of signaled event, we should solve it here
            EnterCriticalSection(&m_crit_output);
            if (!SetEvent(m_estInputFrameReadyEvent))
                printf("SetEvent failed (%d)\n", GetLastError());
            LeaveCriticalSection(&m_crit_output);
        }

        // un-wrap part of code where estimator should be in stabilized state by their crit-section
        LeaveCriticalSection(&m_crit_estimator);
    }

    return 0;
}

CFaceSwitcher::CFaceSwitcher(void) :
    m_pEstimator(NULL),
    m_estimatorThread(0),
    m_isEstimatorThreadShouldBeClose(false),
    m_estInputFrame(480, 640, CV_8UC3),
    m_estOutputFrame(480, 640, CV_8UC3),
    m_estOutputFrameBackbuffer(480, 640, CV_8UC3),
    m_IsEstOutputFrameHasBeenUpdated(false) {

    InitializeCriticalSection(&m_crit_input);
    InitializeCriticalSection(&m_crit_estimator);
    InitializeCriticalSection(&m_crit_output);

    // Crit sections should be initialized before using \InitEstimator()
    InitEstimator(new OFEstimator(), "lomachenko.jpg");
    /*
    Create a manual-reset event object. The estInputFrame preparation thread sets this
    object to the signaled state when it finishes preparation.
    To start processing, first set should set event to signal state
    */
    m_estInputFrameReadyEvent = CreateEvent(NULL,               // default security attributes
                                            TRUE,               // manual-reset event
                                            TRUE,               // initial state is nonsignaled(FALSE), signaled (TRUE)
                                            TEXT("EstInputFrameReadyEvent")  // object name
                                            );
    /*
    A thread created by _beginthread() will close the handle to the thread when the thread exits, while the handle returned
    by _beginthreadex() will have to be explicitly closed by calling CloseHandle(), which is a similar to the detached thread
    in POSIX.

    thread func created by _beginthreadex() returns an unsigned int and uses the __stdcall calling convention.

    For more onfo see http://www.bogotobogo.com/cplusplus/multithreading_win32A.php
    */
    m_estimatorThread = (HANDLE)_beginthreadex(0, 0, &CFaceSwitcher::estimatorFuncThread, this, 0, 0);
    WaitForSingleObject(CFaceSwitcher::estimatorFuncThread, INFINITE); // start thread
}

CFaceSwitcher::~CFaceSwitcher(void) {
    m_isEstimatorThreadShouldBeClose = true;
    if (!SetEvent(m_estInputFrameReadyEvent))  // Set m_estInputFrameReadyEvent to signaled
        printf("SetEvent failed (%d)\n", GetLastError());
    CloseHandle(m_estInputFrameReadyEvent);
    CloseHandle(m_estimatorThread);
    CloseHandle(&m_crit_input);
    CloseHandle(&m_crit_estimator);
    CloseHandle(&m_crit_output);
    if (m_pEstimator)
        delete m_pEstimator;
}

void CFaceSwitcher::SetNewFace(const char * pFakeFaceFileName) {
    // wrap part of code where estimator should be in stabilized state by their crit-section
    EnterCriticalSection(&m_crit_estimator);

    // Some about how to know the focal length
    // http://photo.stackexchange.com/questions/61724/how-can-i-know-the-focal-length-and-sensor-size-of-my-webcam
    // http://opencv-users.1802565.n2.nabble.com/How-t-o-measure-focal-length-in-logitech-webcam-td5316245.html
    //
    m_fakeFace.Initialize(m_pEstimator, pFakeFaceFileName);

    // For example, say the webcam has a horizontal FOV of about 60 degrees and the image size is 640x480.
    // Then the 4 parameters are:
    // cx = 640/2 = 320
    // cy = 480/2 = 240
    // fx = fy = cx/tan(60/2 * pi / 180) = 554.26
    m_pEstimator->Initialize(554.26f, 640 / 2, 480 / 2);
    // Do not forget reinit main stream
    m_isFirstEstimatorFrame = true;

    // un-wrap part of code where estimator should be in stabilized state by their crit-section
    LeaveCriticalSection(&m_crit_estimator);
}

void CFaceSwitcher::InitEstimator(IHeadEstimator * newEstimator, const char * pFakeFaceFileName) {
    // wrap part of code where estimator should be in stabilized state by their crit-section
    EnterCriticalSection(&m_crit_estimator);

    if (m_pEstimator != NULL)
        delete m_pEstimator;

    m_pEstimator = newEstimator;

    // un-wrap part of code where estimator should be in stabilized state by their crit-section
    LeaveCriticalSection(&m_crit_estimator);

    SetNewFace(pFakeFaceFileName);
}

void CFaceSwitcher::push(BYTE * pData) {
    EnterCriticalSection(&m_crit_input);
    cv::Mat  cvFrameFlipped(480, 640, CV_8UC3, pData);
    cv::flip(cvFrameFlipped, m_estInputFrame, 0); // flip requires different holders
    LeaveCriticalSection(&m_crit_input);
}

BYTE * CFaceSwitcher::pull(void) {
    BYTE *  result = NULL;
    EnterCriticalSection(&m_crit_output);

    // To avoid slow deep copy operation: m_estOutputFrameBackbuffer = m_estOutputFrame.clone()
    // use backbuffer technique
    if (m_IsEstOutputFrameHasBeenUpdated == true)
    {
        std::swap(m_estOutputFrame.data, m_estOutputFrameBackbuffer.data);
        m_IsEstOutputFrameHasBeenUpdated = false;

        if (!SetEvent(m_estInputFrameReadyEvent))
            printf("SetEvent failed (%d)\n", GetLastError());
    }
    result = m_estOutputFrameBackbuffer.data;
    LeaveCriticalSection(&m_crit_output);
    return result;
}

