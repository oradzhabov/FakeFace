#include "FaceSwitcher.h"
#include "dlib/HeadPoseEstimator.h"
#include "OpenFace/OFEstimator.h"
#include <process.h>

unsigned  int __stdcall CFaceSwitcher::estimatorFunc(void*)
{
    cv::Mat                                     estInputFrame(480, 640, CV_8UC3);
    cv::Mat                                     estInputFrame32f(480, 640, CV_32FC3);
    IHeadEstimator::TriMesh						estFaceMesh;
    cv::Mat                                     imgMorph = cv::Mat::zeros(480, 640, CV_32FC3);
    cv::Mat                                     imgMorph8uc3 = cv::Mat::zeros(480, 640, CV_8UC3);
    cv::Mat                                     mask(480, 640, CV_8UC3);
    //

    m_isFirstEstimatorFrame = true;

    while (m_isEstimatorThreadShouldBeClose == false)
    {
        // Optimal strategy: continue processing right after previous result has been provided to consumer.
        // To do it, event set to signal state only from \getFromEstimator() and estimator have new results.
        // Tests demonstrates that such strategy have 2x free CPU than we set signal's state in \putToEstimator().
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
            m_pEstimator->GetTriangles(estFaceMesh);

        m_pEstimator->update(estInputFrame, m_isFirstEstimatorFrame);

        m_isFirstEstimatorFrame = false;


        // calc poses, not just obtaining results
        // todo: Actually, for obtaing shapes it does not necessary and may take additional time for processing
        //std::vector<head_pose> poses = m_pEstimator->poses();

        // Do not update result if we have no success
        //if (poses.empty() == false)
        if (m_pEstimator->getShapesNb() == 1)
        {
            const dlib::full_object_detection & estShape = m_pEstimator->getShape(0);

            //
            // If we have mesh and fake face
            //
            if (estFaceMesh.empty() == false && m_fakeFace.IsInitialized())
            {
                const dlib::full_object_detection & fakeShape = m_fakeFace.GetLandmarks();
                estInputFrame.convertTo(estInputFrame32f, CV_32FC3);

                // fast way to clear image
                // http://answers.opencv.org/question/88254/most-efficient-way-to-clear-an-image-with-c-interface/
                imgMorph = cv::Mat::zeros(imgMorph.size(), imgMorph.type());
                //imgMorph = estInputFrame32f; // todo: actually heere should be clone()

                const size_t                numTris = estFaceMesh.size();
                std::vector<cv::Point2f>    t1(3);
                std::vector<cv::Point2f>    t2(3);
                std::vector<cv::Point2f>    t2_all;
                for (size_t ti = 0; ti < numTris; ++ti)
                {
                    const IHeadEstimator::sTriangle & tri = estFaceMesh[ti];

                    t1[0] = toCv(fakeShape.part(tri.vInd[0]));
                    t1[1] = toCv(fakeShape.part(tri.vInd[1]));
                    t1[2] = toCv(fakeShape.part(tri.vInd[2]));

                    t2[0] = toCv(estShape.part(tri.vInd[0]));
                    t2[1] = toCv(estShape.part(tri.vInd[1]));
                    t2[2] = toCv(estShape.part(tri.vInd[2]));

                    /*
                    at least some info from original will make result natural in lighting sense
                    0:fakeFace, 1:original
                    BUT: Poisson blending will could make more natural result later.
                    */
                    const double morphFactor = 0.0;

                    morphTriangle(m_fakeFace.GetImg32f(),
                        estInputFrame32f,
                        imgMorph,
                        t1, t2, t2,
                        morphFactor);

                    // store it for future using
                    t2_all.insert(t2_all.end(), t2.begin(), t2.end());
                }

                mask = cv::Mat::zeros(estInputFrame.size(), estInputFrame.type());

                cv::Mat temp8uc3;
                imgMorph.convertTo(temp8uc3, CV_8UC3);
                cv::Rect    estMeshRect = cv::boundingRect(t2_all);
                cv::threshold(temp8uc3, mask, 1, 255, cv::THRESH_BINARY);

                // Fast implementation of Poisson blend (not OpenCV's) negotiate to use lazy data preparation.
                // So it is not necessary to set \isLazyDataPrepared true because it does not effect to speed performance
                //
                // cv::Point   center = (estMeshRect.tl() + estMeshRect.br()) / 2;
                // cv::seamlessClone(temp, estInputFrame, mask, center, imgMorph8uc3, cv::NORMAL_CLONE);
                //
                PoissonBlend(estInputFrame, temp8uc3, mask, imgMorph8uc3, estMeshRect);

                //mask.convertTo (imgMorph8uc3, CV_8UC3);
                //imgMorph.convertTo (imgMorph8uc3, CV_8UC3);
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
                m_pEstimator->calc_pose(0); // to draw lines

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

void CFaceSwitcher::putToEstimator(BYTE * pData) {
    EnterCriticalSection(&m_crit_input);
    cv::Mat  cvFrameFlipped(480, 640, CV_8UC3, pData);
    cv::flip(cvFrameFlipped, m_estInputFrame, 0); // flip requires different holders
    LeaveCriticalSection(&m_crit_input);
}

BYTE * CFaceSwitcher::getFromEstimator(void) {
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

