#include "main.h"
#include "SampleGrabber0.h"
#include <opencv2/core/core.hpp>
#include <opencv2/photo/photo.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <reftime.h>

// landmark indices
// http://openface-api.readthedocs.io/en/latest/_images/dlib-landmark-mean.png
//
// Speeding up dlib landmark detector
// http://126kr.com/article/5q783ux8uhe
//
// Ext Kalman Filter
// https://sites.google.com/site/timecontroll/tutorials/extended-kalman-filtering-with-opencv
//
// COOL blog about CV and head pose estimation
// http://www.learnopencv.com/head-pose-estimation-using-opencv-and-dlib/
//
// FaceTracker with wrappers
// https://github.com/kylemcdonald/ofxFaceTracker

//
// Global data
//
#ifdef HEAD_POSE_ESTIMATOR_DEBUG
    #include <Windows.h>
    #include <process.h>
    #include "FakeFace.h"
    #include "IHeadEstimator.h"
    #include "dlib/HeadPoseEstimator.h"
    #include "OpenFace/OFEstimator.h"

	IHeadEstimator	*	g_estimator = NULL;
    HANDLE              g_estimatorThread = 0;
    bool                g_isFirstEstimatorFrame;
    HANDLE              g_estInputFrameReadyEvent;
    CRITICAL_SECTION    g_crit_input;
    CRITICAL_SECTION    g_crit_estimator; // work with estimator should be wrapped by this crit section
    CRITICAL_SECTION    g_crit_output;
    bool                g_isEstimatorThreadShouldBeClose = false;
    cv::Mat             g_estInputFrame(480, 640, CV_8UC3);
    cv::Mat             g_estOutputFrame(480, 640, CV_8UC3);
    cv::Mat             g_estOutputFrameBackbuffer(480, 640, CV_8UC3);
    bool                g_IsEstOutputFrameHasBeenUpdated = false;
    CFakeFace           g_fakeFace;
    //
    void    putToEstimator(BYTE * pData)
    {
        EnterCriticalSection(&g_crit_input);
            cv::Mat  cvFrameFlipped(480, 640, CV_8UC3, pData);
            cv::flip(cvFrameFlipped, g_estInputFrame, 0); // flip requires different holders

            // To start processing, first set should set event to signal state
            static bool a = false;
            if (a == false)
            {
                if (! SetEvent(g_estInputFrameReadyEvent) ) 
                    printf("SetEvent failed (%d)\n", GetLastError());
            }
            a = true;
            
        LeaveCriticalSection(&g_crit_input);
    }
    BYTE *  getFromEstimator(void)
    {
        BYTE *  result = NULL;
        EnterCriticalSection(&g_crit_output);

            // To avoid slow deep copy operation: g_estOutputFrameBackbuffer = g_estOutputFrame.clone()
            // use backbuffer technique
            if (g_IsEstOutputFrameHasBeenUpdated == true)
            {
                std::swap (g_estOutputFrame.data, g_estOutputFrameBackbuffer.data);
                g_IsEstOutputFrameHasBeenUpdated = false;

                if (! SetEvent(g_estInputFrameReadyEvent) ) 
                    printf("SetEvent failed (%d)\n", GetLastError());
            }
            result = g_estOutputFrameBackbuffer.data;
        LeaveCriticalSection(&g_crit_output);
        return result;
    }
    unsigned  int __stdcall estimatorFunc(void*)
    {
        cv::Mat                                     estInputFrame(480, 640, CV_8UC3);
        cv::Mat                                     estInputFrame32f(480, 640, CV_32FC3);
        IHeadEstimator::TriMesh						estFaceMesh;
        cv::Mat                                     imgMorph = cv::Mat::zeros(480, 640, CV_32FC3);
        cv::Mat                                     imgMorph8uc3 = cv::Mat::zeros(480, 640, CV_8UC3);
        cv::Mat                                     mask(480, 640, CV_8UC3);
        //
        
        g_isFirstEstimatorFrame = true;

        while (g_isEstimatorThreadShouldBeClose == false)
        {
            // Optimal strategy: continue processing right after previous result has been provided to consumer.
            // To do it, event set to signal only from \getFromEstimator() and estimator have preared new results.
            // Tests demonstrates that such strategy have 2x free CPU than we set signal's state in \putToEstimator().
            DWORD dwWaitResult = WaitForSingleObject(   g_estInputFrameReadyEvent,  // event handle
                                                        INFINITE);                  // indefinite wait
            if (dwWaitResult == WAIT_OBJECT_0) // signal
            {
                // Set to nonsignaled. Next loop will lock thread till new source will be prepared
                if (! ResetEvent(g_estInputFrameReadyEvent) )
                    printf("SetEvent failed (%d)\n", GetLastError());
            }

            EnterCriticalSection(&g_crit_input);
                estInputFrame = g_estInputFrame.clone();
                // todo: actually we could use follow line to avoid clone() but I'm not sure that .data will be good separated
                // in different threads
                // const cv::Mat   estInputFrame(480, 640, CV_8UC3, g_estInputFrame.data);
            LeaveCriticalSection(&g_crit_input);

            // wrap part of code where estimator should be in stabilized state by their crit-section
            EnterCriticalSection(&g_crit_estimator);

            if (g_isFirstEstimatorFrame == true)
                g_estimator->GetTriangles(estFaceMesh);

            g_estimator->update(estInputFrame, g_isFirstEstimatorFrame);

            g_isFirstEstimatorFrame = false;


            // calc poses, not just obtaining results
            // todo: Actually, for obtaing shapes it does not necessary and may take additional time for processing
            //std::vector<head_pose> poses = g_estimator->poses();

            // Do not update result if we have no success
            //if (poses.empty() == false)
            if (g_estimator->getShapesNb() == 1)
            {
                const dlib::full_object_detection & estShape = g_estimator->getShape(0);

                //
                // If we have mesh and fake face
                //
                if (estFaceMesh.empty() == false && g_fakeFace.IsInitialized())
                {
                    const dlib::full_object_detection & fakeShape = g_fakeFace.GetLandmars();
                    estInputFrame.convertTo(estInputFrame32f, CV_32FC3);

                    // fast way to clear image
                    // http://answers.opencv.org/question/88254/most-efficient-way-to-clear-an-image-with-c-interface/
                    imgMorph = cv::Mat::zeros( imgMorph.size(), imgMorph.type() );
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

                        morphTriangle ( g_fakeFace.GetImg32f(),
                                        estInputFrame32f,
                                        imgMorph,
                                        t1, t2, t2,
                                        morphFactor);

                        // store it for future using
                        t2_all.insert (t2_all.end(), t2.begin(), t2.end());
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

                    EnterCriticalSection(&g_crit_output);
                        cv::flip(imgMorph8uc3, g_estOutputFrame, 0); // flip requires different holders
                        g_IsEstOutputFrameHasBeenUpdated = true;
                    LeaveCriticalSection(&g_crit_output);
                    
                }
                else // If we have not mesh OR fake face
                {
                    g_estimator->calc_pose(0); // to draw lines

                    EnterCriticalSection(&g_crit_output);
                        cv::flip(g_estimator->_debug, g_estOutputFrame, 0); // flip requires different holders
                        g_IsEstOutputFrameHasBeenUpdated = true;
                    LeaveCriticalSection(&g_crit_output);
                }
            }
            else
            {
                // To prevent infinite waiting of signaled event, we should solve it
                EnterCriticalSection(&g_crit_output);
                    if (! SetEvent(g_estInputFrameReadyEvent) ) 
                        printf("SetEvent failed (%d)\n", GetLastError());
                LeaveCriticalSection(&g_crit_output);
            }

            // un-wrap part of code where estimator should be in stabilized state by their crit-section
            LeaveCriticalSection(&g_crit_estimator);
        }

        return 0;
    }
#endif // HEAD_POSE_ESTIMATOR_DEBUG

HWND ghApp=0;
#ifdef REGISTER_FILTERGRAPH
    DWORD g_dwGraphRegister=0;
#endif // REGISTER_FILTERGRAPH
IVideoWindow  * g_pVW = NULL;
IMediaControl * g_pMC = NULL;
IMediaEventEx * g_pME = NULL;
IGraphBuilder * g_pGraph = NULL;
ICaptureGraphBuilder2 * g_pCapture = NULL;
PLAYSTATE g_psCurrent = Stopped;

HANDLE hMapFile = NULL;

std::vector<std::wstring> device_names;


class CFakeCallback : public ISampleGrabberCB 
{
	ULONG	m_ref;
public:
    //some variables and stuff... 
    STDMETHODIMP_(ULONG) AddRef()  { return m_ref++; }
    STDMETHODIMP_(ULONG) Release() { return m_ref--; }

    STDMETHODIMP QueryInterface(REFIID riid, void ** ppv)
    {
        //CheckPointer(ppv, E_POINTER);

        if (riid == IID_ISampleGrabberCB || riid == IID_IUnknown) 
        {
            *ppv = (void *) static_cast<ISampleGrabberCB *>(this);
            return NOERROR;
        }    
        return E_NOINTERFACE;
    }

	STDMETHODIMP SampleCB( double SampleTime, IMediaSample * pms )
    {
        const REFERENCE_TIME frame_timestamp = SampleTime >= 0 ? SampleTime * UNITS : 0;

		BYTE *pData;
		long lDataLen;
		pms->GetPointer(&pData);
		lDataLen = pms->GetSize();
	
		/*
		for(int i = 0; i < lDataLen; ++i)
			pData[i] = rand();
		*/
	/*
		for(int i = 0; i < lDataLen - 3; i+=3)
		{
			pData[i] = 0; // B
			pData[i+1] = 255; // G
			pData[i+2] = 0; // R
		}
	*/
		/*
	HBITMAP hBmp = (HBITMAP) LoadImage( NULL, L"E:/Install/graphstudio/sample_3.bmp", IMAGE_BITMAP, 0, 0, LR_LOADFROMFILE | LR_CREATEDIBSECTION);
	HRESULT hr = g_pLiveSource->AddFrame(hBmp);
	::DeleteObject(hBmp);
		*/
	    if (hMapFile != NULL)
	    {
            const int sizeOfLong = sizeof(long);
            const int sizeOfRefTime = sizeof(REFERENCE_TIME);
		    unsigned char * pBuf = (unsigned char*) MapViewOfFile(hMapFile,   // handle to map object
																    FILE_MAP_ALL_ACCESS, // read/write permission
																    0,
																    0,
																    sizeOfLong + sizeOfRefTime + lDataLen);

	       if (pBuf != NULL)
	       {
		       // CopyMemory((PVOID)pBuf, szMsg, (_tcslen(szMsg) * sizeof(TCHAR)));
		       memcpy ((unsigned char*)pBuf, & lDataLen, sizeOfLong);

               memcpy ((unsigned char*)pBuf + sizeOfLong, & frame_timestamp, sizeOfRefTime);

		   
#ifdef HEAD_POSE_ESTIMATOR_DEBUG
               //
               // Async set/get
               //
               putToEstimator (pData);

               pData = getFromEstimator();

               memcpy ((unsigned char*)pBuf + sizeOfLong + sizeOfRefTime, pData, lDataLen);
#else
               memcpy ((unsigned char*)pBuf + sizeOfLong + sizeOfRefTime, pData, lDataLen);
#endif

		       UnmapViewOfFile(pBuf);
	       }
	    }

		return NOERROR;
    }

    STDMETHODIMP BufferCB( double SampleTime, BYTE * pBuffer, long BufferLen )
    {

        return S_OK;
    }

};
CFakeCallback fakeGrabber;

void
InitEstimator(IHeadEstimator * newEstimator, const char * pFakeFaceFileName)
{
    // wrap part of code where estimator should be in stabilized state by their crit-section
    EnterCriticalSection(&g_crit_estimator);

    if (g_estimator != NULL)
        delete g_estimator;
    
    g_estimator = newEstimator;

    // Some about how to know the focal length
    // http://photo.stackexchange.com/questions/61724/how-can-i-know-the-focal-length-and-sensor-size-of-my-webcam
    // http://opencv-users.1802565.n2.nabble.com/How-t-o-measure-focal-length-in-logitech-webcam-td5316245.html
	//
    g_fakeFace.Initialize(g_estimator, pFakeFaceFileName);

	// For example, say the webcam has a horizontal FOV of about 60 degrees and the image size is 640x480.
	// Then the 4 parameters are:
	// cx = 640/2 = 320
	// cy = 480/2 = 240
	// fx = fy = cx/tan(60/2 * pi / 180) = 554.26
	g_estimator->Initialize(554.26f, 640/2, 480/2);
    // Do not forget reinit main stream
    g_isFirstEstimatorFrame = true;

    // un-wrap part of code where estimator should be in stabilized state by their crit-section
    LeaveCriticalSection(&g_crit_estimator);
}

HRESULT CaptureVideo()
{
#ifdef HEAD_POSE_ESTIMATOR_DEBUG
    //
    InitializeCriticalSection(&g_crit_input);
    InitializeCriticalSection(&g_crit_estimator);
    InitializeCriticalSection(&g_crit_output);
    g_isEstimatorThreadShouldBeClose = false;

    // Crit sections should be initialized before using \InitEstimator()
    InitEstimator(new OFEstimator(), "lomachenko.jpg");
    /*
        Create a manual-reset event object. The estInputFrame preparation thread sets this
        object to the signaled state when it finishes preparation
    */
    g_estInputFrameReadyEvent = CreateEvent(NULL,               // default security attributes
                                            TRUE,               // manual-reset event
                                            FALSE,              // initial state is nonsignaled
                                            TEXT("EstInputFrameReadyEvent")  // object name
                                            );
    /*
        A thread created by _beginthread() will close the handle to the thread when the thread exits, while the handle returned 
        by _beginthreadex() will have to be explicitly closed by calling CloseHandle(), which is a similar to the detached thread 
        in POSIX.

        thread func created by _beginthreadex() returns an unsigned int and uses the __stdcall calling convention.

        For more onfo see http://www.bogotobogo.com/cplusplus/multithreading_win32A.php
    */
    g_estimatorThread = (HANDLE)_beginthreadex(0, 0, &estimatorFunc, (void*)0, 0, 0);
    WaitForSingleObject(estimatorFunc, INFINITE); // start thread
#endif    

    HRESULT hr;
    IBaseFilter *pSrcFilter=NULL;

    // Get DirectShow interfaces
    hr = GetInterfaces();
    if (FAILED(hr))
    {
        Msg(TEXT("Failed to get video interfaces!  hr=0x%x"), hr);
        return hr;
    }

    // Attach the filter graph to the capture graph
    hr = g_pCapture->SetFiltergraph(g_pGraph);
    if (FAILED(hr))
    {
        Msg(TEXT("Failed to set capture filter graph!  hr=0x%x"), hr);
        return hr;
    }


	hr = CreateVideoGraph(&pSrcFilter);
    if (FAILED(hr))
    {
        // Don't display a message because CreateVideoGraph will handle it
        return hr;
    }

    // Render the preview pin on the video capture filter
    // Use this instead of g_pGraph->RenderFile
	/*
		A pointer to a GUID that specifies one of the pin categories listed in Pin Property Set. 
		To match any pin, regardless of category, set this parameter to NULL. Typical values include the following.

        PIN_CATEGORY_CAPTURE
        PIN_CATEGORY_PREVIEW
        PIN_CATEGORY_CC
	*/
//	const GUID * pCategory = & PIN_CATEGORY_PREVIEW;
//	const GUID * pCategory = & PIN_CATEGORY_CAPTURE;
//	const GUID * pCategory = & PIN_CATEGORY_VIDEOPORT;
//	const GUID * pCategory = & PIN_CATEGORY_CC;
//	const GUID * pCategory = & PIN_CATEGORY_VBI;
//  const GUID * pCategory = & PIN_CATEGORY_STILL;
	const GUID * pCategory = NULL; // To match any pin, regardless of category, set this parameter to NULL

	IBaseFilter * renderer = NULL;
	hr = CreateFilter(L"Null Renderer", & renderer, CLSID_LegacyAmFilterCategory);
	hr = g_pGraph->AddFilter(renderer, L"renderer");

//    hr = g_pCapture->RenderStream (&PIN_CATEGORY_PREVIEW, &MEDIATYPE_Video, // ros:
	hr = g_pCapture->RenderStream (pCategory, &MEDIATYPE_Video,
                                   pSrcFilter, NULL, renderer);
    if (FAILED(hr))
    {
        Msg(TEXT("Couldn't render the video capture stream.  hr=0x%x\r\n")
            TEXT("The capture device may already be in use by another application.\r\n\r\n")
            TEXT("The sample will now close."), hr);
        pSrcFilter->Release();
        return hr;
    }

    // Now that the filter has been added to the graph and we have
    // rendered its stream, we can release this reference to the filter.
    pSrcFilter->Release();

    // Set video window style and position
    hr = SetupVideoWindow();
    if (FAILED(hr))
    {
        // If we stub the output to null-renderer, there is no error when window cannot be setup for output
        if (renderer == NULL)
        {
            Msg(TEXT("Couldn't initialize video window!  hr=0x%x"), hr);
            return hr;
        }
    }

#ifdef REGISTER_FILTERGRAPH
    // Add our graph to the running object table, which will allow
    // the GraphEdit application to "spy" on our graph
    hr = AddGraphToRot(g_pGraph, &g_dwGraphRegister);
    if (FAILED(hr))
    {
        Msg(TEXT("Failed to register filter graph with ROT!  hr=0x%x"), hr);
        g_dwGraphRegister = 0;
    }
#endif

    // Start previewing video data
    hr = g_pMC->Run();
    if (FAILED(hr))
    {
        Msg(TEXT("Couldn't run the graph!  hr=0x%x"), hr);
        return hr;
    }

    // Remember current state
    g_psCurrent = Running;
        
    return S_OK;
}

// https://www.codeproject.com/Articles/158053/DirectShow-Filters-Development-Part-Live-Source
HRESULT CreateVideoGraph(IBaseFilter ** ppLastFilterinGraph)
{
	HRESULT hr = E_FAIL;
	IBaseFilter * ppSourceFilter = NULL;
//	if (!FAILED(CreateFilter(L"PushSource Desktop Filter", & ppSourceFilter, CLSID_LegacyAmFilterCategory)))
	if (!FAILED(FindCaptureDevice(&ppSourceFilter, 0, device_names)))
	{
		if (!FAILED(g_pGraph->AddFilter(ppSourceFilter, L"source")))
		{
//*ppLastFilterinGraph = ppSourceFilter;
//return S_OK;

			// use AVI decompressor filter if source is camera
			IBaseFilter * aviDecompressor = NULL;
			if (!FAILED(CreateFilter(L"AVI Decompressor", & aviDecompressor, CLSID_LegacyAmFilterCategory)))
			{
				if (!FAILED(g_pGraph->AddFilter(aviDecompressor, L"AVI Decompressor")))
				{
					IPin *pOut = NULL;
					// Find an output pin on the upstream filter.
					if (!FAILED(FindUnconnectedPin(ppSourceFilter, PINDIR_OUTPUT, &pOut)))
					{
						if (!FAILED(ConnectFilters(g_pGraph, // Filter Graph Manager.
							pOut,            // Output pin on the upstream filter.
							aviDecompressor)))    // Downstream filter.
						{
							// let source filter will be avi decompressor
							SAFE_RELEASE(ppSourceFilter);
							ppSourceFilter = aviDecompressor;
//							ppSourceFilter->AddRef();
						}
					}
					SAFE_RELEASE(pOut);
				}
			}

			// use ARGB32->RGB24 by tranform filter
			IBaseFilter * tranformFilter = NULL;
			if (!FAILED(CreateFilter(L"Image Effects (EZRGB24)", & tranformFilter, CLSID_LegacyAmFilterCategory)))
			{
				if (!FAILED(g_pGraph->AddFilter(tranformFilter, L"Image Effects")))
				{
					IPin *pOut = NULL;
					// Find an output pin on the upstream filter.
					if (!FAILED(FindUnconnectedPin(ppSourceFilter, PINDIR_OUTPUT, &pOut)))
					{
						if (!FAILED(ConnectFilters(g_pGraph, // Filter Graph Manager.
							pOut,            // Output pin on the upstream filter.
							tranformFilter)))    // Downstream filter.
						{
							// let source filter will be avi decompressor
							SAFE_RELEASE(ppSourceFilter);
							ppSourceFilter = tranformFilter;
//							ppSourceFilter->AddRef();
						}
					}
					SAFE_RELEASE(pOut);
				}
			}
			

			IBaseFilter * SampleGrabFilter = NULL;
//			if (!FAILED(CreateFilter(L"SampleGrabberFilter", & SampleGrabFilter, CLSID_LegacyAmFilterCategory)))
            if (!FAILED(CreateFilter(L"SampleGrabber", & SampleGrabFilter, CLSID_LegacyAmFilterCategory)))
			{
				if (!FAILED(g_pGraph->AddFilter(SampleGrabFilter, L"SampleGrabberFilter")))
				{
					IPin *pOut = NULL;
					if (!FAILED(FindUnconnectedPin(ppSourceFilter, PINDIR_OUTPUT, &pOut)))
					{
						if (!FAILED(ConnectFilters(g_pGraph, // Filter Graph Manager.
							pOut,            // Output pin on the upstream filter.
							SampleGrabFilter)))    // Downstream filter.
						{
							ISampleGrabber * m_pISampleGrabber = NULL;
							if (!FAILED(SampleGrabFilter->QueryInterface( IID_ISampleGrabber, (void**)&m_pISampleGrabber )))
							{
								m_pISampleGrabber->SetCallback(&fakeGrabber, 0);
								//
								DWORD HIGH_BUF_SIZE = 0;
								DWORD LOW_BUF_SIZE = 0x00ffffff;
								hMapFile = CreateFileMapping(INVALID_HANDLE_VALUE,    // use paging file
																NULL,                    // default security
																PAGE_READWRITE,          // read/write access
																HIGH_BUF_SIZE,			// maximum object size (high-order DWORD)
																LOW_BUF_SIZE,            // maximum object size (low-order DWORD)
																TEXT("Local\\VirtualCamDeviceStream"));                 // name of mapping object
                                DWORD lastError = GetLastError();
                                if (hMapFile == NULL)
                                {
                                    Msg(TEXT("Could not create file mapping object (%d).\n"),
                                        GetLastError());
                                }
								hr = S_OK;
							}
							SAFE_RELEASE(m_pISampleGrabber);


							*ppLastFilterinGraph = SampleGrabFilter;
//							(*ppLastFilterinGraph)->AddRef();

//							SAFE_RELEASE(ppSourceFilter);
//							ppSourceFilter = SampleGrabFilter;
						}
					}
					SAFE_RELEASE(pOut);
				}
			}
		}
	}
	SAFE_RELEASE(ppSourceFilter);
	return hr;
}


// original
HRESULT FindCaptureDevice(IBaseFilter ** ppSrcFilter, const int selectIndex, std::vector<std::wstring> & names)
{
    HRESULT hr = S_OK;
    IBaseFilter * pSrc = NULL;
    IMoniker* pMoniker =NULL;
    ICreateDevEnum *pDevEnum =NULL;
    IEnumMoniker *pClassEnum = NULL;

    if (!ppSrcFilter)
	{
        return E_POINTER;
	}
   
    // Create the system device enumerator
    hr = CoCreateInstance (CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC,
                           IID_ICreateDevEnum, (void **) &pDevEnum);
    if (FAILED(hr))
    {
        Msg(TEXT("Couldn't create system enumerator!  hr=0x%x"), hr);
    }

    // Create an enumerator for the video capture devices

	if (SUCCEEDED(hr))
	{
	    hr = pDevEnum->CreateClassEnumerator (CLSID_VideoInputDeviceCategory, &pClassEnum, 0);
		if (FAILED(hr))
		{
			Msg(TEXT("Couldn't create class enumerator!  hr=0x%x"), hr);
	    }
	}

	if (SUCCEEDED(hr))
	{
		// If there are no enumerators for the requested type, then 
		// CreateClassEnumerator will succeed, but pClassEnum will be NULL.
		if (pClassEnum == NULL)
		{
			MessageBox(ghApp,TEXT("No video capture device was detected.\r\n\r\n")
				TEXT("This sample requires a video capture device, such as a USB WebCam,\r\n")
				TEXT("to be installed and working properly.  The sample will now close."),
				TEXT("No Video Capture Hardware"), MB_OK | MB_ICONINFORMATION);
			hr = E_FAIL;
		}
	}

    // Use the first video capture device on the device list.
    // Note that if the Next() call succeeds but there are no monikers,
    // it will return S_FALSE (which is not a failure).  Therefore, we
    // check that the return code is S_OK instead of using SUCCEEDED() macro.

	if (SUCCEEDED(hr))
	{
		HRESULT lastHR = S_FALSE;
		IMoniker * lastMoniker = NULL;

        /*
		if (true)
		{
			hr = pClassEnum->Next (1, &pMoniker, NULL);
			if (hr == S_FALSE)
			{
				Msg(TEXT("Unable to access video capture device!"));   
				hr = E_FAIL;
			}
		}
		else
		{
			while (hr == S_OK)
			{
				hr = pClassEnum->Next (1, &pMoniker, NULL);
		
				if (hr == S_FALSE && lastHR == S_OK)
				{
				}
				else
				{
					lastHR = hr;
					lastMoniker = pMoniker;
					if (hr == S_FALSE)
					{
						Msg(TEXT("Unable to access video capture device!"));   
						hr = E_FAIL;
					}
				}
			}
			pMoniker = lastMoniker;
			hr = lastHR;
		}
        */
        int counter = 0;
		while (hr == S_OK)
		{
			hr = pClassEnum->Next (1, &pMoniker, NULL);

            if (hr == S_OK)
            {
                IPropertyBag *pPropBag;
                HRESULT hr = pMoniker->BindToStorage(0, 0, IID_PPV_ARGS(&pPropBag));
                if (FAILED(hr))
                {
                    pMoniker->Release();
                    continue;  
                }
                VARIANT var;
                VariantInit(&var);

                hr = pPropBag->Read(L"FriendlyName", &var, 0);
                if (SUCCEEDED(hr))
                {
                    names.push_back(std::wstring(var.bstrVal));
                    VariantClear(&var); 

                    if (counter == selectIndex)
                    {
                        lastMoniker = pMoniker;
                    }

                    counter++;
                }
                pPropBag->Release();
            }
        }
        if (lastMoniker != NULL)
        {
            pMoniker = lastMoniker;
            hr = S_OK;
        }
	}

	if (SUCCEEDED(hr))
    {
        // Bind Moniker to a filter object
        hr = pMoniker->BindToObject(0,0,IID_IBaseFilter, (void**)&pSrc);
        if (FAILED(hr))
        {
            Msg(TEXT("Couldn't bind moniker to filter object!  hr=0x%x"), hr);
        }
    }

    // Copy the found filter pointer to the output parameter.
	if (SUCCEEDED(hr))
	{
	    *ppSrcFilter = pSrc;
		(*ppSrcFilter)->AddRef();
	}

	SAFE_RELEASE(pSrc);
    SAFE_RELEASE(pMoniker);
    SAFE_RELEASE(pDevEnum);
    SAFE_RELEASE(pClassEnum);

    return hr;
}


HRESULT GetInterfaces(void)
{
    HRESULT hr;

    // Create the filter graph
    hr = CoCreateInstance (CLSID_FilterGraph, NULL, CLSCTX_INPROC,
                           IID_IGraphBuilder, (void **) &g_pGraph);
    if (FAILED(hr))
        return hr;

    // Create the capture graph builder
    hr = CoCreateInstance (CLSID_CaptureGraphBuilder2 , NULL, CLSCTX_INPROC,
                           IID_ICaptureGraphBuilder2, (void **) &g_pCapture);
    if (FAILED(hr))
        return hr;
    
    // Obtain interfaces for media control and Video Window
    hr = g_pGraph->QueryInterface(IID_IMediaControl,(LPVOID *) &g_pMC);
    if (FAILED(hr))
        return hr;

    hr = g_pGraph->QueryInterface(IID_IVideoWindow, (LPVOID *) &g_pVW);
    if (FAILED(hr))
        return hr;

    hr = g_pGraph->QueryInterface(IID_IMediaEventEx, (LPVOID *) &g_pME);
    if (FAILED(hr))
        return hr;

    // Set the window handle used to process graph events
    hr = g_pME->SetNotifyWindow((OAHWND)ghApp, WM_GRAPHNOTIFY, 0);

    return hr;
}


void CloseInterfaces(void)
{
    // Stop previewing data
    if (g_pMC)
        g_pMC->StopWhenReady();

    g_psCurrent = Stopped;

    // Stop receiving events
    if (g_pME)
        g_pME->SetNotifyWindow(NULL, WM_GRAPHNOTIFY, 0);

    // Relinquish ownership (IMPORTANT!) of the video window.
    // Failing to call put_Owner can lead to assert failures within
    // the video renderer, as it still assumes that it has a valid
    // parent window.
    if(g_pVW)
    {
        g_pVW->put_Visible(OAFALSE);
        g_pVW->put_Owner(NULL);
    }

#ifdef REGISTER_FILTERGRAPH
    // Remove filter graph from the running object table   
    if (g_dwGraphRegister)
        RemoveGraphFromRot(g_dwGraphRegister);
#endif

    // Release DirectShow interfaces
    SAFE_RELEASE(g_pMC);
    SAFE_RELEASE(g_pME);
    SAFE_RELEASE(g_pVW);
    SAFE_RELEASE(g_pGraph);
    SAFE_RELEASE(g_pCapture);
}


HRESULT SetupVideoWindow(void)
{
    HRESULT hr;

    // Set the video window to be a child of the main window
    hr = g_pVW->put_Owner((OAHWND)ghApp);
    if (FAILED(hr))
        return hr;
    
    // Set video window style
    hr = g_pVW->put_WindowStyle(WS_CHILD | WS_CLIPCHILDREN);
    if (FAILED(hr))
        return hr;

    // Use helper function to position video window in client rect 
    // of main application window
    ResizeVideoWindow();

    // Make the video window visible, now that it is properly positioned
    hr = g_pVW->put_Visible(OATRUE);
    if (FAILED(hr))
        return hr;

    return hr;
}


void ResizeVideoWindow(void)
{
    // Resize the video preview window to match owner window size
    if (g_pVW)
    {
        RECT rc;
        
        // Make the preview video fill our window
        GetClientRect(ghApp, &rc);
        g_pVW->SetWindowPosition(0, 0, rc.right, rc.bottom);
    }
}


HRESULT ChangePreviewState(int nShow)
{
    HRESULT hr=S_OK;
    
    // If the media control interface isn't ready, don't call it
    if (!g_pMC)
        return S_OK;
    
    if (nShow)
    {
        if (g_psCurrent != Running)
        {
            // Start previewing video data
            hr = g_pMC->Run();
            g_psCurrent = Running;
        }
    }
    else
    {
/*
ros: make it always playing
        // Stop previewing video data
        hr = g_pMC->StopWhenReady();
        g_psCurrent = Stopped;
*/
    }

    return hr;
}


#ifdef REGISTER_FILTERGRAPH

HRESULT AddGraphToRot(IUnknown *pUnkGraph, DWORD *pdwRegister) 
{
    IMoniker * pMoniker;
    IRunningObjectTable *pROT;
    WCHAR wsz[128];
    HRESULT hr;

    if (!pUnkGraph || !pdwRegister)
        return E_POINTER;

    if (FAILED(GetRunningObjectTable(0, &pROT)))
        return E_FAIL;

    hr = StringCchPrintfW(wsz, NUMELMS(wsz), L"FilterGraph %08x pid %08x\0", (DWORD_PTR)pUnkGraph, 
              GetCurrentProcessId());

    hr = CreateItemMoniker(L"!", wsz, &pMoniker);
    if (SUCCEEDED(hr)) 
    {
        // Use the ROTFLAGS_REGISTRATIONKEEPSALIVE to ensure a strong reference
        // to the object.  Using this flag will cause the object to remain
        // registered until it is explicitly revoked with the Revoke() method.
        //
        // Not using this flag means that if GraphEdit remotely connects
        // to this graph and then GraphEdit exits, this object registration 
        // will be deleted, causing future attempts by GraphEdit to fail until
        // this application is restarted or until the graph is registered again.
        hr = pROT->Register(ROTFLAGS_REGISTRATIONKEEPSALIVE, pUnkGraph, 
                            pMoniker, pdwRegister);
        pMoniker->Release();
    }

    pROT->Release();
    return hr;
}


// Removes a filter graph from the Running Object Table
void RemoveGraphFromRot(DWORD pdwRegister)
{
    IRunningObjectTable *pROT;

    if (SUCCEEDED(GetRunningObjectTable(0, &pROT))) 
    {
        pROT->Revoke(pdwRegister);
        pROT->Release();
    }
}

#endif


void Msg(TCHAR *szFormat, ...)
{
    TCHAR szBuffer[1024];  // Large buffer for long filenames or URLs
    const size_t NUMCHARS = sizeof(szBuffer) / sizeof(szBuffer[0]);
    const int LASTCHAR = NUMCHARS - 1;

    // Format the input string
    va_list pArgs;
    va_start(pArgs, szFormat);

    // Use a bounded buffer size to prevent buffer overruns.  Limit count to
    // character size minus one to allow for a NULL terminating character.
    (void)StringCchVPrintf(szBuffer, NUMCHARS - 1, szFormat, pArgs);
    va_end(pArgs);

    // Ensure that the formatted string is NULL-terminated
    szBuffer[LASTCHAR] = TEXT('\0');

    MessageBox(NULL, szBuffer, TEXT("Application Message"), MB_OK | MB_ICONERROR);
}


HRESULT HandleGraphEvent(void)
{
    LONG evCode;
	LONG_PTR evParam1, evParam2;
    HRESULT hr=S_OK;

    if (!g_pME)
        return E_POINTER;

    while(SUCCEEDED(g_pME->GetEvent(&evCode, &evParam1, &evParam2, 0)))
    {
        //
        // Free event parameters to prevent memory leaks associated with
        // event parameter data.  While this application is not interested
        // in the received events, applications should always process them.
        //
        hr = g_pME->FreeEventParams(evCode, evParam1, evParam2);
        
        // Insert event processing code here, if desired
    }

    return hr;
}

// http://stackoverflow.com/questions/4473431/win32-how-to-create-a-listbox-control-using-the-createwindowexw-function
void InitializeComponent(HWND hWnd) {
    HINSTANCE hInstance = GetModuleHandle(NULL);
}

LRESULT CALLBACK WndMainProc (HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    switch (message)
    {
        // >>
        case WM_CREATE:
            InitializeComponent(hwnd);
            break;
        case WM_COMMAND:
        {
            WORD    wmId    = LOWORD(wParam);
            WORD    wmEvent = HIWORD(wParam);
            switch(wmId)
            {
                case 101:
                    {
                        switch (wmEvent) 
                        {
                        }
                    }
                    break;
            };
            break;
        }
        // <<
        case WM_KEYDOWN:
            {
                switch (wParam)
                {
                case '1':
                    {
                        // todo: actually each estimator could be cashed and do not reinstanced each time.
                        // as was noticed in tests, the main time for this operation takes by constructor of estimator
                        InitEstimator(new HeadPoseEstimator(), NULL);
                    }
                    break;
                case '2':
                    {
                        // todo: actually each estimator could be cashed and do not reinstanced each time.
                        // as was noticed in tests, the main time for this operation takes by constructor of estimator
                        InitEstimator(new OFEstimator(), NULL);
                    }
                    break;
                case 's': // Switch face
                case 'S':
                    {
                        OPENFILENAME ofn;
                        TCHAR szFileName[MAX_PATH] = L"";

                        ZeroMemory(&ofn, sizeof(ofn));

                        ofn.lStructSize = sizeof(ofn); // SEE NOTE BELOW
                        ofn.hwndOwner = hwnd;
                        ofn.lpstrFilter = L"Image Files (*.jpg)\0*.jpg\0All Files (*.*)\0*.*\0";
                        ofn.lpstrFile = szFileName;
                        ofn.nMaxFile = MAX_PATH;
                        ofn.Flags = OFN_EXPLORER | OFN_FILEMUSTEXIST | OFN_HIDEREADONLY|OFN_ALLOWMULTISELECT;
                        ofn.lpstrDefExt = L"jpg";

                        // to prevent switching the current folder and as a result - loosing placement other files
                        // we need to store placement and restore it after changing
                        TCHAR Buffer[MAX_PATH];
                        DWORD dwRet = GetCurrentDirectory(MAX_PATH, Buffer);

                        if(GetOpenFileName(&ofn))
                        {
                            SetCurrentDirectory(Buffer);

                            // wrap part of code where estimator should be in stabilized state by their crit-section
                            EnterCriticalSection(&g_crit_estimator);
                            //
                            g_fakeFace.Initialize(g_estimator, ws2s(szFileName).c_str() );
                            // Do not forget reinit main stream
                            g_estimator->Initialize(554.26f, 640/2, 480/2);
                            g_isFirstEstimatorFrame = true;
                            //
                            // un-wrap part of code where estimator should be in stabilized state by their crit-section
                            LeaveCriticalSection(&g_crit_estimator);
                        }
                    }
                    break;  //or return 0; if you don't want to pass it further to def proc
                    //If not your key, skip to default:
                };
            }
            break;
        case WM_GRAPHNOTIFY:
            HandleGraphEvent();
            break;

        case WM_SIZE:
            ResizeVideoWindow();
            break;

        case WM_WINDOWPOSCHANGED:
            ChangePreviewState(! (IsIconic(hwnd)));
            break;

        case WM_CLOSE:            
            // Hide the main window while the graph is destroyed
            ShowWindow(ghApp, SW_HIDE);
            CloseInterfaces();  // Stop capturing and release interfaces
            break;

        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;
    }

    // Pass this message to the video window for notification of system changes
    if (g_pVW)
        g_pVW->NotifyOwnerMessage((LONG_PTR) hwnd, message, wParam, lParam);

    return DefWindowProc (hwnd , message, wParam, lParam);
}


int PASCAL WinMain(HINSTANCE hInstance, HINSTANCE hInstP, LPSTR lpCmdLine, int nCmdShow)
{
    MSG msg={0};
    WNDCLASS wc;

    // Initialize COM
    if(FAILED(CoInitializeEx(NULL, COINIT_APARTMENTTHREADED)))
    {
        Msg(TEXT("CoInitialize Failed!\r\n"));   
        exit(1);
    } 

    // Register the window class
    ZeroMemory(&wc, sizeof wc);
    wc.lpfnWndProc   = WndMainProc;
    wc.hInstance     = hInstance;
    wc.lpszClassName = CLASSNAME;
    wc.lpszMenuName  = NULL;
    wc.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
    wc.hCursor       = LoadCursor(NULL, IDC_ARROW);
    wc.hIcon         = NULL;
    if(!RegisterClass(&wc))
    {
        Msg(TEXT("RegisterClass Failed! Error=0x%x\r\n"), GetLastError());
        CoUninitialize();
        exit(1);
    }

    // Create the main window.  The WS_CLIPCHILDREN style is required.
    ghApp = CreateWindow(CLASSNAME, APPLICATIONNAME,
                         WS_OVERLAPPEDWINDOW | WS_CAPTION | WS_CLIPCHILDREN,
                         CW_USEDEFAULT, CW_USEDEFAULT,
                         DEFAULT_VIDEO_WIDTH, DEFAULT_VIDEO_HEIGHT,
                         0, 0, hInstance, 0);

    if(ghApp)
    {
        HRESULT hr;

        // Create DirectShow graph and start capturing video
        hr = CaptureVideo();
        if (FAILED (hr))
        {
            CloseInterfaces();
            DestroyWindow(ghApp);
        }
        else
        {
            // Don't display the main window until the DirectShow
            // preview graph has been created.  Once video data is
            // being received and processed, the window will appear
            // and immediately have useful video data to display.
            // Otherwise, it will be black until video data arrives.
            ShowWindow(ghApp, nCmdShow);
        }       

        // Main message loop
        while(GetMessage(&msg,NULL,0,0))
        {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
    }

#ifdef HEAD_POSE_ESTIMATOR_DEBUG
    g_isEstimatorThreadShouldBeClose = true;
    if (! SetEvent(g_estInputFrameReadyEvent) )  // Set g_estInputFrameReadyEvent to signaled
        printf("SetEvent failed (%d)\n", GetLastError());
    CloseHandle(g_estInputFrameReadyEvent);
    CloseHandle(g_estimatorThread);
    CloseHandle(&g_crit_input);
    CloseHandle(&g_crit_estimator);
    CloseHandle(&g_crit_output);
	if (g_estimator)
		delete g_estimator;
#endif // HEAD_POSE_ESTIMATOR_DEBUG
	CloseHandle(hMapFile);

    // Release COM
    CoUninitialize();

    return (int) msg.wParam;
}



