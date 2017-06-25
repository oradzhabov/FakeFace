// dshow video/audio timing.
// https://www.codeproject.com/Articles/24482/DirectShow-Video-and-Audio-Trimming
// multithread in russin
// http://eax.me/winapi-threads/
// semaphores in russin
// http://www.firststeps.ru/mfc/winapi/r.php?117
// О синхронизации на руск.
// http://directshow.wonderu.com/%D1%81%D1%82%D0%B0%D1%82%D1%8C%D0%B8/%D0%BF%D0%B5%D1%80%D0%B2%D1%8B%D0%B5-%D1%88%D0%B0%D0%B3%D0%B8-%D1%81-directshow/%D0%BE%D1%82%D1%81%D1%82%D1%83%D0%BF%D0%BB%D0%B5%D0%BD%D0%B8%D0%B5-%D0%BE-%D1%81%D0%B8%D0%BD%D1%85%D1%80%D0%BE%D0%BD%D0%B8%D0%B7%D0%B0%D1%86%D0%B8%D0%B8
// SSE & filters
// http://stackoverflow.com/questions/26630968/sse-directshow-filter
// another way to build source filter
// https://www.codeproject.com/Articles/158053/DirectShow-Filters-Development-Part-Live-Source

#pragma warning(disable:4244)
#pragma warning(disable:4711)

#include <streams.h>
#include <stdio.h>
#include <olectl.h>
#include <dvdmedia.h>

#include <strsafe.h>
#include <string>
#include <limits.h>

#include "VirtualCamDevice.h"

#ifdef __GNUC__
# include <vfwmsgs.h>
# define _uuidof(x) IID_ ## x
#endif

/*
    Global Definitions
*/
// #define NATIVE_CODE
#define DECLARE_PTR(type, ptr, expr) type* ptr = (type*)(expr);
/*
    Global Contants
*/
const double    c_fps = 20;
const double    c_nMinFPS = 1;
const double    c_nMaxFPS = 30;
//
const int       c_iFormatsCount = 8;
const int       c_iDefaultWidth = 640;//1024;
const int       c_iDefaultHeight = 480;//756;
const int       c_nGranularityW = 80;//160;
const int       c_nGranularityH = 60;//120;
const int       c_nMinWidth = 80;//320;
const int       c_nMinHeight = 60;//240;
const int       c_nMaxWidth = c_nMinWidth + c_nGranularityW * (c_iFormatsCount - 1);
const int       c_nMaxHeight = c_nMinHeight + c_nGranularityH * (c_iFormatsCount - 1);
const int       c_nDefaultBitCount = 24;
/*
    Function Declarations
*/
void Msg(TCHAR *szFormat, ...);
const unsigned int ALIGN16(const unsigned int & rhs);


//////////////////////////////////////////////////////////////////////////
//  VirtualCamDevice is the source filter which masquerades as a capture device
//////////////////////////////////////////////////////////////////////////
CUnknown * WINAPI VirtualCamDevice::CreateInstance(LPUNKNOWN lpunk, HRESULT *phr)
{
    ASSERT(phr);
    CUnknown *punk = new VirtualCamDevice(lpunk, phr);
    return punk;
}

VirtualCamDevice::VirtualCamDevice(LPUNKNOWN lpunk, HRESULT *phr) : 
    CSource(NAME("Virtual Cam"), lpunk, CLSID_VirtualCamDevice)
{
    ASSERT(phr);
    CAutoLock cAutoLock(&m_cStateLock);

    m_nBitCount = c_nDefaultBitCount;
    m_nAvgTimePerFrame = UNITS / c_fps;
    int m_nWidth = c_iDefaultWidth;
    int m_nHeight = c_iDefaultHeight;

    // Create the one and only output pin
    m_paStreams = (CSourceStream **) new VirtualCamDeviceStream*[1];
    m_paStreams[0] = new VirtualCamDeviceStream(phr, this, L"Virtual Cam");
}

// IAMFilterMiscFlags
ULONG VirtualCamDevice::GetMiscFlags( void)
{
    return AM_FILTER_MISC_FLAGS_IS_SOURCE;
}

HRESULT VirtualCamDevice::QueryInterface(REFIID riid, void **ppv)
{
    //Forward request for IAMStreamConfig & IKsPropertySet to the pin
    if(riid == _uuidof(IAMStreamConfig) ||
        riid == _uuidof(IKsPropertySet) ||
        riid == _uuidof(IAMLatency) ||
        riid == _uuidof(IAMBufferNegotiation) ||
        riid == _uuidof(IAMPushSource) ||
        riid == _uuidof(IAMStreamControl))
        return m_paStreams[0]->QueryInterface(riid, ppv);
    else
        return CSource::QueryInterface(riid, ppv);
}

HRESULT VirtualCamDevice::GetDefaultCaps(int nIndex, VIDEO_STREAM_CONFIG_CAPS * _caps)
{
    _caps->guid = FORMAT_VideoInfo;
    _caps->VideoStandard = AnalogVideo_None;
    _caps->InputSize.cx = c_iDefaultWidth;
    _caps->InputSize.cy = c_iDefaultHeight;
    _caps->MinCroppingSize.cx = c_nMinWidth;
    _caps->MinCroppingSize.cy = c_nMinHeight;

    _caps->MaxCroppingSize.cx = c_nMaxWidth;
    _caps->MaxCroppingSize.cy = c_nMaxHeight;
    _caps->CropGranularityX = c_nGranularityW;
    _caps->CropGranularityY = c_nGranularityH;
    _caps->CropAlignX = 0;
    _caps->CropAlignY = 0;

    _caps->MinOutputSize.cx = _caps->MinCroppingSize.cx;
    _caps->MinOutputSize.cy = _caps->MinCroppingSize.cy;
    _caps->MaxOutputSize.cx = _caps->MaxCroppingSize.cx;
    _caps->MaxOutputSize.cy = _caps->MaxCroppingSize.cy;
    _caps->OutputGranularityX = _caps->CropGranularityX;
    _caps->OutputGranularityY = _caps->CropGranularityY;
    _caps->StretchTapsX = 0;
    _caps->StretchTapsY = 0;
    _caps->ShrinkTapsX = 0;
    _caps->ShrinkTapsY = 0;
    _caps->MinFrameInterval = UNITS / c_nMaxFPS;
    _caps->MaxFrameInterval = UNITS / c_nMinFPS;
    _caps->MinBitsPerSecond = (_caps->MinOutputSize.cx * _caps->MinOutputSize.cy * c_nDefaultBitCount) * c_nMinFPS;
    _caps->MaxBitsPerSecond = (_caps->MaxOutputSize.cx * _caps->MaxOutputSize.cy * c_nDefaultBitCount) * c_nMaxFPS;

    return NOERROR;
}

HRESULT VirtualCamDevice::GetMediaType(int iPosition, CMediaType *pmt)
{
    if(iPosition < 0)
        return E_INVALIDARG;
    if(iPosition > c_iFormatsCount) 
        return VFW_S_NO_MORE_ITEMS;

    VIDEO_STREAM_CONFIG_CAPS _caps;
    GetDefaultCaps(iPosition, & _caps);

    int nWidth = 0;
    int nHeight = 0;

    if (iPosition == 0)
    {
        if (GetPinCount() > 0)
        {
//            VirtualCamDeviceStream * p = reinterpret_cast<VirtualCamDeviceStream *>(GetPin(0));
            VirtualCamDeviceStream * p = (VirtualCamDeviceStream *)(GetPin(0));
            if (p->m_mt.majortype == MEDIATYPE_Video)
            {
                //*pmt = p->m_mt;
                pmt->Set(p->m_mt);
                return NOERROR;
            }
        }
        nWidth = _caps.InputSize.cx;
        nHeight = _caps.InputSize.cy;
    }
    else
    {
        nWidth = _caps.MinOutputSize.cx + _caps.OutputGranularityX * (iPosition - 1);
        nHeight = _caps.MinOutputSize.cy + _caps.OutputGranularityY * (iPosition - 1);
        if (nWidth > _caps.MaxOutputSize.cx || nHeight > _caps.MaxOutputSize.cy)
        {
            return VFW_S_NO_MORE_ITEMS;
        }
    }

    DECLARE_PTR(VIDEOINFOHEADER, pvi, pmt->AllocFormatBuffer(sizeof(VIDEOINFOHEADER)));
    ZeroMemory(pvi, sizeof(VIDEOINFOHEADER));

    pvi->AvgTimePerFrame = m_nAvgTimePerFrame;
    pvi->bmiHeader.biCompression = BI_RGB;
    pvi->bmiHeader.biSize       = sizeof(BITMAPINFOHEADER);
    pvi->bmiHeader.biClrImportant = 0;
    pvi->bmiHeader.biBitCount = (short)m_nBitCount;
    pvi->bmiHeader.biWidth = nWidth;
    pvi->bmiHeader.biHeight = nHeight;
    pvi->bmiHeader.biPlanes = 1;
    pvi->bmiHeader.biSizeImage = GetBitmapSize(&pvi->bmiHeader);

    SetRectEmpty(&(pvi->rcSource)); // we want the whole image area rendered.
    SetRectEmpty(&(pvi->rcTarget)); // no particular destination rectangle

    pmt->SetType(&MEDIATYPE_Video);
    pmt->SetFormatType(&FORMAT_VideoInfo);
    pmt->SetTemporalCompression(FALSE);

    // Work out the GUID for the subtype from the header info.
    const GUID SubTypeGUID = GetBitmapSubtype(&pvi->bmiHeader);
    pmt->SetSubtype(&SubTypeGUID);
    pmt->SetSampleSize(pvi->bmiHeader.biSizeImage);
    
    return NOERROR;
}

HRESULT STDMETHODCALLTYPE  VirtualCamDevice::GetStreamCaps(int iIndex, CMediaType *pmt, BYTE *pSCC)
{
    if (pmt == NULL || pSCC == NULL) return E_INVALIDARG;
    if (iIndex < 0) return E_INVALIDARG;

    HRESULT hr = GetMediaType(iIndex, pmt);
    if (FAILED(hr)) return hr;
    if (hr == VFW_S_NO_MORE_ITEMS) return S_FALSE;

    DECLARE_PTR(VIDEO_STREAM_CONFIG_CAPS, pvscc, pSCC);
    hr = GetDefaultCaps(iIndex, pvscc);

    return hr;
}

HRESULT VirtualCamDevice::GetLatency( REFERENCE_TIME *prtLatency)
{
    *prtLatency = UNITS / 30;

//    VirtualCamDeviceStream * p = reinterpret_cast<VirtualCamDeviceStream *>(GetPin(0));
    VirtualCamDeviceStream * p = (VirtualCamDeviceStream *)(GetPin(0));

    CMediaType mt = p->m_mt;
    if (mt.majortype == MEDIATYPE_Video)
    {
        {
            VIDEOINFOHEADER *pvi = (VIDEOINFOHEADER *)(mt.Format());
            if (pvi)
                *prtLatency = pvi->AvgTimePerFrame;
        }
        {
            /*
            VideoInfoHeader2 _pvi = mt;
            if (_pvi != null)
            {
                prtLatency = _pvi.AvgTimePerFrame;
            }
            */
        }
    }
    return NOERROR;
}

HRESULT VirtualCamDevice::DecideBufferSize(IMemAllocator *pAlloc, ALLOCATOR_PROPERTIES *prop)
{
    if (GetPinCount() > 0)
    {
        ALLOCATOR_PROPERTIES _actual;
    
        VirtualCamDeviceStream * p = (VirtualCamDeviceStream*)(GetPin(0));

        if (p->m_mt.majortype != MEDIATYPE_Video)
            return VFW_E_INVALIDMEDIATYPE;

        BITMAPINFOHEADER * _bmi = &((VIDEOINFOHEADER *)p->m_mt.pbFormat)->bmiHeader;

        prop->cbBuffer = GetBitmapSize(_bmi);

        if (prop->cbBuffer < _bmi->biSizeImage)
            prop->cbBuffer = _bmi->biSizeImage;

        if (prop->cbBuffer < m_bmi.biSizeImage)
            prop->cbBuffer = m_bmi.biSizeImage;

        prop->cBuffers = 1;
        prop->cbAlign = 1;
        prop->cbPrefix = 0;
        return pAlloc->SetProperties(prop, &_actual);
    }
    return VFW_E_INVALIDMEDIATYPE;
}

HRESULT VirtualCamDevice::SetMediaType(const CMediaType *pmt)
{
    CAutoLock cAutoLock(&m_cStateLock);

    BITMAPINFOHEADER * _bmi = &((VIDEOINFOHEADER *)pmt->pbFormat)->bmiHeader;

    m_bmi.biBitCount = _bmi->biBitCount;
    if (_bmi->biHeight != 0) m_bmi.biHeight = _bmi->biHeight;
    if (_bmi->biWidth > 0) m_bmi.biWidth = _bmi->biWidth;
    m_bmi.biCompression = BI_RGB;
    m_bmi.biPlanes = 1;
    m_bmi.biSizeImage = ALIGN16(m_bmi.biWidth) * ALIGN16(abs(m_bmi.biHeight)) * m_bmi.biBitCount / 8;
    m_nWidth = _bmi->biWidth;
    m_nHeight = _bmi->biHeight;
    m_nBitCount = _bmi->biBitCount;

    VIDEOINFOHEADER *pvi = (VIDEOINFOHEADER *)(pmt->Format());
    if (pvi)
        m_nAvgTimePerFrame = pvi->AvgTimePerFrame;
    /*
    {
        VideoInfoHeader2 _pvi = pmt;
        if (_pvi != null)
        {
            m_nAvgTimePerFrame = _pvi.AvgTimePerFrame;
        }
    }
    */
    return NOERROR;
}
HRESULT VirtualCamDevice::CheckMediaType(const CMediaType *pMediaType)
{
    if (pMediaType == NULL)
        return E_POINTER;
    if (pMediaType->majortype != MEDIATYPE_Video)
        return VFW_E_INVALIDMEDIATYPE;
    if (pMediaType->subtype != MEDIASUBTYPE_RGB24)
        return VFW_E_INVALIDMEDIATYPE;

    //const BITMAPINFOHEADER & _bmi = (VIDEOINFOHEADER*)(pMediaType->pbFormat)->bmiHeader;
    const BITMAPINFOHEADER & _bmi = ((VIDEOINFOHEADER *)pMediaType->pbFormat)->bmiHeader;

    if (_bmi.biCompression != BI_RGB)
        return VFW_E_TYPE_NOT_ACCEPTED;
    if (_bmi.biBitCount != c_nDefaultBitCount)
        return VFW_E_TYPE_NOT_ACCEPTED;
    
    VIDEO_STREAM_CONFIG_CAPS _caps;
    GetDefaultCaps(0, & _caps);
    if (_bmi.biWidth < _caps.MinOutputSize.cx || _bmi.biWidth > _caps.MaxOutputSize.cx)
        return VFW_E_INVALIDMEDIATYPE;

    long _rate = 0;
    VIDEOINFOHEADER *pvi = (VIDEOINFOHEADER *)(pMediaType->Format());
    if (pvi)
        _rate = pvi->AvgTimePerFrame;
    /*
    {
        VideoInfoHeader2 _pvi = pmt;
        if (_pvi != null)
        {
            _rate = _pvi.AvgTimePerFrame;
        }
    }
    */
    if (_rate < _caps.MinFrameInterval || _rate > _caps.MaxFrameInterval)
        return VFW_E_INVALIDMEDIATYPE;
    return NOERROR;
}

HRESULT VirtualCamDevice::GetAllocatorProperties(ALLOCATOR_PROPERTIES * pprop)
{
    VirtualCamDeviceStream * p = (VirtualCamDeviceStream*)(GetPin(0));
    const CMediaType & mt = p->m_mt;
    if (mt.majortype == MEDIATYPE_Video)
    {
        ULONG lSize = mt.GetSampleSize();
        BITMAPINFOHEADER * _bmi = &((VIDEOINFOHEADER *)mt.pbFormat)->bmiHeader;

        if (lSize < GetBitmapSize(_bmi))
        {
            lSize = GetBitmapSize(_bmi);
        }
        if (lSize < _bmi->biSizeImage)
        {
            lSize = _bmi->biSizeImage;
        }
        pprop->cbBuffer = lSize;
        pprop->cBuffers = 1;
        pprop->cbAlign = 1;
        pprop->cbPrefix = 0;
    }

    return NOERROR;
}

HRESULT VirtualCamDevice::SuggestAllocatorProperties(const ALLOCATOR_PROPERTIES *pprop)
{
    ALLOCATOR_PROPERTIES _properties;
    HRESULT hr = GetAllocatorProperties(& _properties);
    if (FAILED(hr))
        return hr;

    if (pprop->cbBuffer != -1)
    {
        if (pprop->cbBuffer < _properties.cbBuffer)
            return E_FAIL;
    }
    if (pprop->cbAlign != -1 && pprop->cbAlign != _properties.cbAlign)
        return E_FAIL;
    if (pprop->cbPrefix != -1 && pprop->cbPrefix != _properties.cbPrefix)
        return E_FAIL;
    if (pprop->cBuffers != -1 && pprop->cBuffers < 1)
        return E_FAIL;
    return NOERROR;
}

//////////////////////////////////////////////////////////////////////////
// VirtualCamDeviceStream is the one and only output pin of VirtualCamDevice
// which handles all the stuff.
//////////////////////////////////////////////////////////////////////////
VirtualCamDeviceStream::VirtualCamDeviceStream(HRESULT *phr, VirtualCamDevice *pParent, LPCWSTR pPinName) :
    CSourceStream(NAME("Virtual Cam"),phr, pParent, pPinName), m_pParent(pParent)
{
    hMapFile = NULL;
    m_IsPPropertiesValid  = false;

    m_rtStreamOffset = 0;
    m_rtStreamOffsetMax = -1;

    m_rtStartAt = -1;
    m_rtStopAt = -1;
    m_dwAdviseToken = 0;
    m_rtClockStart = 0;
    m_bShouldFlush = false;
    m_dwStartCookie = 0;
    m_dwStopCookie = 0;



    // Set the default media type as 640x480xBitCount@15
    GetMediaType(8, &m_mt);
}

VirtualCamDeviceStream::~VirtualCamDeviceStream()
{
    if (hMapFile)
	    CloseHandle(hMapFile);
} 

HRESULT VirtualCamDeviceStream::QueryInterface(REFIID riid, void **ppv)
{   
    // Standard OLE stuff
    if(riid == _uuidof(IAMStreamConfig))
        *ppv = (IAMStreamConfig*)this;
    else if(riid == _uuidof(IKsPropertySet))
        *ppv = (IKsPropertySet*)this;
    else if (riid == _uuidof(IAMBufferNegotiation))
        *ppv = (IAMBufferNegotiation*)this;
    else if (riid == _uuidof(IAMPushSource))
        *ppv = (IAMPushSource*)this;
    else if (riid == _uuidof(IAMLatency))
        *ppv = (IAMLatency*)this;
    else if (riid == _uuidof(IAMStreamControl))
        *ppv = (IAMStreamControl*)this;
    else
        return CSourceStream::QueryInterface(riid, ppv);

    AddRef();
    return S_OK;
}


HRESULT VirtualCamDeviceStream::Active()
{
    /*
    m_rtStart = 0;
    m_bStartNotified = false;
    m_bStopNotified = false;
    {
        lock (m_Filter.FilterLock)
        {
            m_pClock = m_Filter.Clock;
		    if (m_pClock.IsValid)
		    {
                m_pClock._AddRef();
			    m_hSemaphore = new Semaphore(0,0x7FFFFFFF);
		    }
        }
    }
    return base.Active();
    */
    //
    m_rtStart = 0;
    m_bStartNotified = false;
    m_bStopNotified = false;
    {
        CAutoLock cAutoLock(m_pFilter->pStateLock());

        m_hSemaphore = NULL;
        m_pRefClock = NULL;
        HRESULT hr = m_pFilter->GetSyncSource(& m_pRefClock);

        if (hr == S_OK)
        {
            /*
                In some cases (like in Skype) filter does not contain Clock and GetSyncSource() returns NULL.
                To be able use optimized fillBuffer() here we construct the Clock and connect it to all parent's filters
            */
            if (m_pRefClock == NULL)
            {
                IReferenceClock *pClock = NULL;
                IEnumFilters *pFiltEnum = NULL;

                if (!FAILED(CoCreateInstance (CLSID_SystemClock, NULL, CLSCTX_INPROC_SERVER,
                                    IID_IReferenceClock, (void **) (&m_pRefClock))) )
                {
                    if (!FAILED(m_pParent->GetFilterGraph()->EnumFilters (&pFiltEnum)))
                    {
                        IBaseFilter *pCurrFilt = NULL;
                        while (pFiltEnum->Next (1, &pCurrFilt, 0) == S_OK)
                        {
                            pCurrFilt->SetSyncSource (pClock);
                            pCurrFilt->Release ();
                        }
                    }
                    else
                    {
                        m_pRefClock->Release();
                        m_pRefClock = NULL;
                    }
                    if (pFiltEnum) pFiltEnum->Release ();
                    if (pClock) pClock->Release ();
                }
            }
        
            if (m_pRefClock)
            {
                // There is no necessary to AddRef() because \GetSyncSource() or \CoCreateInstance (CLSID_SystemClock..) added it automatically.
                // m_pRefClock->AddRef();
                m_hSemaphore = CreateSemaphore(NULL, 0, 0x7FFFFFFF, NULL);
            }
        }
    }
    return CSourceStream::Active();
}

HRESULT VirtualCamDeviceStream::Inactive()
{
    HRESULT hr = CSourceStream::Inactive();
    if (m_pRefClock != NULL)
    {
        if (m_dwAdviseToken != 0)
        {
            m_pRefClock->Unadvise (m_dwAdviseToken);
            m_dwAdviseToken = 0;
        }

        m_pRefClock->Release();
        m_pRefClock = NULL;

        if (m_hSemaphore != NULL)
        {
            CloseHandle(m_hSemaphore);
            m_hSemaphore = NULL;
        }
    }
    return hr;
}

//////////////////////////////////////////////////////////////////////////
//  This is the routine where we create the data being output by the Virtual
//  Camera device.
//////////////////////////////////////////////////////////////////////////
HRESULT VirtualCamDeviceStream::FillBuffer(IMediaSample *pms)
{
    {
        //AMMediaType pmt;
        //CMediaType * pmt;
        /*
            Address of a variable that receives a pointer to an AM_MEDIA_TYPE structure. 
            If the media type has not changed from the previous sample, *ppMediaType is set to NULL.
            NOTE:
            If the method returns S_OK, the caller must free the memory for the media type, including the format block
        */
        
        AM_MEDIA_TYPE * ppMediaType;
        if (S_OK == pms->GetMediaType (& ppMediaType))
        {
            CMediaType pmt(*ppMediaType);
            if (FAILED(SetMediaType( &pmt )) )
            {
                ASSERT(false);
                pms->SetMediaType(NULL);
            }
            DeleteMediaType(ppMediaType);
        }
    }
    if (m_pRefClock == NULL || m_hSemaphore == NULL)
        return FillBuffer_0(pms);

    REFERENCE_TIME _start, _stop;
    HRESULT hr = NOERROR;
    REFERENCE_TIME rtLatency;
    if (FAILED(GetLatency(& rtLatency)))
    {
        rtLatency = UNITS / 30;
    }

    bool bShouldDeliver = false;
    do
    {
        if (m_dwAdviseToken == 0)
        {
            m_pRefClock->GetTime( & m_rtClockStart);
            hr = m_pRefClock->AdvisePeriodic(m_rtClockStart + rtLatency, rtLatency, (HSEMAPHORE)m_hSemaphore, & m_dwAdviseToken);
//            hr.Assert();
        }
        else
        {
            const DWORD waitResult = WaitForSingleObject(m_hSemaphore, INFINITE);
            switch (waitResult)
            {
            case WAIT_FAILED:
                {
                    DWORD errorMessageID = GetLastError();
                    LPWSTR messageBuffer = NULL;
                    size_t size = FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
                                 NULL, errorMessageID, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPWSTR)&messageBuffer, 0, NULL);

                    // std::wstring message(messageBuffer, size);

                    Msg (L"WaitForSingleObject ERROR: %s", messageBuffer);

                    //Free the buffer.
                    LocalFree(messageBuffer);
                    break;
                }
            case WAIT_OBJECT_0: // ok
                {
                    int a = 0;
                    break;
                }
            case WAIT_TIMEOUT: // interrupt via timeout
                {
                    int a = 0;
                    break;
                }
            case WAIT_ABANDONED: // thread locked semaphore are finished without releaseing the semaphore.
                                //Now semaphore locked by current thread
                {
                    int a = 0;
                    break;
                }
            default:
                {
                    int a = 0;
                    break;
                }
            }
        }
        bShouldDeliver = TRUE;
        _start = m_rtStart;
        _stop = m_rtStart + 1;
        pms->SetTime(&_start, &_stop);

        hr = FillBuffer_0(pms);
        if (FAILED(hr) || S_FALSE == hr)
            return hr;

        m_pRefClock->GetTime(& m_rtClockStop);
        pms->GetTime(& _start, & _stop);
                
        if (rtLatency > 0 && m_rtClockStop - m_rtClockStart > rtLatency * 3)
        {
            m_rtClockStop = m_rtClockStart + rtLatency;
        }
        _stop = _start + (m_rtClockStop - m_rtClockStart);
        m_rtStart = _stop;
        
        {
            CAutoLock cAutoLock(&m_csPinLock);

            _start -= m_rtStreamOffset;
            _stop -= m_rtStreamOffset;
        }
        pms->SetTime(&_start, &_stop);
        m_rtClockStart = m_rtClockStop;

        bShouldDeliver = ((_start >= 0) && (_stop >= 0));

        if (bShouldDeliver)
        {
            {
                CAutoLock cAutoLock(&m_csPinLock);

                if (m_rtStartAt != -1)
                {
                    if (m_rtStartAt > _start)
                    {
                        bShouldDeliver = FALSE;
                    }
                    else
                    {
                        if (m_dwStartCookie != 0 && !m_bStartNotified)
                        {
                            m_bStartNotified = TRUE;
                            hr = (HRESULT)m_pFilter->NotifyEvent(EC_STREAM_CONTROL_STARTED, (LONG_PTR)(this), m_dwStartCookie);
                            if (FAILED(hr)) return hr;
                        }
                    }
                }
            }
            if (!bShouldDeliver) continue;
            if (m_rtStopAt != -1)
            {
                if (m_rtStopAt < _start)
                {
                    if (!m_bStopNotified)
                    {
                        m_bStopNotified = TRUE;
                        if (m_dwStopCookie != 0)
                        {
                            // hr = m_pFilter->NotifyEvent(EC_STREAM_CONTROL_STOPPED, Marshal.GetIUnknownForObject(this), (IntPtr)m_dwStopCookie);
                            hr = (HRESULT)m_pFilter->NotifyEvent(EC_STREAM_CONTROL_STOPPED, (LONG_PTR)(this), m_dwStopCookie);
                            if (FAILED(hr)) return hr;
                        }
                        bShouldDeliver = m_bShouldFlush;
                    }
                    else
                    {
                        bShouldDeliver = FALSE;
                    }
                    // EOS
                    if (!bShouldDeliver) return S_FALSE;
                }
            }
        }
    }
    while (!bShouldDeliver);

    return NOERROR;
}
HRESULT VirtualCamDeviceStream::FillBuffer_0(IMediaSample *pms)
{
    REFERENCE_TIME avgFrameTime     = ((VIDEOINFOHEADER*)m_mt.pbFormat)->AvgTimePerFrame;
    REFERENCE_TIME rtNow            = m_rtLastTime;

//    m_rtLastTime += avgFrameTime;
//    pms->SetTime(&rtNow, &m_rtLastTime);
//    pms->SetSyncPoint(TRUE);

    BYTE *pData;
    long lDataLen;
    pms->GetPointer(&pData);
    lDataLen = pms->GetSize();
	
	if (hMapFile != NULL)
	{
		unsigned char * pBuf = (unsigned char *) MapViewOfFile(hMapFile, // handle to map object
															   FILE_MAP_ALL_ACCESS,  // read/write permission
															   0,
															   0,
															   sizeof(long) + sizeof(REFERENCE_TIME));
		if (pBuf != NULL)
		{
			long bufLength = *(long*)(pBuf);
            const REFERENCE_TIME receivedTimeStamp = *(REFERENCE_TIME*)(pBuf + sizeof(long));
			UnmapViewOfFile(pBuf);

			if (bufLength != 0)
			{
                REFERENCE_TIME receivedFrameTime = receivedTimeStamp - m_rtLastReceivedTime;
                /*
                    
                if (receivedFrameTime <= 0 || avgFrameTime < receivedFrameTime)
                    receivedFrameTime = avgFrameTime;

                
                m_rtLastTime = rtNow + receivedFrameTime;
                m_rtLastReceivedTime = receivedTimeStamp;


                pms->SetTime(&rtNow, &m_rtLastTime);
                pms->SetSyncPoint(TRUE);
                */
                m_rtLastTime = rtNow + avgFrameTime;
                m_rtLastReceivedTime = receivedTimeStamp;
                //
                pms->SetTime(&rtNow, &m_rtLastTime);
                pms->SetSyncPoint(TRUE);


				pBuf = (unsigned char *) MapViewOfFile(hMapFile, // handle to map object
														FILE_MAP_ALL_ACCESS,  // read/write permission
														0,
														0,
														bufLength);
				if (pBuf != NULL)
				{
					if (lDataLen < bufLength)
						bufLength = lDataLen;

					memcpy (pData, pBuf, bufLength);

					UnmapViewOfFile(pBuf);
                    /*
                        If server did not change time per frames it means that it stops and we should release the shared memory
                        after some (1 sec) period of time
                    */
                    if (m_rtLastReceivedTime > 0 && receivedFrameTime == 0)
                    {
                        m_delayInReceived += avgFrameTime;
                        if (m_delayInReceived > UNITS * c_nMaxFPS * 1)
                        {
                            CloseHandle(hMapFile);
                            hMapFile = NULL;
                        }
                    }
                    else
                    {
                        m_delayInReceived = 0;
                    }
				}
				else
				{
					Msg(L"Error in %d bytes mapping", bufLength);
				}
			}
			else
			{
//				Msg("Obtained 0-length buffer");
			}
		}
		else
		{
			Msg(L"Error in num bytes mapping");
		}
	}
    else
    {
        /*
            Push random pixels frame
        */
        for(int i = 0; i < lDataLen; ++i)
            pData[i] = rand();
    }

	return NOERROR;
} // FillBuffer


//
// Notify
// Ignore quality management messages sent from the downstream filter
STDMETHODIMP VirtualCamDeviceStream::Notify(IBaseFilter * pSender, Quality q)
{
    return E_NOTIMPL;
} // Notify

//////////////////////////////////////////////////////////////////////////
// This is called when the output format has been negotiated
//////////////////////////////////////////////////////////////////////////
HRESULT VirtualCamDeviceStream::SetMediaType(const CMediaType *pmt)
{
    if (pmt == NULL)
        return E_POINTER;
    if (pmt->FormatType() == NULL) // ros: i've added it
        return VFW_E_INVALIDMEDIATYPE;

    HRESULT hr = CheckMediaType(pmt);
    if (FAILED(hr))
        return hr;

    DECLARE_PTR(VIDEOINFOHEADER, pvi, pmt->Format());
    if (pvi == NULL) // ros: i've added it
        return VFW_E_INVALIDMEDIATYPE;

    hr = CSourceStream::SetMediaType(pmt);
    if (FAILED(hr))
        return hr;
    
    if (m_IsPPropertiesValid == true)
    {
        SuggestAllocatorProperties(& m_pProperties);
    }
    
    hr = m_pParent->SetMediaType(pmt);
    return hr;
}

// See Directshow help topic for IAMStreamConfig for details on this method
HRESULT VirtualCamDeviceStream::GetMediaType(int iPosition, CMediaType *pmt)
{
    /*
    if(iPosition < 0)
        return E_INVALIDARG;
    if(iPosition > c_iFormatsCount) 
        return VFW_S_NO_MORE_ITEMS;

    if(iPosition == 0) // ros: i'm not sure that we can link default mt as result without checking is it really video pin connected to filter?
    {
        *pmt = m_mt;
        return S_OK;
    }

    DECLARE_PTR(VIDEOINFOHEADER, pvi, pmt->AllocFormatBuffer(sizeof(VIDEOINFOHEADER)));
    ZeroMemory(pvi, sizeof(VIDEOINFOHEADER));

    pvi->bmiHeader.biCompression = BI_RGB;
    pvi->bmiHeader.biBitCount    = c_nDefaultBitCount;
    pvi->bmiHeader.biSize       = sizeof(BITMAPINFOHEADER);
    pvi->bmiHeader.biWidth      = c_nMinWidth + c_nGranularityW * (iPosition - 1);
    pvi->bmiHeader.biHeight     = c_nMinHeight + c_nGranularityH * (iPosition - 1);
    pvi->bmiHeader.biPlanes     = 1;
    pvi->bmiHeader.biSizeImage  = GetBitmapSize(&pvi->bmiHeader);
    pvi->bmiHeader.biClrImportant = 0;

    pvi->AvgTimePerFrame = UNITS / c_fps;

    SetRectEmpty(&(pvi->rcSource)); // we want the whole image area rendered.
    SetRectEmpty(&(pvi->rcTarget)); // no particular destination rectangle

    pmt->SetType(&MEDIATYPE_Video);
    pmt->SetFormatType(&FORMAT_VideoInfo);
    pmt->SetTemporalCompression(FALSE);

    // Work out the GUID for the subtype from the header info.
    const GUID SubTypeGUID = GetBitmapSubtype(&pvi->bmiHeader);
    pmt->SetSubtype(&SubTypeGUID);
    pmt->SetSampleSize(pvi->bmiHeader.biSizeImage);
    
    return NOERROR;
    */
    //return reinterpret_cast<VirtualCamDevice*>(m_pFilter)->GetMediaType(iPosition, pmt);
    return m_pParent->GetMediaType(iPosition, pmt);

} // GetMediaType

// This method is called to see if a given output format is supported
#ifdef NATIVE_CODE
HRESULT VirtualCamDeviceStream::CheckMediaType(const CMediaType *pMediaType)
{
    VIDEOINFOHEADER *pvi = (VIDEOINFOHEADER *)(pMediaType->Format());
    if(*pMediaType != m_mt) 
        return E_INVALIDARG;
    return S_OK;
} // CheckMediaType
#else // NATIVE_CODE
HRESULT VirtualCamDeviceStream::CheckMediaType(const CMediaType *pMediaType)
{
    /*
    VIDEOINFOHEADER *pvi = (VIDEOINFOHEADER *)(pMediaType->Format());
    if(*pMediaType != m_mt) 
        return E_INVALIDARG;
    return S_OK;
    */
    return m_pParent->CheckMediaType (pMediaType);
}
#endif // NATIVE_CODE
// This method is called after the pins are connected to allocate buffers to stream data
HRESULT VirtualCamDeviceStream::DecideBufferSize(IMemAllocator *pAlloc, ALLOCATOR_PROPERTIES *pProperties)
{
    /*
    CAutoLock cAutoLock(m_pFilter->pStateLock());
    HRESULT hr = NOERROR;

    VIDEOINFOHEADER *pvi = (VIDEOINFOHEADER *) m_mt.Format();
    pProperties->cBuffers = 1;
    pProperties->cbBuffer = pvi->bmiHeader.biSizeImage;

    ALLOCATOR_PROPERTIES Actual;
    hr = pAlloc->SetProperties(pProperties,&Actual);

    if(FAILED(hr)) return hr;
    if(Actual.cbBuffer < pProperties->cbBuffer) return E_FAIL;

    return NOERROR;
    */
    CAutoLock cAutoLock(m_pFilter->pStateLock());
    if (!IsConnected())
        return VFW_E_NOT_CONNECTED;
    ALLOCATOR_PROPERTIES _actual;
    HRESULT hr = GetAllocatorProperties(& _actual);

    if (SUCCEEDED(hr) && _actual.cBuffers <= pProperties->cBuffers && _actual.cbBuffer <= pProperties->cbBuffer && _actual.cbAlign == pProperties->cbAlign)
    {
        ALLOCATOR_PROPERTIES Actual;
        hr = pAlloc->SetProperties(pProperties, & Actual);
        if (SUCCEEDED(hr))
        {
            pProperties->cbAlign  = Actual.cbAlign;
            pProperties->cbBuffer = Actual.cbBuffer;
            pProperties->cbPrefix = Actual.cbPrefix;
            pProperties->cBuffers = Actual.cBuffers;
        }
    }
    return m_pParent->DecideBufferSize(pAlloc, pProperties);
} // DecideBufferSize

// Called when graph is run
HRESULT VirtualCamDeviceStream::OnThreadCreate()
{
    m_rtLastTime = 0;
    m_rtLastReceivedTime = 0;
    m_delayInReceived = 0;

	hMapFile = OpenFileMapping(FILE_MAP_ALL_ACCESS,   // read/write access
								FALSE,                 // do not inherit the name
								TEXT("Local\\VirtualCamDeviceStream"));               // name of mapping object
    /*
        We do not need notify about problem because in this case user will see answer via "snow" frames on the preview

	    if (hMapFile == NULL)
		    Msg(L"Cannot open file mapping");
    */


    return NOERROR;
} // OnThreadCreate


//////////////////////////////////////////////////////////////////////////
//  IAMStreamConfig
//////////////////////////////////////////////////////////////////////////
// ros: actually this methid greatly differs to analogue solution here and should be checked
// https://www.codeproject.com/Articles/437617/DirectShow-Virtual-Video-Capture-Source-Filter-in
#ifdef NATIVE_CODE
HRESULT STDMETHODCALLTYPE VirtualCamDeviceStream::SetFormat(AM_MEDIA_TYPE *pmt)
{
    if (this->m_pFilter->IsActive()) return VFW_E_WRONG_STATE; // ros: i've added this line

    DECLARE_PTR(VIDEOINFOHEADER, pvi, m_mt.pbFormat);
    m_mt = *pmt;
    IPin* pin; 
    ConnectedTo(&pin);
    if(pin)
    {
        IFilterGraph *pGraph = m_pParent->GetGraph();
        pGraph->Reconnect(this);
    }
    return S_OK;
}
#else // NATIVE_CODE
/*
https://www.codeproject.com/Articles/437617/DirectShow-Virtual-Video-Capture-Source-Filter-in
Here the _newType variable is the setted media type, but it can be partially configured by caller, for example only changed 
colorspace (example: RGB32 to YUY2 should recalc image size and reconfig BitsPerPixel) or modified resolution without changing 
image size. I didn't made handling that situations but you can do configuring this new type before passing in to CheckMediaType, 
as in case of partially initialize, type will be rejected in that method. Most application puts proper types but situations with 
partially initialized type is possible.
*/
HRESULT STDMETHODCALLTYPE VirtualCamDeviceStream::SetFormat(AM_MEDIA_TYPE *pmt)
{
    if (m_pFilter->IsActive()) return VFW_E_WRONG_STATE;

    HRESULT hr;
    CMediaType _newType (*pmt);
    CMediaType _oldType (m_mt);

    hr = (HRESULT)CheckMediaType(& _newType);
    if (FAILED(hr)) return hr;
    m_mt.Set(_newType);

    if (IsConnected())
    {
//        hr = (HRESULT)Connected.QueryAccept(_newType);
        hr = (HRESULT)GetConnected()->QueryAccept(& _newType);

        if (SUCCEEDED(hr))
        {
            hr = (HRESULT)m_pFilter->ReconnectPin(this, & _newType);
            if (SUCCEEDED(hr))
            {
//                hr = (HRESULT)(m_pFilter as VirtualCamFilter).SetMediaType(_newType);
                hr = m_pParent->SetMediaType(& _newType);
            }
            else
            {
                m_mt.Set(_oldType);
                m_pFilter->ReconnectPin(this, & _oldType);
            }
        }
    }
    else
    {
//        hr = (HRESULT)(m_Filter as VirtualCamFilter).SetMediaType(_newType);
        hr = m_pParent->SetMediaType(& _newType);
    }
    return hr;
}
#endif // NATIVE_CODE

HRESULT STDMETHODCALLTYPE VirtualCamDeviceStream::GetFormat(AM_MEDIA_TYPE **ppmt)
{
    *ppmt = CreateMediaType(&m_mt);
    return S_OK;
}

HRESULT STDMETHODCALLTYPE VirtualCamDeviceStream::GetNumberOfCapabilities(int *piCount, int *piSize)
{
    *piCount = c_iFormatsCount;
    *piSize = sizeof(VIDEO_STREAM_CONFIG_CAPS);
    return S_OK;
}

HRESULT STDMETHODCALLTYPE VirtualCamDeviceStream::GetStreamCaps(int iIndex, AM_MEDIA_TYPE **pmt, BYTE *pSCC)
{
// ros:    if (iIndex == 0) iIndex = 4;
//	if (iIndex == 0) iIndex = c_iFormatsCount;

    *pmt = CreateMediaType(&m_mt);

    /*
    DECLARE_PTR(VIDEOINFOHEADER, pvi, (*pmt)->pbFormat);
    pvi->bmiHeader.biCompression = BI_RGB;
    pvi->bmiHeader.biBitCount    = c_nDefaultBitCount;
    pvi->bmiHeader.biSize       = sizeof(BITMAPINFOHEADER);
    pvi->bmiHeader.biWidth      = c_nMinWidth + iIndex > 0 ? c_nGranularityW * (iIndex - 1) : 0;
    pvi->bmiHeader.biHeight     = c_nMinHeight + iIndex > 0 ? c_nGranularityH * (iIndex - 1) : 0;
    pvi->bmiHeader.biPlanes     = 1;
    pvi->bmiHeader.biSizeImage  = GetBitmapSize(&pvi->bmiHeader);
    pvi->bmiHeader.biClrImportant = 0;

    SetRectEmpty(&(pvi->rcSource)); // we want the whole image area rendered.
    SetRectEmpty(&(pvi->rcTarget)); // no particular destination rectangle

    (*pmt)->majortype = MEDIATYPE_Video;
    (*pmt)->subtype = MEDIASUBTYPE_RGB24;
    (*pmt)->formattype = FORMAT_VideoInfo;
    (*pmt)->bTemporalCompression = FALSE;
    (*pmt)->bFixedSizeSamples= FALSE;
    (*pmt)->lSampleSize = pvi->bmiHeader.biSizeImage;
    (*pmt)->cbFormat = sizeof(VIDEOINFOHEADER);
    

    DECLARE_PTR(VIDEO_STREAM_CONFIG_CAPS, pvscc, pSCC);
    HRESULT hr = m_pParent->GetDefaultCaps(iIndex, pvscc);

    return S_OK;
    */
    DECLARE_PTR(CMediaType, cmt, *pmt);
    return m_pParent->GetStreamCaps(iIndex, cmt, pSCC);
}

//////////////////////////////////////////////////////////////////////////
// IAMLatency
//////////////////////////////////////////////////////////////////////////
HRESULT STDMETHODCALLTYPE VirtualCamDeviceStream::GetLatency( REFERENCE_TIME *prtLatency)
{
    return m_pParent->GetLatency(prtLatency);
}


//////////////////////////////////////////////////////////////////////////
// IAMBufferNegotiation
//////////////////////////////////////////////////////////////////////////
HRESULT VirtualCamDeviceStream::GetAllocatorProperties(ALLOCATOR_PROPERTIES * pprop)
{
    if (pprop == NULL)
        return E_POINTER;

    if (m_IsPPropertiesValid == true)
    {
        pprop->cbAlign = m_pProperties.cbAlign;
        pprop->cbBuffer = m_pProperties.cbBuffer;
        pprop->cbPrefix = m_pProperties.cbPrefix;
        pprop->cBuffers = m_pProperties.cBuffers;
        return NOERROR;
    }
    if (IsConnected())
    {
        HRESULT hr = this->m_pAllocator->GetProperties(pprop);
        if (SUCCEEDED(hr) && pprop->cBuffers > 0 && pprop->cbBuffer > 0)
            return hr;
    }
    return m_pParent->GetAllocatorProperties(pprop);
}
HRESULT VirtualCamDeviceStream::SuggestAllocatorProperties(const ALLOCATOR_PROPERTIES *pprop)
{
    if (IsConnected())
        return VFW_E_ALREADY_CONNECTED;

    HRESULT hr = m_pParent->SuggestAllocatorProperties(pprop);
    if (FAILED(hr))
    {
        m_IsPPropertiesValid = false;
        return hr;
    }
    if (m_IsPPropertiesValid == false)
    {
        //m_pProperties = new AllocatorProperties();
        m_IsPPropertiesValid = true;
        m_pParent->GetAllocatorProperties(& m_pProperties);
    }
    if (pprop->cbBuffer != -1) m_pProperties.cbBuffer = pprop->cbBuffer;
    if (pprop->cbAlign != -1) m_pProperties.cbAlign = pprop->cbAlign;
    if (pprop->cbPrefix != -1) m_pProperties.cbPrefix = pprop->cbPrefix;
    if (pprop->cBuffers != -1) m_pProperties.cBuffers = pprop->cBuffers;
    return NOERROR;
}


//////////////////////////////////////////////////////////////////////////
//  IAMPushSource
//////////////////////////////////////////////////////////////////////////
HRESULT STDMETHODCALLTYPE VirtualCamDeviceStream::GetPushSourceFlags( ULONG *pFlags)
{
    if (pFlags == NULL)
        return E_POINTER;

    *pFlags = 0;
    return NOERROR;
}
HRESULT STDMETHODCALLTYPE VirtualCamDeviceStream::SetPushSourceFlags( ULONG Flags)
{
    return E_NOTIMPL;
}
HRESULT STDMETHODCALLTYPE VirtualCamDeviceStream::SetStreamOffset( REFERENCE_TIME rtOffset)
{
    CAutoLock cAutoLock(&m_csPinLock);

    m_rtStreamOffset = rtOffset;
    if (m_rtStreamOffset > m_rtStreamOffsetMax)
        m_rtStreamOffsetMax = m_rtStreamOffset;                

    return NOERROR;
}
HRESULT STDMETHODCALLTYPE VirtualCamDeviceStream::GetStreamOffset( REFERENCE_TIME *prtOffset)
{
    if (prtOffset == NULL)
        return E_POINTER;

    *prtOffset = m_rtStreamOffset;

    return NOERROR;
}
HRESULT STDMETHODCALLTYPE VirtualCamDeviceStream::GetMaxStreamOffset( REFERENCE_TIME *prtMaxOffset)
{
    if (prtMaxOffset == NULL)
        return E_POINTER;

    *prtMaxOffset = 0;
    if (m_rtStreamOffsetMax == -1)
    {
        HRESULT hr = (HRESULT)GetLatency(& m_rtStreamOffsetMax);
        if (FAILED(hr)) return hr;
        if (m_rtStreamOffsetMax < m_rtStreamOffset) m_rtStreamOffsetMax = m_rtStreamOffset;
    }
    *prtMaxOffset = m_rtStreamOffsetMax;
    return NOERROR;
}
HRESULT STDMETHODCALLTYPE VirtualCamDeviceStream::SetMaxStreamOffset( REFERENCE_TIME rtMaxOffset)
{
    if (rtMaxOffset < m_rtStreamOffset)
        return E_INVALIDARG;
    
    m_rtStreamOffsetMax = rtMaxOffset;

    return NOERROR;
}

//////////////////////////////////////////////////////////////////////////
//  IAMStreamControl
//////////////////////////////////////////////////////////////////////////
HRESULT VirtualCamDeviceStream::StartAt( const REFERENCE_TIME *ptStart, DWORD dwCookie)
{
    CAutoLock cAutoLock(&m_csPinLock);

    if (ptStart != NULL && *ptStart != LONG_MAX)
    {
        m_rtStartAt = *ptStart;
        m_dwStartCookie = dwCookie;
    }
    else
    {
        m_rtStartAt = -1;
        m_dwStartCookie = 0;
    }

	return NOERROR;
}

HRESULT VirtualCamDeviceStream::StopAt( const REFERENCE_TIME *ptStop, BOOL bSendExtra, DWORD dwCookie)
{
    CAutoLock cAutoLock(&m_csPinLock);
    
    if (ptStop != NULL && *ptStop != LONG_MAX)
    {
        m_rtStopAt = *ptStop;
        m_bShouldFlush = bSendExtra;
        m_dwStopCookie = dwCookie;
    }
    else
    {
        m_rtStopAt = -1;
        m_bShouldFlush = false;
        m_dwStopCookie = 0;
    }
    
	return NOERROR;
}

HRESULT VirtualCamDeviceStream::GetInfo( AM_STREAM_INFO *pInfo)
{
    CAutoLock cAutoLock(&m_csPinLock);

	pInfo = new AM_STREAM_INFO;

    pInfo->dwFlags = 0; // AM_STREAM_INFO_FLAGS

	if (m_rtStart < m_rtStartAt)
	{
        pInfo->dwFlags = pInfo->dwFlags | AM_STREAM_INFO_DISCARDING;
	}
	if (m_rtStartAt != -1)
	{
        pInfo->dwFlags       = pInfo->dwFlags | AM_STREAM_INFO_START_DEFINED;
		pInfo->tStart		= m_rtStartAt;
		pInfo->dwStartCookie	= m_dwStartCookie;
	}
	if (m_rtStopAt != -1)
	{
        pInfo->dwFlags       = pInfo->dwFlags | AM_STREAM_INFO_STOP_DEFINED;
		pInfo->tStop			= m_rtStopAt;
		pInfo->dwStopCookie	= m_dwStopCookie;
	}
    if (m_bShouldFlush) pInfo->dwFlags = pInfo->dwFlags |  AM_STREAM_INFO_STOP_SEND_EXTRA;

    return NOERROR;
}


//////////////////////////////////////////////////////////////////////////
// IKsPropertySet
//////////////////////////////////////////////////////////////////////////


HRESULT VirtualCamDeviceStream::Set(REFGUID guidPropSet, DWORD dwID, void *pInstanceData, 
                        DWORD cbInstanceData, void *pPropData, DWORD cbPropData)
{// Set: Cannot set any properties.
    return E_NOTIMPL;
}

// Get: Return the pin category (our only property). 
HRESULT VirtualCamDeviceStream::Get(
    REFGUID guidPropSet,   // Which property set.
    DWORD dwPropID,        // Which property in that set.
    void *pInstanceData,   // Instance data (ignore).
    DWORD cbInstanceData,  // Size of the instance data (ignore).
    void *pPropData,       // Buffer to receive the property data.
    DWORD cbPropData,      // Size of the buffer.
    DWORD *pcbReturned     // Return the size of the property.
)
{
    if (guidPropSet != AMPROPSETID_Pin)             return E_PROP_SET_UNSUPPORTED;
    if (dwPropID != AMPROPERTY_PIN_CATEGORY)        return E_PROP_ID_UNSUPPORTED;
    if (pPropData == NULL && pcbReturned == NULL)   return E_POINTER;
    
    if (pcbReturned) *pcbReturned = sizeof(GUID);
    if (pPropData == NULL)          return S_OK; // Caller just wants to know the size. 
    if (cbPropData < sizeof(GUID))  return E_UNEXPECTED;// The buffer is too small.
        
    *(GUID *)pPropData = PIN_CATEGORY_CAPTURE;
    return S_OK;
}

// QuerySupported: Query whether the pin supports the specified property.
HRESULT VirtualCamDeviceStream::QuerySupported(REFGUID guidPropSet, DWORD dwPropID, DWORD *pTypeSupport)
{
    if (guidPropSet != AMPROPSETID_Pin) return E_PROP_SET_UNSUPPORTED;
    if (dwPropID != AMPROPERTY_PIN_CATEGORY) return E_PROP_ID_UNSUPPORTED;
    // We support getting this property, but not setting it.
    if (pTypeSupport) *pTypeSupport = KSPROPERTY_SUPPORT_GET; 
    return S_OK;
}
