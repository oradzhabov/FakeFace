#pragma once


// {20C9473D-63BA-486D-84E1-22419CB9DDD2}
DEFINE_GUID(CLSID_VirtualCamDevice, 
            0x20c9473d, 0x63ba, 0x486d, 0x84, 0xe1, 0x22, 0x41, 0x9c, 0xb9, 0xdd, 0xd2);


class VirtualCamDeviceStream;
class VirtualCamDevice : public CSource, public IAMFilterMiscFlags
{
public:
    //////////////////////////////////////////////////////////////////////////
    //  IUnknown
    //////////////////////////////////////////////////////////////////////////
    static CUnknown * WINAPI CreateInstance(LPUNKNOWN lpunk, HRESULT *phr);
    STDMETHODIMP QueryInterface(REFIID riid, void **ppv);

    IFilterGraph *GetGraph() {return m_pGraph;}

    /*
        Help Methods
    */
    HRESULT GetDefaultCaps(int nIndex, VIDEO_STREAM_CONFIG_CAPS * _caps);
    HRESULT GetMediaType(int iPosition, CMediaType *pmt);
    HRESULT STDMETHODCALLTYPE  GetStreamCaps(int iIndex, CMediaType *pmt, BYTE *pSCC);
    HRESULT GetLatency( REFERENCE_TIME *prtLatency);
    HRESULT DecideBufferSize(IMemAllocator *pAlloc, ALLOCATOR_PROPERTIES *pProperties);
    HRESULT SetMediaType(const CMediaType *pmt);
    HRESULT CheckMediaType(const CMediaType *pMediaType);
    HRESULT GetAllocatorProperties(ALLOCATOR_PROPERTIES * pprop);
    HRESULT SuggestAllocatorProperties(const ALLOCATOR_PROPERTIES *pprop);

    // IAMFilterMiscFlags
    
    ULONG STDMETHODCALLTYPE GetMiscFlags( void);
    STDMETHODIMP_(ULONG) AddRef() { return GetOwner()->AddRef(); }                                                          \
    STDMETHODIMP_(ULONG) Release() { return GetOwner()->Release(); }
private:
    VirtualCamDevice(LPUNKNOWN lpunk, HRESULT *phr);

    int m_nBitCount;
    long m_nAvgTimePerFrame;
    int m_nWidth;
    int m_nHeight;
    BITMAPINFOHEADER  m_bmi;
};


class VirtualCamDeviceStream :
    public CSourceStream,
    public IAMStreamConfig, // Format configuring.
    public IKsPropertySet,   // For expose pin category.
//    public IAMLatency, // this interface is parent for \IAMPushSource which included to this class too
    public IAMBufferNegotiation,
    public IAMPushSource,
    public IAMStreamControl
{
    friend class VirtualCamDevice;
public:

    //////////////////////////////////////////////////////////////////////////
    //  IUnknown
    //////////////////////////////////////////////////////////////////////////
    STDMETHODIMP QueryInterface(REFIID riid, void **ppv);
    STDMETHODIMP_(ULONG) AddRef() { return GetOwner()->AddRef(); }                                                          \
    STDMETHODIMP_(ULONG) Release() { return GetOwner()->Release(); }

    //////////////////////////////////////////////////////////////////////////
    //  IQualityControl
    //////////////////////////////////////////////////////////////////////////
    STDMETHODIMP Notify(IBaseFilter * pSender, Quality q);

    //////////////////////////////////////////////////////////////////////////
    //  IAMStreamConfig
    //
    // This is major interface which allowing applications to configure output pin format and resolution. 
    // Via this interface filter returns all available resolutions and formats by index in VideoStreamConfigCaps structure. 
    // Along with that structure this interface can returns the mediatype (actual or available by index)
    //
    // Notes:
    // 1. You can return only one media type but proper specify settings in configuration structure - applications are also should 
    // handle that (not all applications doing so - but most professional software works properly).
    // 2. Values in VideoStreamConfigCaps structure also can be different at any index, this means that filter can have output in 
    // different aspect ratio or other parameters which are also beloned to returned MediaType. For example filter can expose 
    // different Width and Height granularity for RGB24 and RGB32.
    // 3. The MediaTypes returned by this interface also can be different from types retrieved via IEnumMediaTypes interface 
    // (application also should handle that). For Example if capture source supports few colorspaces on output it can return 
    // media types with different colorspaces via IAMStreamConfig interface and only with one "active" colorspace via IEnumMediaTypes.
    // An "active" colorspace in that case will be depends on SetFormat call
    //////////////////////////////////////////////////////////////////////////
    HRESULT STDMETHODCALLTYPE SetFormat(AM_MEDIA_TYPE *pmt);
    HRESULT STDMETHODCALLTYPE GetFormat(AM_MEDIA_TYPE **ppmt);
    HRESULT STDMETHODCALLTYPE GetNumberOfCapabilities(int *piCount, int *piSize);
    HRESULT STDMETHODCALLTYPE GetStreamCaps(int iIndex, AM_MEDIA_TYPE **pmt, BYTE *pSCC);

    //////////////////////////////////////////////////////////////////////////
    //  IKsPropertySet
    //
    // Using this interface we specify pin category guid. 
    // This necessary in case you specify category while rendering pin with ICaptureGraphBuilder2 interface or looking for pin 
    // with specified category. And as requirements one pin of such filter should have Capture category
    //////////////////////////////////////////////////////////////////////////
    HRESULT STDMETHODCALLTYPE Set(REFGUID guidPropSet, DWORD dwID, void *pInstanceData, DWORD cbInstanceData, void *pPropData, DWORD cbPropData);
    HRESULT STDMETHODCALLTYPE Get(REFGUID guidPropSet, DWORD dwPropID, void *pInstanceData,DWORD cbInstanceData, void *pPropData, DWORD cbPropData, DWORD *pcbReturned);
    HRESULT STDMETHODCALLTYPE QuerySupported(REFGUID guidPropSet, DWORD dwPropID, DWORD *pTypeSupport);
    

    //////////////////////////////////////////////////////////////////////////
    //  IAMLatency
    //////////////////////////////////////////////////////////////////////////
    HRESULT STDMETHODCALLTYPE GetLatency( REFERENCE_TIME *prtLatency);

    //////////////////////////////////////////////////////////////////////////
    //  IAMBufferNegotiation
    //////////////////////////////////////////////////////////////////////////
    HRESULT STDMETHODCALLTYPE GetAllocatorProperties(ALLOCATOR_PROPERTIES * pprop);
    HRESULT STDMETHODCALLTYPE SuggestAllocatorProperties(const ALLOCATOR_PROPERTIES *pprop);
    

    //////////////////////////////////////////////////////////////////////////
    //  IAMPushSource
    //////////////////////////////////////////////////////////////////////////
    HRESULT STDMETHODCALLTYPE GetPushSourceFlags( ULONG *pFlags);
    HRESULT STDMETHODCALLTYPE SetPushSourceFlags( ULONG Flags);
    HRESULT STDMETHODCALLTYPE SetStreamOffset( REFERENCE_TIME rtOffset);
    HRESULT STDMETHODCALLTYPE GetStreamOffset( REFERENCE_TIME *prtOffset);
    HRESULT STDMETHODCALLTYPE GetMaxStreamOffset( REFERENCE_TIME *prtMaxOffset);
    HRESULT STDMETHODCALLTYPE SetMaxStreamOffset( REFERENCE_TIME rtMaxOffset);

    //////////////////////////////////////////////////////////////////////////
    //  IAMStreamControl
    //////////////////////////////////////////////////////////////////////////
    HRESULT STDMETHODCALLTYPE StartAt( const REFERENCE_TIME *ptStart, DWORD dwCookie);
    HRESULT STDMETHODCALLTYPE StopAt( const REFERENCE_TIME *ptStop, BOOL bSendExtra, DWORD dwCookie);
    HRESULT STDMETHODCALLTYPE GetInfo( AM_STREAM_INFO *pInfo);

    //////////////////////////////////////////////////////////////////////////
    //  CSourceStream
    //////////////////////////////////////////////////////////////////////////
    VirtualCamDeviceStream(HRESULT *phr, VirtualCamDevice *pParent, LPCWSTR pPinName);
    ~VirtualCamDeviceStream();

    HRESULT Active(void);
    HRESULT Inactive(void);
    HRESULT FillBuffer_0(IMediaSample *pms);
    HRESULT FillBuffer(IMediaSample *pms);
    HRESULT DecideBufferSize(IMemAllocator *pIMemAlloc, ALLOCATOR_PROPERTIES *pProperties);
    HRESULT CheckMediaType(const CMediaType *pMediaType);
    HRESULT GetMediaType(int iPosition, CMediaType *pmt);
    HRESULT SetMediaType(const CMediaType *pmt);
    HRESULT OnThreadCreate(void);
    
private:
    VirtualCamDevice *m_pParent;
    REFERENCE_TIME m_rtLastTime;
    REFERENCE_TIME m_rtLastReceivedTime;
    REFERENCE_TIME m_delayInReceived; // period till server did not change the current frame time
//    HBITMAP m_hLogoBmp;
//    CCritSec m_cSharedState;
//    IReferenceClock *m_pClock;
    //
    REFERENCE_TIME m_rtStreamOffset;
    REFERENCE_TIME m_rtStreamOffsetMax;
    //
    ALLOCATOR_PROPERTIES    m_pProperties;
    bool                    m_IsPPropertiesValid;
    //
    //
    CCritSec                m_csPinLock;
    long                    m_rtStart;
    REFERENCE_TIME          m_rtStartAt;
    REFERENCE_TIME          m_rtStopAt;
    bool                    m_bStartNotified;
    bool                    m_bStopNotified;
    DWORD_PTR               m_dwAdviseToken;
    IReferenceClock         * m_pRefClock;
    REFERENCE_TIME          m_rtClockStart;
    REFERENCE_TIME          m_rtClockStop;
//    HSEMAPHORE              m_hSemaphore;
    HANDLE                  m_hSemaphore;
    DWORD                   m_dwStartCookie;
    DWORD                   m_dwStopCookie;
    BOOL                    m_bShouldFlush;
	//
	HANDLE hMapFile;
};


