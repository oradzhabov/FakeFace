#include "main.h"


// multi byte to wide char:
std::wstring s2ws(const std::string& str)
{
    int size_needed = MultiByteToWideChar(CP_UTF8, 0, &str[0], (int)str.size(), NULL, 0);
    std::wstring wstrTo(size_needed, 0);
    MultiByteToWideChar(CP_UTF8, 0, &str[0], (int)str.size(), &wstrTo[0], size_needed);
    return wstrTo;
}

// wide char to multi byte:
std::string ws2s(const std::wstring& wstr)
{
    int size_needed = WideCharToMultiByte(CP_ACP, 0, wstr.c_str(), int(wstr.length() + 1), 0, 0, 0, 0); 
    std::string strTo(size_needed, 0);
    WideCharToMultiByte(CP_ACP, 0, wstr.c_str(), int(wstr.length() + 1), &strTo[0], size_needed, 0, 0); 
    return strTo;
}

HRESULT FindPinInterface(
    IBaseFilter *pFilter,  // Pointer to the filter to search.
    REFGUID iid,           // IID of the interface.
    void **ppUnk)          // Receives the interface pointer.
{
    if (!pFilter || !ppUnk) return E_POINTER;

    HRESULT hr = E_FAIL;
    IEnumPins *pEnum = 0;
    if (FAILED(pFilter->EnumPins(&pEnum)))
    {
        return E_FAIL;
    }
    // Query every pin for the interface.
    IPin *pPin = 0;
    while (S_OK == pEnum->Next(1, &pPin, 0))
    {
        hr = pPin->QueryInterface(iid, ppUnk);
        pPin->Release();
        if (SUCCEEDED(hr))
        {
            break;
        }
    }
    pEnum->Release();
    return hr;
}
HRESULT FindInterfaceAnywhere(
    IGraphBuilder *pGraph, 
    REFGUID iid, 
    void **ppUnk)
{
    if (!pGraph || !ppUnk) return E_POINTER;
    HRESULT hr = E_FAIL;
    IEnumFilters *pEnum = 0;
    if (FAILED(pGraph->EnumFilters(&pEnum)))
    {
        return E_FAIL;
    }
    // Loop through every filter in the graph.
    IBaseFilter *pF = 0;
    while (S_OK == pEnum->Next(1, &pF, 0))
    {
        hr = pF->QueryInterface(iid, ppUnk);
        if (FAILED(hr))
        {
            // The filter does not expose the interface, but maybe
            // one of its pins does.
            hr = FindPinInterface(pF, iid, ppUnk);
        }
        pF->Release();
        if (SUCCEEDED(hr))
        {
            break;
        }
    }
    pEnum->Release();
    return hr;
}

// Query whether a pin is connected to another pin.
//
// Note: This function does not return a pointer to the connected pin.

HRESULT IsPinConnected(IPin *pPin, BOOL *pResult)
{
    IPin *pTmp = NULL;
    HRESULT hr = pPin->ConnectedTo(&pTmp);
    if (SUCCEEDED(hr))
    {
        *pResult = TRUE;
    }
    else if (hr == VFW_E_NOT_CONNECTED)
    {
        // The pin is not connected. This is not an error for our purposes.
        *pResult = FALSE;
        hr = S_OK;
    }

//    SafeRelease(&pTmp);
	SAFE_RELEASE(pTmp)
    return hr;
}

// Query whether a pin has a specified direction (input / output)
HRESULT IsPinDirection(IPin *pPin, PIN_DIRECTION dir, BOOL *pResult)
{
    PIN_DIRECTION pinDir;
    HRESULT hr = pPin->QueryDirection(&pinDir);
    if (SUCCEEDED(hr))
    {
        *pResult = (pinDir == dir);
    }
    return hr;
}



// Match a pin by pin direction and connection state.
HRESULT MatchPin(IPin *pPin, PIN_DIRECTION direction, BOOL bShouldBeConnected, BOOL *pResult)
{
//    assert(pResult != NULL);

    BOOL bMatch = FALSE;
    BOOL bIsConnected = FALSE;

    HRESULT hr = IsPinConnected(pPin, &bIsConnected);
    if (SUCCEEDED(hr))
    {
        if (bIsConnected == bShouldBeConnected)
        {
            hr = IsPinDirection(pPin, direction, &bMatch);
        }
    }

    if (SUCCEEDED(hr))
    {
        *pResult = bMatch;
    }
    return hr;
}
// Return the first unconnected input pin or output pin.
HRESULT FindUnconnectedPin(IBaseFilter *pFilter, PIN_DIRECTION PinDir, IPin **ppPin)
{
    IEnumPins *pEnum = NULL;
    IPin *pPin = NULL;
    BOOL bFound = FALSE;

    HRESULT hr = pFilter->EnumPins(&pEnum);
    if (FAILED(hr))
    {
        goto done;
    }

    while (S_OK == pEnum->Next(1, &pPin, NULL))
    {
        hr = MatchPin(pPin, PinDir, FALSE, &bFound);
        if (FAILED(hr))
        {
            goto done;
        }
        if (bFound)
        {
            *ppPin = pPin;
            (*ppPin)->AddRef();
            break;
        }
        //SafeRelease(&pPin);
		SAFE_RELEASE(pPin);
    }

    if (!bFound)
    {
        hr = VFW_E_NOT_FOUND;
    }

done:
    //SafeRelease(&pPin);
	SAFE_RELEASE(pPin);
    //SafeRelease(&pEnum);
	SAFE_RELEASE(pEnum);
    return hr;
}

// Connect output pin to filter.
HRESULT ConnectFilters(
    IGraphBuilder *pGraph, // Filter Graph Manager.
    IPin *pOut,            // Output pin on the upstream filter.
    IBaseFilter *pDest)    // Downstream filter.
{
    IPin *pIn = NULL;
        
    // Find an input pin on the downstream filter.
    HRESULT hr = FindUnconnectedPin(pDest, PINDIR_INPUT, &pIn);
    if (SUCCEEDED(hr))
    {
        // Try to connect them.
        hr = pGraph->Connect(pOut, pIn);
        pIn->Release();
    }
    return hr;
}


/// 
/// Create a filter by category and name, and add it to a filter 
/// graph. Will enumerate all filters of the given category and 
/// add the filter whose name matches, if any. If the filter could be 
/// created but not added to the graph, the filter is destroyed. 
/// 
/// @param Graph Filter graph. 
/// @param Name of filter to create. 
/// @param Filter Receives a pointer to the filter. 
/// @param FilterCategory Filter category. 
/// @param NameInGraph Name for the filter in the graph, or 0 for no 
/// name. 
/// 
/// @return true if successful. 
bool AddFilter(IFilterGraph *Graph, WCHAR *Name, 
   IBaseFilter **Filter, REFCLSID FilterCategory, 
   const WCHAR *NameInGraph) 
{ 
//   ASSERT(Graph); 
//   assert(Name); 
//   assert(Filter); 
//   assert(!*Filter);   

   if (FAILED(CreateFilter(Name, Filter, FilterCategory)) )
      return false;   

   if (FAILED(Graph->AddFilter(*Filter, NameInGraph))) 
   { 
      (*Filter)->Release(); 
      *Filter = 0; 
      return false; 
   }   

   return true; 
}   

// https://sid6581.wordpress.com/2006/10/12/finding-directshow-filters-by-name/
// https://msdn.microsoft.com/ru-ru/library/windows/desktop/dd407292(v=vs.85).aspx
// Filter Categories
// https://msdn.microsoft.com/en-us/library/windows/desktop/dd375655(v=vs.85).aspx
// sample grabbing
// https://www.codeproject.com/Articles/28790/Creating-Custom-DirectShow-SampleGrabber-Filter-fo
// 
/// 
/// Create a filter by category and name. Will enumerate all filters 
/// of the given category and return the filter whose name matches, 
/// if any. 
/// 
/// @param Name of filter to create. 
/// @param Filter Will receive the pointer to the interface 
/// for the created filter. 
/// @param FilterCategory Filter category. 
/// 
/// @return true if successful. 
HRESULT CreateFilter(WCHAR *Name, IBaseFilter **ppSrcFilter, REFCLSID FilterCategory) 
{ 
	BSTR bstrVal(Name);
	VARIANT varString;
	varString.vt = VT_BSTR;
	varString.bstrVal = bstrVal;


	// Create the System Device Enumerator.
	HRESULT hr;
	ICreateDevEnum *pSysDevEnum = NULL;
//	hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER,
	hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC,
		IID_ICreateDevEnum, (void **)&pSysDevEnum);
	if (FAILED(hr))
	{
		return hr;
	}

	// Obtain a class enumerator for the video compressor category.
	IEnumMoniker *pEnumCat = NULL;
//	hr = pSysDevEnum->CreateClassEnumerator(CLSID_VideoCompressorCategory, &pEnumCat, 0);
	hr = pSysDevEnum->CreateClassEnumerator(FilterCategory, &pEnumCat, 0);

	if (hr == S_OK) 
	{
		bool found = false;
		// Enumerate the monikers.
		IMoniker *pMoniker = NULL;
		ULONG cFetched;
		while(pEnumCat->Next(1, &pMoniker, &cFetched) == S_OK && found == false)
		{
			IPropertyBag *pPropBag;
			hr = pMoniker->BindToStorage(0, 0, IID_IPropertyBag, 
				(void **)&pPropBag);

			if (SUCCEEDED(hr))
			{
				// To retrieve the filter's friendly name, do the following:
				VARIANT varName;
				VariantInit(&varName);
				hr = pPropBag->Read(L"FriendlyName", &varName, 0);

				if (SUCCEEDED(hr))
				{
					// Display the name in your UI somehow.
//					if (varName == Name)
					LCID lcid = 0;
					ULONG dwFlags = 0;
//					HRESULT hr0 = VarBstrCmp(varName.bstrVal,varString.bstrVal, lcid, dwFlags);
					int a =wcscmp(varName.bstrVal, varString.bstrVal);
					if ( a == 0 )
					{
						// To create an instance of the filter, do the following:
						IBaseFilter *pFilter;
						hr = pMoniker->BindToObject(NULL, NULL, IID_IBaseFilter, (void**)&pFilter);
						// Now add the filter to the graph. 


						// Copy the found filter pointer to the output parameter.
						if (SUCCEEDED(hr))
						{
							*ppSrcFilter = pFilter;
							(*ppSrcFilter)->AddRef();
						}
						found = true;
						SAFE_RELEASE(pFilter);
					}
				}
				VariantClear(&varName);


				//Remember to release pFilter later.
				pPropBag->Release();
			}
			pMoniker->Release();
		}
		pEnumCat->Release();

		if (found == false)
		{
			Msg(TEXT("Couldn't find filter \"%s\""), Name);
			hr = E_FAIL;
		}
	}
	pSysDevEnum->Release();
	return hr;
}
