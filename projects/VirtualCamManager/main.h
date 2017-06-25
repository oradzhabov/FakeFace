#pragma warning(disable : 4995) 

// #define _WIN32_WINNT 0x0500

#include <windows.h>
#include <dshow.h>
#include <stdio.h>
#include <strsafe.h>

#include <vector>
#include <string>

//
// Function prototypes
//
int PASCAL WinMain(HINSTANCE hInstance, HINSTANCE hInstP, LPSTR lpCmdLine, int nCmdShow);
LRESULT CALLBACK WndMainProc (HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam);

std::wstring s2ws(const std::string& str);
std::string ws2s(const std::wstring& wstr);


HRESULT GetInterfaces(void);
HRESULT CaptureVideo();
HRESULT FindCaptureDevice(IBaseFilter ** ppSrcFilter, const int selectIndex, std::vector<std::wstring> & names);
HRESULT CreateVideoGraph(IBaseFilter ** ppLastFilterinGraph);
HRESULT SetupVideoWindow(void);
HRESULT ChangePreviewState(int nShow);
HRESULT HandleGraphEvent(void);

void Msg(TCHAR *szFormat, ...);
void CloseInterfaces(void);
void ResizeVideoWindow(void);

// Remote graph viewing functions
#ifdef REGISTER_FILTERGRAPH
    HRESULT AddGraphToRot(IUnknown *pUnkGraph, DWORD *pdwRegister);
    void RemoveGraphFromRot(DWORD pdwRegister);
#endif // REGISTER_FILTERGRAPH
enum PLAYSTATE {Stopped, Paused, Running, Init};

//
// Macros
//
#define SAFE_RELEASE(x) { if (x) x->Release(); x = NULL; }

#define JIF(x) if (FAILED(hr=(x))) \
    {Msg(TEXT("FAILED(hr=0x%x) in ") TEXT(#x) TEXT("\n\0"), hr); return hr;}

//
// Constants
//
#define DEFAULT_VIDEO_WIDTH     320
#define DEFAULT_VIDEO_HEIGHT    320

#define APPLICATIONNAME TEXT("FakeFace Manager\0")
#define CLASSNAME       TEXT("FakeFaceManager\0")

// Application-defined message to notify app of filtergraph events
#define WM_GRAPHNOTIFY  WM_APP+1

//
HRESULT CreateFilter(WCHAR *Name, IBaseFilter **Filter, REFCLSID FilterCategory);

// Return the first unconnected input pin or output pin.
HRESULT FindUnconnectedPin(IBaseFilter *pFilter, PIN_DIRECTION PinDir, IPin **ppPin);

// Connect output pin to filter.
HRESULT ConnectFilters(
    IGraphBuilder *pGraph, // Filter Graph Manager.
    IPin *pOut,            // Output pin on the upstream filter.
    IBaseFilter *pDest);    // Downstream filter.

//HRESULT MakeCamToDvGraph();