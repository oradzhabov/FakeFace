#include <olectl.h>
#include <strsafe.h>

#define MSG_CAPTION TEXT("Virtual Cam Device Message")


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

    MessageBox(NULL, szBuffer, MSG_CAPTION, MB_OK | MB_ICONERROR);
}

const unsigned int ALIGN16(const unsigned int & rhs)
{
    unsigned int value = rhs % 16;
    return value > 0 ? rhs - value + 16 : rhs;
}