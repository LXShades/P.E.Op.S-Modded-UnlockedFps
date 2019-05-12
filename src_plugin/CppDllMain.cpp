#include <Windows.h>

/// C++ DllMain used for proper C++ initialisation

extern "C" {BOOL APIENTRY CDllMain(HANDLE hModule, DWORD dwReason, LPVOID lpReserved); }

BOOL APIENTRY DllMain(HANDLE hModule,                  // DLL INIT
	DWORD  dwReason,
	LPVOID lpReserved)
{
	CDllMain(hModule, dwReason, lpReserved);
	return TRUE;
}

