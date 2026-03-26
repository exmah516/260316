#pragma once

#include <stdio.h>
#include <string.h>
#include <memory>
#include <iostream>
#include <string>
#include <unordered_map>
#include <windows.h>
#include "TcAdsDef.h"
#include "TcAdsAPI.h"

class CADSComm
{
public:
	CADSComm(void);
	~CADSComm(void);

	bool OpenComm();
	bool OpenComm_inside();

	bool CloseComm();
	bool ADSWrite(const char * paraName, unsigned long length, void * data);
	bool ADSRead(const char * paraName, unsigned long length, void * data);
	bool ADSReadSum(const char* const* symbols, const unsigned long* lengths, void* const* outputs, unsigned long count);
	bool ADSWrite(unsigned long addr, unsigned long length, void * data);
	bool ADSRead(unsigned long addr, unsigned long length, void * data);
	unsigned long ADSGetAddr(const char * paraName);
	bool IsCommOpen()
	{
		return m_bOpen;
	}

	const char * GetLastError()
	{
		return m_lastError;
	}

private:

	PAmsAddr		m_PAmsAddr;
	long			m_adsPort;
	std::unordered_map<std::string, unsigned long> m_symbolHandles;
	bool			m_bOpen;
	char 			m_lastError[256];
};
