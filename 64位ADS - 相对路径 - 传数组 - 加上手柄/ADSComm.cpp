#include <ADSComm1.h>

CADSComm::CADSComm(void)
{
	m_bOpen=false;
	m_PAmsAddr =new AmsAddr;
	m_adsPort = 0;
	memset(m_lastError,0,sizeof(char)*256);
}

CADSComm::~CADSComm(void)
{
	CloseComm();
	delete m_PAmsAddr;
}

bool CADSComm::ADSWrite(const char * paraName, unsigned long length, void * data)//plc��������sizeof(����)��c++Ҫ����Ķ����ĵ�ַ
{
	const auto addr = ADSGetAddr(paraName);
	if (addr == 0)
	{
		return false;
	}
	return ADSWrite(addr, length, data);
}
bool CADSComm::ADSRead(const char * paraName, unsigned long length, void * data)
{
	const auto addr = ADSGetAddr(paraName);
	if (addr == 0)
	{
		return false;
	}
	return ADSRead(addr, length, data);
}
bool CADSComm::ADSWrite(unsigned long addr , unsigned long length, void * data)
{
	if (!m_bOpen)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: Ads not Open\n");
		return false;
	}
	long nErr;
	nErr = AdsSyncWriteReqEx(m_adsPort, m_PAmsAddr, ADSIGRP_SYM_VALBYHND, addr, length, data);
	if (nErr)
	{ 
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsSyncWriteReqEx: %ld\n", nErr);
		return false;
	}
	return true;
}
bool CADSComm::ADSRead(unsigned long addr , unsigned long length, void * data)
{
	if (!m_bOpen)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: Ads not Open\n");
		return false;
	}
	long nErr;
	unsigned long cbReturn = 0;
	nErr = AdsSyncReadReqEx2(m_adsPort, m_PAmsAddr, ADSIGRP_SYM_VALBYHND, addr, length, data, &cbReturn);
	if (nErr)
	{ 
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsSyncReadReqEx2: %ld\n", nErr);
		return false;
	}
	return true;
}

unsigned long CADSComm::ADSGetAddr(const char * paraName)
{
	if (!m_bOpen)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: Ads not Open\n");
		return 0;
	}
	const std::string key = paraName ? std::string(paraName) : std::string();
	if (key.empty())
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: Empty symbol name\n");
		return 0;
	}

	const auto it = m_symbolHandles.find(key);
	if (it != m_symbolHandles.end())
	{
		return it->second;
	}

	long nErr;
	unsigned long lHdlVar = 0;
	unsigned long cbReturn = 0;
	const unsigned long nameLen = static_cast<unsigned long>(key.size() + 1);
	void* namePtr = const_cast<void*>(static_cast<const void*>(key.c_str()));
	nErr = AdsSyncReadWriteReqEx2(
		m_adsPort,
		m_PAmsAddr,
		ADSIGRP_SYM_HNDBYNAME,
		0,
		sizeof(lHdlVar),
		&lHdlVar,
		nameLen,
		namePtr,
		&cbReturn);
	if (nErr)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsSyncReadWriteReqEx2(HNDBYNAME): %ld\n", nErr);
		return 0;
	}

	m_symbolHandles.emplace(key, lHdlVar);
	return lHdlVar;
}

bool CADSComm::OpenComm()
{
	m_bOpen = false;
	long nErr;
	USHORT   nAdsState;

	//AmsNetId _AmsId = { 5, 100, 219, 36, 1, 1 };
	AmsNetId _AmsId = { 169, 254, 119, 135, 1, 1 }; 

	m_PAmsAddr->netId = _AmsId;			// external comm

	m_PAmsAddr->port = 851;

	m_adsPort = AdsPortOpenEx();
	if (m_adsPort <= 0)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsPortOpenEx failed\n");
		return false;
	}

	USHORT        nDeviceState;
	nErr = AdsSyncReadStateReqEx(m_adsPort, m_PAmsAddr, &nAdsState, &nDeviceState);

	if (nErr)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsSyncReadStateReqEx: %ld\n", nErr);
		AdsPortCloseEx(m_adsPort);
		m_adsPort = 0;
		return false;
	}
	if (nAdsState == 6)
	{
		nAdsState = ADSSTATE_RUN;
		nErr = AdsSyncWriteControlReqEx(m_adsPort, m_PAmsAddr, nAdsState, nDeviceState, 0, NULL);
		if (nErr)
		{
			sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsSyncWriteControlReqEx: %ld\n", nErr);
			AdsPortCloseEx(m_adsPort);
			m_adsPort = 0;
			return false;
		}
	}
	m_bOpen = true;

	return true;
}

bool CADSComm::OpenComm_inside()
{
	m_bOpen = false;
	long nErr;
	USHORT   nAdsState;

	m_adsPort = AdsPortOpenEx();
	if (m_adsPort <= 0)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsPortOpenEx failed\n");
		return false;
	}

	nErr = AdsGetLocalAddressEx(m_adsPort, m_PAmsAddr);
	if (nErr)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsGetLocalAddressEx: %ld\n", nErr);
		AdsPortCloseEx(m_adsPort);
		m_adsPort = 0;
		return false;
	}

	m_PAmsAddr->port = 851;

	USHORT        nDeviceState;
	nErr = AdsSyncReadStateReqEx(m_adsPort, m_PAmsAddr, &nAdsState, &nDeviceState);
	if (nErr)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsSyncReadStateReqEx: %ld\n", nErr);
		AdsPortCloseEx(m_adsPort);
		m_adsPort = 0;
		return false;
	}
	if (nAdsState == 6)
	{
		nAdsState = ADSSTATE_RUN;
		nErr = AdsSyncWriteControlReqEx(m_adsPort, m_PAmsAddr, nAdsState, nDeviceState, 0, NULL);
		if (nErr)
		{
			sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsSyncWriteControlReqEx: %ld\n", nErr);
			AdsPortCloseEx(m_adsPort);
			m_adsPort = 0;
			return false;
		}
	}
	m_bOpen = true;
	return true;
}
bool CADSComm::CloseComm()
{
	if (!m_bOpen && m_adsPort == 0)
	{
		return true;
	}

	if (m_adsPort > 0 && m_PAmsAddr != nullptr)
	{
		for (const auto& kv : m_symbolHandles)
		{
			unsigned long h = kv.second;
			AdsSyncWriteReqEx(m_adsPort, m_PAmsAddr, ADSIGRP_SYM_RELEASEHND, 0, sizeof(h), &h);
		}
	}
	m_symbolHandles.clear();

	if (m_adsPort > 0)
	{
		AdsPortCloseEx(m_adsPort);
		m_adsPort = 0;
	}
	m_bOpen = false;
	memset(m_lastError, 0, sizeof(m_lastError));
	return true;
}
