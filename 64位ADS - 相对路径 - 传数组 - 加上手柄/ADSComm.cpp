#include <ADSComm1.h>
#include <vector>

// ADS 通讯封装说明：
// - 对外提供“按符号名”和“按句柄”两类读写接口。
// - 内部缓存符号句柄，减少重复 HNDBYNAME 查询。
// - CloseComm() 中统一释放句柄并关闭端口，避免 PLC 侧资源泄漏。

CADSComm::CADSComm(void)
{
	m_bOpen = false;
	m_PAmsAddr = new AmsAddr;
	m_adsPort = 0;
	memset(m_lastError, 0, sizeof(char) * 256);
}

CADSComm::~CADSComm(void)
{
	CloseComm();
	delete m_PAmsAddr;
}

// 按符号名写：
// 1) 将符号名解析为句柄（带缓存）；
// 2) 复用按句柄写接口。
bool CADSComm::ADSWrite(const char* paraName, unsigned long length, void* data)
{
	const auto addr = ADSGetAddr(paraName);
	if (addr == 0)
	{
		return false;
	}
	return ADSWrite(addr, length, data);
}

// 按符号名读：
// 1) 将符号名解析为句柄（带缓存）；
// 2) 复用按句柄读接口。
bool CADSComm::ADSRead(const char* paraName, unsigned long length, void* data)
{
	const auto addr = ADSGetAddr(paraName);
	if (addr == 0)
	{
		return false;
	}
	return ADSRead(addr, length, data);
}

// 按句柄写：要求通讯已打开，失败时记录 ADS 错误码。
bool CADSComm::ADSWrite(unsigned long addr, unsigned long length, void* data)
{
	if (!m_bOpen)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: Ads not Open\n");
		return false;
	}

	long nErr = AdsSyncWriteReqEx(m_adsPort, m_PAmsAddr, ADSIGRP_SYM_VALBYHND, addr, length, data);
	if (nErr)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsSyncWriteReqEx: %ld\n", nErr);
		return false;
	}
	return true;
}

// 按句柄读：要求通讯已打开，失败时记录 ADS 错误码。
bool CADSComm::ADSRead(unsigned long addr, unsigned long length, void* data)
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

// 鎸夌鍙峰悕鎵归噺璇伙紙Sum Read锛夛細
// - 涓€娆″彂璧?ADSIGRP_SUMUP_READ锛屽噺灏戠綉缁滃線杩旀鏁般€?
// - 鍏堣В鏋愬苟缂撳瓨鍙ユ焺锛屽啀鎸夋枃妗?request table 鎵归噺璇汇€?
// - 杩斿洖鍖洪娈典负姣忛」閿欒鐮侊紙ULONG[count]锛夛紝鍚庢涓烘嫾鎺ユ暟鎹尯銆?
bool CADSComm::ADSReadSum(const char* const* symbols, const unsigned long* lengths, void* const* outputs, unsigned long count)
{
	if (!m_bOpen)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: Ads not Open\n");
		return false;
	}

	if (symbols == NULL || lengths == NULL || outputs == NULL || count == 0)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSReadSum invalid arguments\n");
		return false;
	}

	struct ST_SumReadReq
	{
		ULONG index_group;
		ULONG index_offset;
		ULONG read_length;
	};

	std::vector<ST_SumReadReq> requests(count);
	std::vector<ULONG> item_errors(count, 0);
	unsigned long total_data_bytes = 0;

	for (unsigned long i = 0; i < count; ++i)
	{
		if (symbols[i] == NULL || outputs[i] == NULL || lengths[i] == 0)
		{
			sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSReadSum invalid item at index %lu\n", i);
			return false;
		}

		const unsigned long handle = ADSGetAddr(symbols[i]);
		if (handle == 0)
		{
			return false;
		}

		requests[i].index_group = ADSIGRP_SYM_VALBYHND;
		requests[i].index_offset = handle;
		requests[i].read_length = lengths[i];

		if (total_data_bytes > (0xFFFFFFFFUL - lengths[i]))
		{
			sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSReadSum size overflow\n");
			return false;
		}
		total_data_bytes += lengths[i];
	}

	const unsigned long errors_bytes = count * sizeof(ULONG);
	if (errors_bytes > (0xFFFFFFFFUL - total_data_bytes))
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSReadSum response size overflow\n");
		return false;
	}
	const unsigned long response_bytes = errors_bytes + total_data_bytes;
	std::vector<unsigned char> response(response_bytes, 0);

	unsigned long cb_return = 0;
	const long nErr = AdsSyncReadWriteReqEx2(
		m_adsPort,
		m_PAmsAddr,
		ADSIGRP_SUMUP_READ,
		count,
		response_bytes,
		response.data(),
		static_cast<unsigned long>(requests.size() * sizeof(ST_SumReadReq)),
		requests.data(),
		&cb_return);
	if (nErr)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsSyncReadWriteReqEx2(SUMUP_READ): %ld\n", nErr);
		return false;
	}

	if (cb_return < errors_bytes)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSReadSum short response (%lu)\n", cb_return);
		return false;
	}

	memcpy(item_errors.data(), response.data(), errors_bytes);
	for (unsigned long i = 0; i < count; ++i)
	{
		if (item_errors[i] != 0)
		{
			sprintf_s(
				m_lastError,
				sizeof(m_lastError),
				"Error: ADSReadSum item %lu failed (code=%lu, symbol=%s)\n",
				i,
				static_cast<unsigned long>(item_errors[i]),
				symbols[i]);
			return false;
		}
	}

	unsigned long data_offset = errors_bytes;
	for (unsigned long i = 0; i < count; ++i)
	{
		const unsigned long next_offset = data_offset + lengths[i];
		if (next_offset > cb_return || next_offset > response_bytes)
		{
			sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSReadSum payload truncated at item %lu\n", i);
			return false;
		}

		memcpy(outputs[i], response.data() + data_offset, lengths[i]);
		data_offset = next_offset;
	}

	return true;
}

// 句柄获取流程：
// - 先查缓存；
// - 缓存未命中时通过 ADSIGRP_SYM_HNDBYNAME 向 PLC 申请；
// - 申请成功后写入缓存并返回。
unsigned long CADSComm::ADSGetAddr(const char* paraName)
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

// 外部路由连接（固定远端 NetId）：
// 连接到远端 PLC 运行端口 851。
bool CADSComm::OpenComm()
{
	m_bOpen = false;
	long nErr;
	USHORT nAdsState;

	// AmsNetId _AmsId = { 5, 100, 219, 36, 1, 1 };
	AmsNetId _AmsId = { 169, 254, 119, 135, 1, 1 };

	m_PAmsAddr->netId = _AmsId;
	m_PAmsAddr->port = 851;

	m_adsPort = AdsPortOpenEx();

	if (m_adsPort <= 0)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsPortOpenEx failed\n");
		return false;
	}

	USHORT nDeviceState;
	nErr = AdsSyncReadStateReqEx(m_adsPort, m_PAmsAddr, &nAdsState, &nDeviceState);
	if (nErr)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsSyncReadStateReqEx: %ld\n", nErr);
		AdsPortCloseEx(m_adsPort);
		m_adsPort = 0;
		return false;
	}

	// 当目标设备状态为 6 时，尝试切换到 RUN。
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

// 本地路由连接：
// 先取本机 AMS 地址，再访问本机 PLC 运行端口 851。
bool CADSComm::OpenComm_inside()
{
	m_bOpen = false;
	long nErr;
	USHORT nAdsState;

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

	USHORT nDeviceState;
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

// 关闭顺序：
// 1) 释放所有已缓存的 PLC 符号句柄；
// 2) 清空缓存；
// 3) 关闭 ADS 端口并复位本地状态。
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
