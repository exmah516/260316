#include <ADSComm1.h>

#include <algorithm>
#include <limits>
#include <new>

namespace
{
	constexpr unsigned long kDefaultAdsTimeoutMs = 20UL;
	constexpr size_t kInitialSumItemCapacity = 32U;
	constexpr size_t kInitialSumByteCapacity = 4096U;
}

// ADS 通讯封装说明：
// - 所有公开操作使用同一把递归锁，允许按符号名接口复用按句柄接口。
// - Sum Read/Write 复用对象级缓冲区，容量稳定后不再每拍分配内存。
// - 通知回调只应复制数据并唤醒上层线程，不应在回调中再调用本对象。

CADSComm::CADSComm(void)
	: m_PAmsAddr(new AmsAddr{}),
	  m_adsPort(0),
	  m_bOpen(false),
	  m_timeoutMs(kDefaultAdsTimeoutMs)
{
	static_assert(sizeof(SumRequest) == 3U * sizeof(unsigned long), "Sum request layout must contain three ULONG values");
	static_assert(sizeof(std::uint32_t) == sizeof(unsigned long), "ADS hUser must be 32 bit on Windows");

	memset(m_lastError, 0, sizeof(m_lastError));
	m_notificationHandles.reserve(16U);
	m_sumHandles.reserve(kInitialSumItemCapacity);
	m_sumRequests.reserve(kInitialSumItemCapacity);
	m_sumWriteBuffer.reserve(kInitialSumByteCapacity);
	m_sumResponseBuffer.reserve(kInitialSumByteCapacity);
}

CADSComm::~CADSComm(void)
{
	CloseComm();
	delete m_PAmsAddr;
	m_PAmsAddr = nullptr;
}

bool CADSComm::ValidateOpenLocked(const char* operation)
{
	if (m_bOpen && m_adsPort > 0 && m_PAmsAddr != nullptr)
	{
		return true;
	}

	sprintf_s(
		m_lastError,
		sizeof(m_lastError),
		"Error: %s: Ads not Open\n",
		operation != nullptr ? operation : "ADS operation");
	return false;
}

void CADSComm::ClearLastErrorLocked()
{
	memset(m_lastError, 0, sizeof(m_lastError));
}

bool CADSComm::IsCommOpen() const
{
	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	return m_bOpen && m_adsPort > 0;
}

std::string CADSComm::GetLastErrorCopy() const
{
	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	return std::string(m_lastError);
}

const char* CADSComm::GetLastError() const
{
	// 保留旧接口，但返回每线程副本，避免解锁后暴露可变的内部缓冲区。
	static thread_local char errorCopy[sizeof(m_lastError)]{};
	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	strcpy_s(errorCopy, sizeof(errorCopy), m_lastError);
	return errorCopy;
}

// 按符号名写：先解析缓存句柄，再复用按句柄写接口。
bool CADSComm::ADSWrite(const char* paraName, unsigned long length, void* data)
{
	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	const unsigned long addr = ADSGetAddr(paraName);
	if (addr == 0)
	{
		return false;
	}
	return ADSWrite(addr, length, data);
}

// 按符号名读：先解析缓存句柄，再复用按句柄读接口。
bool CADSComm::ADSRead(const char* paraName, unsigned long length, void* data)
{
	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	const unsigned long addr = ADSGetAddr(paraName);
	if (addr == 0)
	{
		return false;
	}
	return ADSRead(addr, length, data);
}

bool CADSComm::ADSWrite(unsigned long addr, unsigned long length, void* data)
{
	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	if (!ValidateOpenLocked("ADSWrite"))
	{
		return false;
	}
	if (addr == 0 || length == 0 || data == nullptr)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSWrite invalid arguments\n");
		return false;
	}

	const long nErr = AdsSyncWriteReqEx(m_adsPort, m_PAmsAddr, ADSIGRP_SYM_VALBYHND, addr, length, data);
	if (nErr != 0)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsSyncWriteReqEx: %ld\n", nErr);
		return false;
	}
	ClearLastErrorLocked();
	return true;
}

bool CADSComm::ADSRead(unsigned long addr, unsigned long length, void* data)
{
	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	if (!ValidateOpenLocked("ADSRead"))
	{
		return false;
	}
	if (addr == 0 || length == 0 || data == nullptr)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSRead invalid arguments\n");
		return false;
	}

	unsigned long cbReturn = 0;
	const long nErr = AdsSyncReadReqEx2(
		m_adsPort,
		m_PAmsAddr,
		ADSIGRP_SYM_VALBYHND,
		addr,
		length,
		data,
		&cbReturn);
	if (nErr != 0)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsSyncReadReqEx2: %ld\n", nErr);
		return false;
	}
	if (cbReturn != length)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSRead short response (%lu/%lu)\n", cbReturn, length);
		return false;
	}

	ClearLastErrorLocked();
	return true;
}

// 符号名版仅负责将符号解析为句柄；100 Hz 路径应在启动时解析后直接调用 ByHandle 版。
bool CADSComm::ADSReadSum(
	const char* const* symbols,
	const unsigned long* lengths,
	void* const* outputs,
	unsigned long count)
{
	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	if (!ValidateOpenLocked("ADSReadSum"))
	{
		return false;
	}
	if (symbols == nullptr || lengths == nullptr || outputs == nullptr || count == 0)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSReadSum invalid arguments\n");
		return false;
	}

	try
	{
		m_sumHandles.resize(count);
	}
	catch (const std::bad_alloc&)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSReadSum buffer allocation failed\n");
		return false;
	}

	for (unsigned long i = 0; i < count; ++i)
	{
		if (symbols[i] == nullptr || outputs[i] == nullptr || lengths[i] == 0)
		{
			sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSReadSum invalid item at index %lu\n", i);
			return false;
		}
		m_sumHandles[i] = ADSGetAddr(symbols[i]);
		if (m_sumHandles[i] == 0)
		{
			return false;
		}
	}

	return ADSReadSumByHandle(m_sumHandles.data(), lengths, outputs, count);
}

bool CADSComm::ADSReadSumByHandle(
	const unsigned long* handles,
	const unsigned long* lengths,
	void* const* outputs,
	unsigned long count)
{
	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	if (!ValidateOpenLocked("ADSReadSumByHandle"))
	{
		return false;
	}
	if (handles == nullptr || lengths == nullptr || outputs == nullptr || count == 0)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSReadSumByHandle invalid arguments\n");
		return false;
	}

	const unsigned long maxValue = (std::numeric_limits<unsigned long>::max)();
	if (count > maxValue / static_cast<unsigned long>(sizeof(SumRequest)) ||
		count > maxValue / static_cast<unsigned long>(sizeof(unsigned long)))
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSReadSumByHandle item count overflow\n");
		return false;
	}

	unsigned long totalDataBytes = 0;
	for (unsigned long i = 0; i < count; ++i)
	{
		if (handles[i] == 0 || outputs[i] == nullptr || lengths[i] == 0)
		{
			sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSReadSumByHandle invalid item at index %lu\n", i);
			return false;
		}
		if (totalDataBytes > maxValue - lengths[i])
		{
			sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSReadSumByHandle payload size overflow\n");
			return false;
		}
		totalDataBytes += lengths[i];
	}

	const unsigned long errorsBytes = count * static_cast<unsigned long>(sizeof(unsigned long));
	if (errorsBytes > maxValue - totalDataBytes)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSReadSumByHandle response size overflow\n");
		return false;
	}
	const unsigned long responseBytes = errorsBytes + totalDataBytes;

	try
	{
		m_sumRequests.resize(count);
		m_sumResponseBuffer.resize(responseBytes);
	}
	catch (const std::bad_alloc&)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSReadSumByHandle buffer allocation failed\n");
		return false;
	}

	for (unsigned long i = 0; i < count; ++i)
	{
		m_sumRequests[i].indexGroup = ADSIGRP_SYM_VALBYHND;
		m_sumRequests[i].indexOffset = handles[i];
		m_sumRequests[i].length = lengths[i];
	}

	unsigned long cbReturn = 0;
	const long nErr = AdsSyncReadWriteReqEx2(
		m_adsPort,
		m_PAmsAddr,
		ADSIGRP_SUMUP_READ,
		count,
		responseBytes,
		m_sumResponseBuffer.data(),
		count * static_cast<unsigned long>(sizeof(SumRequest)),
		m_sumRequests.data(),
		&cbReturn);
	if (nErr != 0)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsSyncReadWriteReqEx2(SUMUP_READ): %ld\n", nErr);
		return false;
	}
	if (cbReturn < responseBytes)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSReadSumByHandle short response (%lu/%lu)\n", cbReturn, responseBytes);
		return false;
	}

	for (unsigned long i = 0; i < count; ++i)
	{
		unsigned long itemError = 0;
		memcpy(&itemError, m_sumResponseBuffer.data() + i * sizeof(unsigned long), sizeof(itemError));
		if (itemError != 0)
		{
			sprintf_s(
				m_lastError,
				sizeof(m_lastError),
				"Error: ADSReadSumByHandle item %lu failed (code=%lu, handle=%lu)\n",
				i,
				itemError,
				handles[i]);
			return false;
		}
	}

	unsigned long dataOffset = errorsBytes;
	for (unsigned long i = 0; i < count; ++i)
	{
		memcpy(outputs[i], m_sumResponseBuffer.data() + dataOffset, lengths[i]);
		dataOffset += lengths[i];
	}

	ClearLastErrorLocked();
	return true;
}

bool CADSComm::ADSWriteSum(
	const char* const* symbols,
	const unsigned long* lengths,
	const void* const* inputs,
	unsigned long count)
{
	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	if (!ValidateOpenLocked("ADSWriteSum"))
	{
		return false;
	}
	if (symbols == nullptr || lengths == nullptr || inputs == nullptr || count == 0)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSWriteSum invalid arguments\n");
		return false;
	}

	try
	{
		m_sumHandles.resize(count);
	}
	catch (const std::bad_alloc&)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSWriteSum buffer allocation failed\n");
		return false;
	}

	for (unsigned long i = 0; i < count; ++i)
	{
		if (symbols[i] == nullptr || inputs[i] == nullptr || lengths[i] == 0)
		{
			sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSWriteSum invalid item at index %lu\n", i);
			return false;
		}
		m_sumHandles[i] = ADSGetAddr(symbols[i]);
		if (m_sumHandles[i] == 0)
		{
			return false;
		}
	}

	return ADSWriteSumByHandle(m_sumHandles.data(), lengths, inputs, count);
}

bool CADSComm::ADSWriteSumByHandle(
	const unsigned long* handles,
	const unsigned long* lengths,
	const void* const* inputs,
	unsigned long count)
{
	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	if (!ValidateOpenLocked("ADSWriteSumByHandle"))
	{
		return false;
	}
	if (handles == nullptr || lengths == nullptr || inputs == nullptr || count == 0)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSWriteSumByHandle invalid arguments\n");
		return false;
	}

	const unsigned long maxValue = (std::numeric_limits<unsigned long>::max)();
	if (count > maxValue / static_cast<unsigned long>(sizeof(SumRequest)) ||
		count > maxValue / static_cast<unsigned long>(sizeof(unsigned long)))
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSWriteSumByHandle item count overflow\n");
		return false;
	}

	unsigned long totalDataBytes = 0;
	for (unsigned long i = 0; i < count; ++i)
	{
		if (handles[i] == 0 || inputs[i] == nullptr || lengths[i] == 0)
		{
			sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSWriteSumByHandle invalid item at index %lu\n", i);
			return false;
		}
		if (totalDataBytes > maxValue - lengths[i])
		{
			sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSWriteSumByHandle payload size overflow\n");
			return false;
		}
		totalDataBytes += lengths[i];
	}

	const unsigned long requestBytes = count * static_cast<unsigned long>(sizeof(SumRequest));
	if (requestBytes > maxValue - totalDataBytes)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSWriteSumByHandle request size overflow\n");
		return false;
	}
	const unsigned long writeBytes = requestBytes + totalDataBytes;
	const unsigned long responseBytes = count * static_cast<unsigned long>(sizeof(unsigned long));

	try
	{
		m_sumRequests.resize(count);
		m_sumWriteBuffer.resize(writeBytes);
		m_sumResponseBuffer.resize(responseBytes);
	}
	catch (const std::bad_alloc&)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSWriteSumByHandle buffer allocation failed\n");
		return false;
	}

	for (unsigned long i = 0; i < count; ++i)
	{
		m_sumRequests[i].indexGroup = ADSIGRP_SYM_VALBYHND;
		m_sumRequests[i].indexOffset = handles[i];
		m_sumRequests[i].length = lengths[i];
	}
	memcpy(m_sumWriteBuffer.data(), m_sumRequests.data(), requestBytes);

	unsigned long dataOffset = requestBytes;
	for (unsigned long i = 0; i < count; ++i)
	{
		memcpy(m_sumWriteBuffer.data() + dataOffset, inputs[i], lengths[i]);
		dataOffset += lengths[i];
	}

	unsigned long cbReturn = 0;
	const long nErr = AdsSyncReadWriteReqEx2(
		m_adsPort,
		m_PAmsAddr,
		ADSIGRP_SUMUP_WRITE,
		count,
		responseBytes,
		m_sumResponseBuffer.data(),
		writeBytes,
		m_sumWriteBuffer.data(),
		&cbReturn);
	if (nErr != 0)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsSyncReadWriteReqEx2(SUMUP_WRITE): %ld\n", nErr);
		return false;
	}
	if (cbReturn < responseBytes)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSWriteSumByHandle short response (%lu/%lu)\n", cbReturn, responseBytes);
		return false;
	}

	for (unsigned long i = 0; i < count; ++i)
	{
		unsigned long itemError = 0;
		memcpy(&itemError, m_sumResponseBuffer.data() + i * sizeof(unsigned long), sizeof(itemError));
		if (itemError != 0)
		{
			sprintf_s(
				m_lastError,
				sizeof(m_lastError),
				"Error: ADSWriteSumByHandle item %lu failed (code=%lu, handle=%lu)\n",
				i,
				itemError,
				handles[i]);
			return false;
		}
	}

	ClearLastErrorLocked();
	return true;
}

// 句柄只在第一次访问符号时向 PLC 申请，后续从本地缓存取得。
unsigned long CADSComm::ADSGetAddr(const char* paraName)
{
	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	if (!ValidateOpenLocked("ADSGetAddr"))
	{
		return 0;
	}
	if (paraName == nullptr || paraName[0] == '\0')
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: Empty symbol name\n");
		return 0;
	}

	const std::string key(paraName);
	const auto existing = m_symbolHandles.find(key);
	if (existing != m_symbolHandles.end())
	{
		ClearLastErrorLocked();
		return existing->second;
	}

	unsigned long handle = 0;
	unsigned long cbReturn = 0;
	const unsigned long nameLength = static_cast<unsigned long>(key.size() + 1U);
	const long nErr = AdsSyncReadWriteReqEx2(
		m_adsPort,
		m_PAmsAddr,
		ADSIGRP_SYM_HNDBYNAME,
		0,
		sizeof(handle),
		&handle,
		nameLength,
		const_cast<char*>(key.c_str()),
		&cbReturn);
	if (nErr != 0)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsSyncReadWriteReqEx2(HNDBYNAME): %ld\n", nErr);
		return 0;
	}
	if (cbReturn != sizeof(handle) || handle == 0)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSGetAddr invalid handle response for %s\n", paraName);
		return 0;
	}

	try
	{
		m_symbolHandles.emplace(key, handle);
	}
	catch (const std::bad_alloc&)
	{
		AdsSyncWriteReqEx(m_adsPort, m_PAmsAddr, ADSIGRP_SYM_RELEASEHND, 0, sizeof(handle), &handle);
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSGetAddr handle cache allocation failed\n");
		return 0;
	}

	ClearLastErrorLocked();
	return handle;
}

bool CADSComm::ADSAddNotification(
	const char* paraName,
	unsigned long length,
	PAdsNotificationFuncEx callback,
	std::uint32_t registrationId,
	unsigned long* notificationHandle,
	ADSTRANSMODE mode,
	unsigned long cycleTime100ns,
	unsigned long maxDelay100ns)
{
	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	if (notificationHandle != nullptr)
	{
		*notificationHandle = 0;
	}
	if (!ValidateOpenLocked("ADSAddNotification"))
	{
		return false;
	}
	if (paraName == nullptr || paraName[0] == '\0' || length == 0 || callback == nullptr || notificationHandle == nullptr)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSAddNotification invalid arguments\n");
		return false;
	}

	const unsigned long symbolHandle = ADSGetAddr(paraName);
	if (symbolHandle == 0)
	{
		return false;
	}

	AdsNotificationAttrib attributes{};
	attributes.cbLength = length;
	attributes.nTransMode = mode;
	attributes.nMaxDelay = maxDelay100ns;
	attributes.nCycleTime = cycleTime100ns;

	unsigned long handle = 0;
	const long nErr = AdsSyncAddDeviceNotificationReqEx(
		m_adsPort,
		m_PAmsAddr,
		ADSIGRP_SYM_VALBYHND,
		symbolHandle,
		&attributes,
		callback,
		static_cast<unsigned long>(registrationId),
		&handle);
	if (nErr != 0)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsSyncAddDeviceNotificationReqEx: %ld\n", nErr);
		return false;
	}

	try
	{
		m_notificationHandles.push_back(handle);
	}
	catch (const std::bad_alloc&)
	{
		AdsSyncDelDeviceNotificationReqEx(m_adsPort, m_PAmsAddr, handle);
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSAddNotification handle tracking allocation failed\n");
		return false;
	}

	*notificationHandle = handle;
	ClearLastErrorLocked();
	return true;
}

bool CADSComm::ADSDeleteNotification(unsigned long notificationHandle)
{
	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	if (!ValidateOpenLocked("ADSDeleteNotification"))
	{
		return false;
	}

	const auto it = std::find(m_notificationHandles.begin(), m_notificationHandles.end(), notificationHandle);
	if (notificationHandle == 0 || it == m_notificationHandles.end())
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: ADSDeleteNotification unknown handle %lu\n", notificationHandle);
		return false;
	}

	const long nErr = AdsSyncDelDeviceNotificationReqEx(m_adsPort, m_PAmsAddr, notificationHandle);
	if (nErr != 0)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsSyncDelDeviceNotificationReqEx: %ld\n", nErr);
		return false;
	}

	m_notificationHandles.erase(it);
	ClearLastErrorLocked();
	return true;
}

bool CADSComm::SetTimeout(unsigned long timeoutMs)
{
	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	if (timeoutMs == 0 || timeoutMs > static_cast<unsigned long>((std::numeric_limits<long>::max)()))
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: SetTimeout invalid timeout %lu ms\n", timeoutMs);
		return false;
	}

	if (m_adsPort > 0)
	{
		const long nErr = AdsSyncSetTimeoutEx(m_adsPort, static_cast<long>(timeoutMs));
		if (nErr != 0)
		{
			sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsSyncSetTimeoutEx: %ld\n", nErr);
			return false;
		}
	}

	m_timeoutMs = timeoutMs;
	ClearLastErrorLocked();
	return true;
}

bool CADSComm::ReadDeviceState(unsigned short& adsState, unsigned short& deviceState)
{
	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	if (!ValidateOpenLocked("ReadDeviceState"))
	{
		return false;
	}

	const long nErr = AdsSyncReadStateReqEx(m_adsPort, m_PAmsAddr, &adsState, &deviceState);
	if (nErr != 0)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsSyncReadStateReqEx: %ld\n", nErr);
		return false;
	}

	ClearLastErrorLocked();
	return true;
}

// 兼容旧调用名：仅建立远端连接，不替用户切换 PLC 状态。
bool CADSComm::OpenComm()
{
	return OpenCommRemoteMode();
}

// 兼容命名：仅表示不请求切换 PLC RUN 状态，并不禁止后续显式写入。
bool CADSComm::OpenCommReadOnly()
{
	return OpenCommRemoteMode();
}

bool CADSComm::OpenCommRemoteMode()
{
	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	if (m_adsPort > 0 || m_bOpen)
	{
		CloseComm();
	}
	ClearLastErrorLocked();

	const AmsNetId remoteId = { 169, 254, 119, 135, 1, 1 };
	m_PAmsAddr->netId = remoteId;
	m_PAmsAddr->port = 851;

	m_adsPort = AdsPortOpenEx();
	if (m_adsPort <= 0)
	{
		m_adsPort = 0;
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsPortOpenEx failed\n");
		return false;
	}

	long nErr = AdsSyncSetTimeoutEx(m_adsPort, static_cast<long>(m_timeoutMs));
	if (nErr != 0)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsSyncSetTimeoutEx: %ld\n", nErr);
		AdsPortCloseEx(m_adsPort);
		m_adsPort = 0;
		return false;
	}

	unsigned short adsState = 0;
	unsigned short deviceState = 0;
	nErr = AdsSyncReadStateReqEx(m_adsPort, m_PAmsAddr, &adsState, &deviceState);
	if (nErr != 0)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsSyncReadStateReqEx: %ld\n", nErr);
		AdsPortCloseEx(m_adsPort);
		m_adsPort = 0;
		return false;
	}
	m_bOpen = true;
	ClearLastErrorLocked();
	return true;
}

// 兼容旧调用名：仅建立本地连接，不替用户切换 PLC 状态。
bool CADSComm::OpenComm_inside()
{
	return OpenCommLocalMode();
}

// 兼容命名：仅表示不请求切换 PLC RUN 状态，并不禁止后续显式写入。
bool CADSComm::OpenCommInsideReadOnly()
{
	return OpenCommLocalMode();
}

bool CADSComm::OpenCommLocalMode()
{
	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	if (m_adsPort > 0 || m_bOpen)
	{
		CloseComm();
	}
	ClearLastErrorLocked();

	m_adsPort = AdsPortOpenEx();
	if (m_adsPort <= 0)
	{
		m_adsPort = 0;
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsPortOpenEx failed\n");
		return false;
	}

	long nErr = AdsSyncSetTimeoutEx(m_adsPort, static_cast<long>(m_timeoutMs));
	if (nErr != 0)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsSyncSetTimeoutEx: %ld\n", nErr);
		AdsPortCloseEx(m_adsPort);
		m_adsPort = 0;
		return false;
	}

	nErr = AdsGetLocalAddressEx(m_adsPort, m_PAmsAddr);
	if (nErr != 0)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsGetLocalAddressEx: %ld\n", nErr);
		AdsPortCloseEx(m_adsPort);
		m_adsPort = 0;
		return false;
	}
	m_PAmsAddr->port = 851;

	unsigned short adsState = 0;
	unsigned short deviceState = 0;
	nErr = AdsSyncReadStateReqEx(m_adsPort, m_PAmsAddr, &adsState, &deviceState);
	if (nErr != 0)
	{
		sprintf_s(m_lastError, sizeof(m_lastError), "Error: AdsSyncReadStateReqEx: %ld\n", nErr);
		AdsPortCloseEx(m_adsPort);
		m_adsPort = 0;
		return false;
	}
	m_bOpen = true;
	ClearLastErrorLocked();
	return true;
}

// 关闭顺序：先注销通知，再释放符号句柄，最后关闭 ADS 端口。
bool CADSComm::CloseComm()
{
	std::lock_guard<std::recursive_mutex> lock(m_mutex);
	if (!m_bOpen && m_adsPort == 0)
	{
		m_notificationHandles.clear();
		m_symbolHandles.clear();
		ClearLastErrorLocked();
		return true;
	}

	bool success = true;
	char firstError[sizeof(m_lastError)]{};
	auto rememberError = [&success, &firstError](const char* operation, long errorCode)
	{
		if (success)
		{
			sprintf_s(firstError, sizeof(firstError), "Error: %s: %ld\n", operation, errorCode);
		}
		success = false;
	};

	if (m_adsPort > 0 && m_PAmsAddr != nullptr)
	{
		for (const unsigned long notificationHandle : m_notificationHandles)
		{
			const long nErr = AdsSyncDelDeviceNotificationReqEx(m_adsPort, m_PAmsAddr, notificationHandle);
			if (nErr != 0)
			{
				rememberError("AdsSyncDelDeviceNotificationReqEx", nErr);
			}
		}
	}
	m_notificationHandles.clear();

	if (m_adsPort > 0 && m_PAmsAddr != nullptr)
	{
		for (const auto& entry : m_symbolHandles)
		{
			unsigned long handle = entry.second;
			const long nErr = AdsSyncWriteReqEx(
				m_adsPort,
				m_PAmsAddr,
				ADSIGRP_SYM_RELEASEHND,
				0,
				sizeof(handle),
				&handle);
			if (nErr != 0)
			{
				rememberError("AdsSyncWriteReqEx(RELEASEHND)", nErr);
			}
		}
	}
	m_symbolHandles.clear();

	if (m_adsPort > 0)
	{
		const long nErr = AdsPortCloseEx(m_adsPort);
		if (nErr != 0)
		{
			rememberError("AdsPortCloseEx", nErr);
		}
		m_adsPort = 0;
	}
	m_bOpen = false;

	if (success)
	{
		ClearLastErrorLocked();
	}
	else
	{
		strcpy_s(m_lastError, sizeof(m_lastError), firstError);
	}
	return success;
}
