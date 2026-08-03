#pragma once

#include <cstdint>
#include <stdio.h>
#include <string.h>
#include <memory>
#include <iostream>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
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
	// 仅表示“不请求切换 PLC RUN 状态”，并不禁止后续显式 ADSWrite。
	bool OpenCommReadOnly();
	bool OpenCommInsideReadOnly();

	bool CloseComm();
	bool ADSWrite(const char * paraName, unsigned long length, void * data);
	bool ADSRead(const char * paraName, unsigned long length, void * data);
	bool ADSReadSum(const char* const* symbols, const unsigned long* lengths, void* const* outputs, unsigned long count);
	bool ADSReadSumByHandle(const unsigned long* handles, const unsigned long* lengths, void* const* outputs, unsigned long count);
	bool ADSWriteSum(const char* const* symbols, const unsigned long* lengths, const void* const* inputs, unsigned long count);
	bool ADSWriteSumByHandle(const unsigned long* handles, const unsigned long* lengths, const void* const* inputs, unsigned long count);
	bool ADSWrite(unsigned long addr, unsigned long length, void * data);
	bool ADSRead(unsigned long addr, unsigned long length, void * data);
	unsigned long ADSGetAddr(const char * paraName);

	// hUser 是明确的 32 位注册 ID，不能用来存放 x64 指针。
	bool ADSAddNotification(
		const char* paraName,
		unsigned long length,
		PAdsNotificationFuncEx callback,
		std::uint32_t registrationId,
		unsigned long* notificationHandle,
		ADSTRANSMODE mode = ADSTRANS_SERVERONCHA,
		unsigned long cycleTime100ns = 100000UL,
		unsigned long maxDelay100ns = 0UL);
	bool ADSDeleteNotification(unsigned long notificationHandle);
	bool SetTimeout(unsigned long timeoutMs);
	bool ReadDeviceState(unsigned short& adsState, unsigned short& deviceState);

	bool IsCommOpen() const;
	std::string GetLastErrorCopy() const;
	const char* GetLastError() const;

private:
	struct SumRequest
	{
		unsigned long indexGroup;
		unsigned long indexOffset;
		unsigned long length;
	};

	bool ValidateOpenLocked(const char* operation);
	bool OpenCommRemoteMode();
	bool OpenCommLocalMode();
	void ClearLastErrorLocked();

	PAmsAddr		m_PAmsAddr;
	long			m_adsPort;
	std::unordered_map<std::string, unsigned long> m_symbolHandles;
	std::vector<unsigned long> m_notificationHandles;
	std::vector<unsigned long> m_sumHandles;
	std::vector<SumRequest> m_sumRequests;
	std::vector<unsigned char> m_sumWriteBuffer;
	std::vector<unsigned char> m_sumResponseBuffer;
	mutable std::recursive_mutex m_mutex;
	bool			m_bOpen;
	unsigned long	m_timeoutMs;
	char 			m_lastError[256];
};
