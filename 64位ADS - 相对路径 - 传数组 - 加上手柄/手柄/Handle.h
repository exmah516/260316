#pragma once
#include <SDKDDKVer.h>
#include <stdio.h>
#include <tchar.h>
#include <conio.h>
#include <windows.h>
#include <FLCatheter.h>
#define Rad  57.29577951308232

struct USERData
{
	unsigned char buttons2;
	long encoders2[2];
};

class Handle
{
public:
	explicit Handle(DWORD serial = 582);
	bool bQuit = false;
	bool bForceEnable = false;
	bool bPause = false;
	int iID1 = -1;
	int iFnt = 2;
	USERData objData;
	double fVels2[2], fJoints2[2];
	unsigned char buttons2;
	long encoders2[2];

	bool init();
	bool init(DWORD serial);
	bool poll();

	void clear_cache();
	void close();

	void setforce(double F, double N);

	void setforce_axis(double F, int axis, double N);

	void showinfo(const char* label = nullptr);

	// 远程模式：手柄实体在主端，采样经远程网关写入 remote_handle_bridge，本类不再访问 SDK。
	// 必须在任何 Handle 对象 init() 之前设置。
	static void use_remote(bool on) { s_remote = on; }
	static bool remote_mode() { return s_remote; }

	DWORD serial() const
	{
		return serial_number_;
	}

	bool is_open() const
	{
		return iID1 >= 0;
	}

private:
	DWORD serial_number_;
	static LONG s_open_devices;
	static bool s_servo_loop_started;
	static bool s_remote;
};
