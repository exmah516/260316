#include "Handle.h"

// 设备级全局状态：
// - s_open_devices: 当前已打开的设备数量（用于决定何时真正停止伺服循环）
// - s_servo_loop_started: 伺服循环是否已经启动（避免重复启动）
LONG Handle::s_open_devices = 0;
bool Handle::s_servo_loop_started = false;

int __stdcall SyncUpdate(void* abc)
{
	// SDK 伺服循环回调，返回设备状态码。
	return deviceStatus();
};

Handle::Handle(DWORD serial)
	: serial_number_(serial)
{
	// 构造期只做本地缓存清零，不触发硬件连接。
	objData.buttons2 = 0;
	objData.encoders2[0] = 0;
	objData.encoders2[1] = 0;
	fVels2[0] = 0.0;
	fVels2[1] = 0.0;
	fJoints2[0] = 0.0;
	fJoints2[1] = 0.0;
	buttons2 = 0;
	encoders2[0] = 0;
	encoders2[1] = 0;
}

bool Handle::init()
{
	// 使用构造时给定的序列号进行初始化。
	return init(serial_number_);
}

bool Handle::init(DWORD serial)
{
	// 幂等保护：已打开时直接返回成功。
	serial_number_ = serial;
	if (iID1 >= 0)
	{
		return true;
	}

	iID1 = openDevice(serial_number_);

	if (iID1 < 0)
	{
		printf("Open device error! %lu\n", static_cast<unsigned long>(serial_number_));
		return false;
	}
	else
		printf(" You have opened a device! SN: %d (request %lu).\n\n",
			getSerialNumber(iID1),
			static_cast<unsigned long>(serial_number_));

	// 伺服循环在首个设备打开时启动一次，后续设备复用同一循环。
	if (!s_servo_loop_started)
	{
		startServoLoop(SyncUpdate, NULL);
		s_servo_loop_started = true;
	}

	enableForces(true, iID1);
	++s_open_devices;

	return true;
}

bool Handle::poll()
{
	// 拉取当前手柄状态快照：速度、关节、编码器、按键。
	if (iID1 < 0)
	{
		return false;
	}

	getEncVel(fVels2, iID1);
	getJoints(fJoints2, iID1);
	getEncoders(encoders2, iID1);
	getSwitch(buttons2, iID1);

	objData.buttons2 = buttons2;
	objData.encoders2[0] = encoders2[0];
	objData.encoders2[1] = encoders2[1];

	return true;
}

void Handle::close()
{
	// 幂等保护：重复关闭直接返回。
	if (iID1 < 0)
	{
		return;
	}

	// 先关闭当前设备力输出，再注销设备 ID。
	enableForces(false, iID1);
	iID1 = -1;

	if (s_open_devices > 0)
	{
		--s_open_devices;
	}

	if ((s_open_devices == 0) && s_servo_loop_started)
	{
		// 最后一个设备关闭时，统一停止伺服循环并关闭设备层资源。
		stopServoLoop();
		closeDevice();
		s_servo_loop_started = false;
	}
}

void Handle::setforce(double F, double N)
{
	// 默认将线力施加在 axis=0。
	setforce_axis(F, 0, N);
}

void Handle::setforce_axis(double F, int axis, double N)
{
	// SDK 需要三轴力向量 + 力矩；这里只开启指定轴，其余轴为 0。
	double fForce[3] = { 0, 0, 0 };
	if (axis < 0) axis = 0;
	if (axis > 2) axis = 2;
	fForce[axis] = F;
	sendForce(fForce, N, iID1);
}

void Handle::showinfo(const char* label)
{
	// showinfo 仅用于调试展示，不参与控制闭环。
	if (!poll())
	{
		return;
	}

	if (label != nullptr)
	{
		printf("%s ", label);
	}

	// 输出字段说明：
	// SN      : 设备序列号
	// Rate    : 伺服循环频率
	// Btns    : buttons2 位掩码
	// Encoders: 两路编码器原始值
	// V       : 两路编码器速度
	// J       : 两路关节值（axis0 以 mm 显示，axis1 以 deg 显示）
	printf("SN:%d\t Rate:%4dHz\t Btns:%02X\t Encoders:%9d %9d\t V:%9.3f %9.3f\t J:%3.3f %3.3f\r",
		getSerialNumber(iID1),
		(int)getServoLoopRate(),
		buttons2,
		encoders2[0],
		encoders2[1],
		fVels2[0],
		fVels2[1],
		fJoints2[0] * 1000.0,
		fJoints2[1] * Rad);
}
