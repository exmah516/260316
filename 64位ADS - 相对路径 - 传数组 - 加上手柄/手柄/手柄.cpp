#include "Handle.h"

int __stdcall SyncUpdate(void* abc)
{
	return deviceStatus();
};

bool Handle::init()
{
	iID1 = openDevice((DWORD)582);

	if (iID1 < 0)
	{
		printf("Open device error! %d\n", 582);
		return false;
	}
	else
		printf(" You have opened a device! No: %d.\n\n", getSerialNumber(iID1));

	//开始循环读取设备状态
	startServoLoop(SyncUpdate, NULL);

	enableForces(true, iID1);

	return true;
};

void Handle::close()
{
	enableForces(false, iID1);
	stopServoLoop();//停止服务循环
	closeDevice();//关闭设备
}

void Handle::setforce(double F, double N)
{
	setforce_axis(F, 0, N);
}

void Handle::setforce_axis(double F, int axis, double N)
{
	double fForce[3] = { 0, 0, 0 };
	if (axis < 0) axis = 0;
	if (axis > 2) axis = 2;
	fForce[axis] = F;
	sendForce(fForce, N, iID1);
}

void Handle::showinfo()
{
	getEncVel(fVels2, iID1);
	getJoints(fJoints2, iID1);
	getEncoders(encoders2, iID1);
	getSwitch(buttons2, iID1);

	printf("Rate:%4dHz\t Btns:%02X\t Encoders:%9d %9d\t V:%9.3f %9.3f\t J:%3.3f %3.3f\r",
		(int)getServoLoopRate(), buttons2, encoders2[0], encoders2[1], fVels2[0], fVels2[1],fJoints2[0] * 1000.0, fJoints2[1] * Rad);
}
