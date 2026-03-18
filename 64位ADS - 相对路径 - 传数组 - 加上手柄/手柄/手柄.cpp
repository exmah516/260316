#include "Handle.h"

LONG Handle::s_open_devices = 0;
bool Handle::s_servo_loop_started = false;

int __stdcall SyncUpdate(void* abc)
{
	return deviceStatus();
};

Handle::Handle(DWORD serial)
	: serial_number_(serial)
{
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
	return init(serial_number_);
}

bool Handle::init(DWORD serial)
{
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
	if (iID1 < 0)
	{
		return;
	}

	enableForces(false, iID1);
	iID1 = -1;

	if (s_open_devices > 0)
	{
		--s_open_devices;
	}

	if ((s_open_devices == 0) && s_servo_loop_started)
	{
		stopServoLoop();
		closeDevice();
		s_servo_loop_started = false;
	}
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

void Handle::showinfo(const char* label)
{
	if (!poll())
	{
		return;
	}

	if (label != nullptr)
	{
		printf("%s ", label);
	}

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
