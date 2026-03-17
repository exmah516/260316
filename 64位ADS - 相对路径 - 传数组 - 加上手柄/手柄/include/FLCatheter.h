#pragma once

#ifdef FLCATHETER_EXPORTS
#define FLCATHETER_API __declspec(dllexport)
#else
#define FLCATHETER_API __declspec(dllimport)
#endif

extern "C"
{
	FLCATHETER_API int openDevice(DWORD sn);
	FLCATHETER_API void PauseDevice(int id = 0);
	FLCATHETER_API void ResumeDevice(int id = 0);
	FLCATHETER_API void closeDevice();
	FLCATHETER_API int startServoLoop(int(_stdcall *func)(void *), void *lpParam);
	FLCATHETER_API int stopServoLoop();
	FLCATHETER_API bool isServoLoopRunning();
	FLCATHETER_API double getServoLoopRate(int id = 0);
	FLCATHETER_API void enableForces(bool en = true, int id = 0);
	FLCATHETER_API bool isForcesEnabled(int id = 0);
	FLCATHETER_API int getSerialNumber(int id = 0);
	FLCATHETER_API int deviceStatus(int id = 0);
	FLCATHETER_API void sendZeroForce(int id = 0);
	FLCATHETER_API void sendForce(double force[], double torque, int id = 0);
	FLCATHETER_API void zeroEncoders(int id = 0);
	FLCATHETER_API void getEncoders(long encs[], int id = 0);
	FLCATHETER_API void getEncVel(double evels[], int id = 0);
	FLCATHETER_API void getSwitch(byte &swts, int id = 0);
	FLCATHETER_API void getJoints(double joints[], int id = 0);
	FLCATHETER_API void getEndFrame(double EndFrame[], int id = 0);
	FLCATHETER_API void getPositionAndAngle(double pos[], double& Angle, int id = 0);
	FLCATHETER_API void getPose(double rot[], int id = 0);

}
