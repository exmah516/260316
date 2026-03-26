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
	bool bQuit = false;
	bool bForceEnable = false;
	bool bPause = false;
	int iID1 = -1;
	int iFnt = 2;
	USERData objData;
	double fVels2[2], fJoints2[2];
	unsigned char buttons2;
	long encoders2[2];


	bool init();//返回0说明初始化失败，返回1说明成功

	void close();//关闭设备

	void setforce(double F, double N);//设置力与力矩,正方向的力是向后的，正方向的力矩是逆时针的

	void showinfo();//显示状态
};