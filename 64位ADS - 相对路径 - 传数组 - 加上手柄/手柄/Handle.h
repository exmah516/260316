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

	bool init();//����0˵����ʼ��ʧ�ܣ�����1˵���ɹ�

	void close();//�ر��豸

	void setforce(double F, double N);//������������,��������������ģ����������������ʱ���

	void setforce_axis(double F, int axis, double N);

	void showinfo();//��ʾ״̬
};
