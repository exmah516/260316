
#include <cmath>
#include <iostream>
#include "ÊÖ±ú.h"
#include <ADSComm1.h>
#include<cstring>


int main()
{
	Handle handle;

	short force3 = 0;
	short force4 = 0;

	short force3_f = 0;
	short force4_f = 0;
	double pos[7] = { 0 };
	double pos_init[2] = { 0 };

	if (!handle.init())return 0;
	 

	while (1)
	{
		handle.showinfo();

		handle.setforce(0, -1);
	}

	handle.close();
}