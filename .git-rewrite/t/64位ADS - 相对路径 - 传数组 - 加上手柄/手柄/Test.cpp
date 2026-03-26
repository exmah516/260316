#include "Handle.h"
#include <cmath>
#include <iostream>
#include <windows.h>
int main()
{
	Handle handle;
	
	if (!handle.init())return 0;

	double i = 0.0;
	while (1)
	{
		handle.showinfo();
		Sleep(20);
		 
		//i += 0.001;
		//handle.setforce(0, 0.001*sin(i));
	}

	handle.close();
}
