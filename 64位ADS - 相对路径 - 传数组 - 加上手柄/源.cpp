
#include <cmath>
#include <iostream>
#include "ÊÖ±ú.h"
#include <ADSComm1.h>
#include<cstring>


int main()
{

	CADSComm a;

	a.OpenComm();

	short force1 = 0;
	short force2 = 0;
	short force3 = 0;
	short force4 = 0;
	
	double pos[2] = { 1,2 };


	while (1)
	{
		a.ADSRead((char*)"G.Data_1", sizeof(force1), &force1);
		a.ADSRead((char*)"G.Data_2", sizeof(force2), &force2);
		a.ADSRead((char*)"G.Data_3", sizeof(force3), &force3);
		a.ADSRead((char*)"G.Data_4", sizeof(force4), &force4);



		std::cout << force1 << "\t" << force2<<"\t"<<force3 << "\t" << force4 << std::endl;

		//a.ADSWrite((char*)"MAIN.pos", sizeof(pos), &pos);
	}

}