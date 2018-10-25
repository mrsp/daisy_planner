

#include <stdlib.h>
#include <math.h>

#include <stdio.h>
 
#include <unistd.h>
#include <fstream>
#include <strings.h>
#include <string>
#include <sstream>


#include "DP.h"
#include "CoutColors.h"

DP gDP;



commandlineParameters cp;
int DPinitialized=0;
double humanArousal=1.0;
double arousalToTimeEffect=1.25;

//1.2 puts 2 fruits

double human2target=0.0;
double robot2target=0.0;





using namespace std;
//using namespace dpIO;
//#include <matplotpp/matplotpp.h>

//#include "matplotlibcpp.h"
//namespace plt = matplotlibcpp;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//----------------------------------------------------------------

int main(int argc, char **argv) {
	

#ifndef LocalTesting
	cp.nargc=argc;
	cp.nargv=argv;
	
	cout<<cp.nargc<<endl;
	cout<<cp.nargv[0]<<endl;
	cout<<cp.nargv[1]<<endl;
#endif

	gDP.graph2dp();

	cout<<"waiting DP to initialize...."<<endl;
	while (!DPinitialized){
//	  cout <<"DPinitialized:"<<DPinitialized<<endl;
	  cout<<"."<<endl;
	  sleep(1);
	}
	
        gDP.xproc->simulateRun();

	// sleep(999999999);

}



