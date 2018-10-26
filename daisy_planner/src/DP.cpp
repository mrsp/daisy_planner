
#include "DP.h"
#include "FuzzyTimeTBX.h"
#include <iostream>
#include <unistd.h>   


extern int DPinitialized;


using namespace std;

/*--------------------------------------------------------------------*/

DP::DP()
{
    int a=1;
    std::cout << "in the DP !" << '\n' ;

  DPinitialized=0;


}

/*--------------------------------------------------------------------*/

DP::~DP(){


}


/*--------------------------------------------------------------------*/

void DP::graph2dp(){

  xproc = new DP_processor();
  xproc->init();

//  xproc->scanDaisy(globalG);
//  xproc->backConstraints();
//  xproc->updateExpectedTimes(globalG);
  xproc->n_updateExpectedTimes();
  xproc->setPetalsInPlan();
  xproc->showPetals();
  cout <<"\nPetals printed.......\n\n"<<  xproc->petalNumber<<endl;


//      xproc->updateAllPetalInfo(globalG);

       xproc->updateAllPetalInfo();

       DPinitialized=1;

}

/*--------------------------------------------------------------------*/
void DP::makeMenuPlan(int v1, int v2, int v3, int v4){

      DPinitialized=0;

      cout<<"re-creating graph"<<endl;
      
	graph2dp();
	DPinitialized=1;
	cout <<"\nReady to make plans.......\n\n";

}
