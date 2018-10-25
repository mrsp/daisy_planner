
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

/*--------------------------------------------------------------------*/
/*
bool DP::initDP()
{

    igraph_t g, g2;
    FILE *ifile;
    igraph_vector_t gtypes, vtypes, etypes;
    igraph_strvector_t gnames, vnames, enames;
    long int i;
    igraph_vector_t y;
    igraph_strvector_t id;
    char str[20];



  //gia na paroyme to subgraph, des igraph_induced_subgraph

  //  int max_constraintsMAX_CONSTRAINTS;
    int active_constraints=0;
    igraph_t tg;
//    igraph_t sub1g1;
    igraph_t sub1g2;
    
    globalG=&sub1g1;

    igraph_t **GraphList;
    GraphList = (igraph_t **) malloc(MAX_GRAPHS*sizeof(igraph_t *));
    int **Plan;
    Plan = (int **) malloc(MAX_GRAPHS*sizeof(int *));

    igraph_i_set_attribute_table(&igraph_cattribute_table);


}

*/
