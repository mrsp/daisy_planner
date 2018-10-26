
#ifndef _DP_H
#define _DP_H


#include "fuzzy_calc.h"
#include "DP_processor.h"


class DP 
    {
    public:
        DP();
        ~DP();
        bool initDP();

        DP_processor *xproc;
        int test;

	void addMenuIngredientsInPlan(int ing1, int ing2, int ing3, int ing4);
	void addPlanConstraints();
	void graph2dp();
	void reCreateGraph();
	void makeMenuPlan(int v1, int v2, int v3, int v4);

//	void addPetal_urgentJob(igraph_t *cg, int ingr_id);
	
//	igraph_t* globalG;
//	igraph_t sub1g1;


    };

#endif
