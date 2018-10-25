

#ifndef _FUZZYTIMETBX_H
#define _FUZZYTIMETBX_H


//#include "/home/mmaniada/timestorm/igraph-0.7.1/include/igraph.h"
#include "fuzzy_calc.h"

class FuzzyTimeTBX
{
public:
    FuzzyTimeTBX(int id);
    ~FuzzyTimeTBX();
//    void set_fuzzy_time(igraph_t *g, int vn, int rid, struct FuzzyNumber r);
//    struct FuzzyNumber get_fuzzy_time(igraph_t *g, int en, int rid);
    void forientation(struct FuzzyNumber *v);
    struct FuzzyNumber fadd(struct FuzzyNumber v1, struct FuzzyNumber v2);
    struct FuzzyNumber fsubtract(struct FuzzyNumber v1, struct FuzzyNumber v2);
    struct FuzzyNumber fadd_float(struct FuzzyNumber v1, float f);
    struct FuzzyNumber fsubtract_float(struct FuzzyNumber v1, float f);
    int fisbigger(struct FuzzyNumber v1, struct FuzzyNumber v2);
    int fbiggerthanzero(struct FuzzyNumber v1);
    float defuz(struct FuzzyNumber v);

private:
    int PetalID;
    int VertexNum;

};

#endif
