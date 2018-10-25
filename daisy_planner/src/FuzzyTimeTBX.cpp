#include "FuzzyTimeTBX.h"



/*The fuzzy ToolBox implements fuzzy calculus*/

//*******************************************************
//*******************************************************
FuzzyTimeTBX::FuzzyTimeTBX(int id){
    int PetalID=id;
    int VertexNum=0;
}

//*******************************************************
//*******************************************************
FuzzyTimeTBX::~FuzzyTimeTBX(){


}

/*--------------------------------------------------------------------*/
void FuzzyTimeTBX::forientation(struct FuzzyNumber *v){
    float tmp;
    if (v->m1 > v->m2){
        tmp=v->m1;
        v->m1 = v->m2;
        v->m2=tmp;
    }
    else if (v->s > v->e){
        tmp=v->s;
        v->s = v->e;
        v->e = tmp;
    }
}


/*--------------------------------------------------------------------*/
struct FuzzyNumber FuzzyTimeTBX::fadd(struct FuzzyNumber v1, struct FuzzyNumber v2){
    struct FuzzyNumber r;

    r.s=v1.s+v2.s;
    r.m1=v1.m1+v2.m1;
    r.m2=v1.m2+v2.m2;
    r.e=v1.e+v2.e;

    forientation(&r);
    return r;
//   struct a_tag *p;
//   p = (struct a_tag *) malloc(sizeof(struct a_tag));
}

/*--------------------------------------------------------------------*/
struct FuzzyNumber FuzzyTimeTBX::fsubtract(struct FuzzyNumber v1, struct FuzzyNumber v2){
    struct FuzzyNumber r;

    r.s=v1.s-v2.s;
    r.m1=v1.m1-v2.m1;
    r.m2=v1.m2-v2.m2;
    r.e=v1.e-v2.e;

    forientation(&r);
    return r;
//   struct a_tag *p;
//   p = (struct a_tag *) malloc(sizeof(struct a_tag));
}

/*--------------------------------------------------------------------*/
struct FuzzyNumber FuzzyTimeTBX::fadd_float(struct FuzzyNumber v1, float f){
    struct FuzzyNumber r;

    r.s=v1.s+f;
    r.m1=v1.m1+f;
    r.m2=v1.m2+f;
    r.e=v1.e+f;

    forientation(&r);
    return r;
}

/*--------------------------------------------------------------------*/
struct FuzzyNumber FuzzyTimeTBX::fsubtract_float(struct FuzzyNumber v1, float f){
    struct FuzzyNumber r;

    r.s=v1.s-f;
    r.m1=v1.m1-f;
    r.m2=v1.m2-f;
    r.e=v1.e-f;

    forientation(&r);
    return r;
}

/*--------------------------------------------------------------------*/
int FuzzyTimeTBX::fisbigger(struct FuzzyNumber v1, struct FuzzyNumber v2){

    if (v1.s+2*v1.m1+2*v1.m2+v1.e > v2.s+2*v2.m1+2*v2.m2+v2.e)
        return 1;
    else
        return 0;
}

/*--------------------------------------------------------------------*/
int FuzzyTimeTBX::fbiggerthanzero(struct FuzzyNumber v1){

    if (v1.s+2*v1.m1+2*v1.m2+v1.e > 0.0)
        return 1;
    else
        return 0;
}


/*--------------------------------------------------------------------*/
float FuzzyTimeTBX::defuz(struct FuzzyNumber v){

  //      printf("(%f,%f,%f,%f)=%f ",v.s,v.m1,v.m2,v.e,v.s+2*v.m1+2*v.m2+v.e);
    return (v.s+2*v.m1+2*v.m2+v.e)/6.0;
}

/*--------------------------------------------------------------------*/

