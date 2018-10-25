
#ifndef _FUZZY_CALC_H
#define _FUZZY_CALC_H



//#define MAX_CONSTRAINTS 10
#define MAX_GRAPHS 3

struct constraint{  //the current constraint formulation assumes that [smallv < greatv +thresh]
  int smallv;
  int greatv;
  int thresh;
};


struct FuzzyNumber
{
   float s;
   float m1;
   float m2;
   float e;
};  

#endif
