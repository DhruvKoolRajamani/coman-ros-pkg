#include "saturate.hh"
double saturate(double input, double u, double l)
{

    input = input > u ? u : (input<l ? l : input);
    return input;
//
//    if (input > u)
//        return u;
//    else if (input < l)
//        return l;
//    else
//        return input;
}
double saturatel(double input, double l)
{
    input = input < l ? l : input;
    return input;
}
double saturateu(double input, double u)
{
    input = input > u ? u : input;
    return input;
}
