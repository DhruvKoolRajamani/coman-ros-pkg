#include "bezier.hh"
#include <math.h>
int bezier::factorial(int n)
{
    int fact = n;
    if (n == 0)
    {
        fact = 1;
        return fact;
    }
    else
        return fact*bezier::factorial(n-1);

}

double bezier::get_bezier(double alpha[], double s)
{
    double b = 0;
    double c;
    int M = degree;
    for (int i = 0; i <= M; i++)
    {
        c = factorial(M)/(factorial(i)*factorial(M-i));
        b = b+c*alpha[i]*pow(s,i)*pow((1-s),M-i);
    }
    return b;
}
double bezier::get_derv_bezier(double alpha[], double s)
{
    double b = 0;
    double c;
    int M = degree;
    for (int i = 1; i <= M; i++)
    {
        c = factorial(M)/(factorial(i)*factorial(M-i));
        b = b+c*alpha[i]*s*pow(s,i-1)*pow((1-s),M-i);
    }
    for (int i = 0; i <= M-1; i++)
    {
        c = factorial(M)/(factorial(i)*factorial(M-i));
        b = b-c*alpha[i]*s*pow(s,i)*pow((1-s),M-i-1);
    }
    return b;
}

