#include <math.h>
# define N 31
#include <iostream>
#include <fstream>

using namespace std;

double Kp[N], Kd[N];
double ALPHA = 1;

void init_pos(double _tm, double Pos_sens0[], double Qfinal[], double Pos_sens[], double Vel_sens[], double tauDes[], int whichComan)
{
    double y[N];  // desired output to be driven to zero
    double dy[N]; // derivative of the desired output to be driven to zero
    double pos_des;
    double vel_des;

    // Lower Body
    
    for (int i = 0; i < 23; i++)
    {
        Kp[i] = 10;
        Kd[i] = 0.1;
    }

    Kp[23] = 5;
    Kd[23] = 0;

    Kp[24] = 5;
    Kd[24] = 0;
    
    Kp[25] = 5;
    Kd[25] = 0;
    
    Kp[26] = 0;
    Kd[26] = 0;
    
    Kp[27] = 5;
    Kd[27] = 0;
    
    Kp[28] = 5;
    Kd[28] = 0;
    
    Kp[29] = 0;
    Kd[29] = 0;
    
    Kp[30] = 0;
    Kd[30] = 0;
    
    Kp[31] = 0;
    Kd[31] = 0;
    
    for ( int i = 0; i < N; i++ )
    {
        if ( i == 15 )
        {
            y[i] = 0.0;
            dy[i] = 0.0;
        }
        else
        {
            // pos_des = qinit<vector> + *(error -> sensor - qinit)*e^(-alpha*_tm)
            pos_des = Qfinal[i]+(Pos_sens0[i]-Qfinal[i])*exp(-ALPHA * _tm);
            vel_des = -ALPHA*(Pos_sens0[i]-Qfinal[i])*exp(-ALPHA * _tm);
            y[i] = Pos_sens[i]-pos_des;
            dy[i] = Vel_sens[i]-vel_des;
            // cout << _tm << " : here : ";

            // y[i] = Qfinal[i] - Pos_sens[i];
            // dy[i] = Qfinal[i] - Pos_sens[i];
        }
    }

    for ( int i= 0; i< N; i++ )
    {
        double temp;
        temp = -Kp[i]*(y[i]) - Kd[i]*(dy[i]);
        tauDes[i] = temp;
        
        cout << i << " : " <<  Pos_sens[i] << " : " << temp << " : "  << pos_des << endl;
    }
}


