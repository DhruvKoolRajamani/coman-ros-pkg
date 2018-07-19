#include <math.h>
# define N 31
#include <iostream>
#include <fstream>

using namespace std;

void init_pos(double _tm, double Pos_sens0[], double Qfinal[], double Pos_sens[], double Vel_sens[], double tauDes[], int whichComan)
{
    double y[N];  // desired output to be driven to zero
    double dy[N]; // derivative of the desired output to be driven to zero
    double pos_des;
    double vel_des;
    double Kp[N], Kd[N];
    double ALPHA = 0.3;

    Pos_sens0[0] = 0;
    Pos_sens0[12] = 0;
    Pos_sens0[13] = 0;
    
    for ( int i = 0; i < 23; i++ )
    {
        Kp[i] = 300;
        Kd[i] = 2;
    }

    for ( int i = 23; i < N; i++ )
    {
        Kp[i] = 2;
        Kd[i] = 0.1;
    }

    // for ( int i = 0; i< N; i++ )
    // {
    //     Kp[i] = 0;
    //     Kd[i] = 0;
    // }

    // Kp[8] = 300;
    // Kp[13] = 300;

    for ( int i = 0; i < N; i++ )
    {
        pos_des = Qfinal[i]+(Pos_sens0[i]-Qfinal[i])*exp(-ALPHA * _tm);
        vel_des = -ALPHA*(Pos_sens0[i]-Qfinal[i])*exp(-ALPHA * _tm);
        y[i] = Pos_sens[i]-pos_des;
        dy[i] = Vel_sens[i]-vel_des;
        
        /* Uncomment for debugging */
        cout << _tm << " : " << i << " : " << Pos_sens[i] << " : " 
             << Pos_sens0[i] << " : " << Qfinal[i] << " : " 
             << pos_des << " : " << tauDes[i] << endl;
    }

    for ( int i= 0; i< N; i++ )
    {
        double temp;
        temp = -Kp[i]*(y[i]) - Kd[i]*(dy[i]);
        if( temp < 10000000000000 && temp > -10000000000000 )
            tauDes[i] = temp;
        else
        {
            tauDes[i] = 0.0;
        }
    }
}


