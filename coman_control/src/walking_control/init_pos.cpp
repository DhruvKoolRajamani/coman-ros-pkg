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

    // Kp[23] = 5;
    // Kd[23] = 0;

    // Kp[24] = 5;
    // Kd[24] = 0;
    
    // Kp[25] = 5;
    // Kd[25] = 0;
    
    // Kp[26] = 5;
    // Kd[26] = 0;
    
    // Kp[27] = 5;
    // Kd[27] = 0;
    
    // Kp[28] = 5;
    // Kd[28] = 0;
    
    // Kp[29] = 5;
    // Kd[29] = 0;
    
    // Kp[30] = 5;
    // Kd[30] = 0;

    for ( int i = 0; i < N; i++ )
    {
        pos_des = Qfinal[i]+(Pos_sens0[i]-Qfinal[i])*exp(-ALPHA * _tm);
        vel_des = -ALPHA*(Pos_sens0[i]-Qfinal[i])*exp(-ALPHA * _tm);
        y[i] = Pos_sens[i]-pos_des;
        dy[i] = Vel_sens[i]-vel_des;

        // cout << _tm << " : " << i << " : " << Pos_sens[i] << " : " 
        //      << Pos_sens0[i] << " : " << Qfinal[i] << " : " 
        //      << pos_des << " : " << tauDes[i] << endl;
    }

    for ( int i= 0; i< N; i++ )
    {
        double temp;
        temp = -Kp[i]*(y[i]) - Kd[i]*(dy[i]);
        if( temp < 1000000000000 && temp > -10000000000000 )
            tauDes[i] = temp;
        else
        {
            cout << i << "\n******************************\n";
            tauDes[i] = 1.0;
        }
    }
}


