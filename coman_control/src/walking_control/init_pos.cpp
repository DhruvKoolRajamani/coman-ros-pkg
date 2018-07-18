#include <math.h>
# define N 31
#include <iostream>
#include <fstream>

using namespace std;

double Kp[N], Kd[N];
double ALPHA = 1;

void init_pos(double time, double Pos_sens0[], double Qfinal[], double Pos_sens[], double Vel_sens[], double tauDes[], int whichComan)
{
    double y[N];  // desired output to be driven to zero
    double dy[N]; // derivative of the desired output to be driven to zero
    double pos_des;
    double vel_des;

    // for ( int i= 0; i< N; i++ )
    // {
    //     y[i] = 0.0;
    //     dy[i] = 0.0;
    // }

    // Lower Body
    
    for (int i = 0; i < 23; i++)
    {
        Kp[i] = 300;
        Kd[i] = 3;
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
    
    for (int i = 0; i < N; i++)
    {
        // pos_des = qinit<vector> + *(error -> sensor - qinit)*e^(-alpha*time)
        pos_des = Qfinal[i]+(Pos_sens0[i]-Qfinal[i])*exp(-ALPHA * time);
        vel_des = -ALPHA*(Pos_sens0[i]-Qfinal[i])*exp(-ALPHA * time);
        y[i] = Pos_sens[i]-pos_des;
        dy[i] = Vel_sens[i]-vel_des;
    }

    for (int i = 0; i < N; i++)
    {
        double temp;
        if (i == 28)
        {
            temp = 1.0;
        }
        else
        {
            temp = Kp[i]*(y[i]) + Kd[i]*(dy[i]);
        }
        cout << i << " : " << temp << endl;
        tauDes[i] = temp;
    }

}


