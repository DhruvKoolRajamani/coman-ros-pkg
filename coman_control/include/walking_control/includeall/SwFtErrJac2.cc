//
//-------------------------------------------------------------
//
//	ROBOTRAN - Version 6.6 (build : february 22, 2008)
//
//	Copyright 
//	Universite catholique de Louvain 
//	Departement de Mecanique 
//	Unite de Production Mecanique et Machines 
//	2, Place du Levant 
//	1348 Louvain-la-Neuve 
//	http://www.robotran.be// 
//
//	==> Generation Date : Tue Jan 10 18:41:48 2017
//
//	==> Project name : coman_init_feet
//	==> using XML input file 
//
//	==> Number of joints : 18
//
//	==> Function : F 8 : Constraints Vector (h) and jacobian Matrix (jac)
//	==> Flops complexity : 236
//
//	==> Generation Time :  0.010 seconds
//	==> Post-Processing :  0.000 seconds
//
//-------------------------------------------------------------
//
#include <math.h>
#include "user_hard_param.h"

/*! \brief compute the constraints vector h and its jacobian jac
 * 
 * \param[in] q leg joints positions (from hip to foot: pitch, roll, yaw, pitch, roll, pitch) [rad]
 * \param[in] swFtPosDes reference positions (x, y, z, pitch, roll, yaw) [m] or [rad]
 * \param[out] h constraints vector
 * \param[out] jac jacobian matrix
 */
void SwFtErrJac2(double q[], double swFtPosDes[], bool side, double h[], double jac[][2], double swFtPos[])
{ 
    // variables declaration
    double C10, S10, C11, S11, C12, S12, C13, S13, C14, S14, C15, S15, C16, S16, C17, S17, C18, S18;

    double RO_0_414, RO_0_614, RO_0_714, RO_0_914, RO_0_115, RO_0_215, RO_0_315, RO_0_415, RO_0_515, RO_0_615;
    double RO_0_116, RO_0_216, RO_0_316, RO_0_716, RO_0_816, RO_0_916, RO_0_417, RO_0_517, RO_0_617, RO_0_717;
    double RO_0_817, RO_0_917, RO_0_718, RO_0_818, RO_0_918, RL_0_115, RL_0_215, RL_0_315, RL_0_116, RL_0_216;
    double RL_0_316, RL_0_117, RL_0_217, RL_0_317, RL_0_119, RL_0_219, RL_0_319;

    double JT_0_119_13, JT_0_319_13, JT_0_119_14, JT_0_219_14;

    double h_1, h_2, h_3;

    double DPT_2_1 = -0.023;
    double DPT_2_2 = -0.0496;
    const double DPT_3_3 = -0.1024;
    const double DPT_3_4 = -0.1234;
    const double DPT_3_6 = -0.201;
    const double DPT_3_8 = -0.1;


    if (side == LEFT)
    {
        DPT_2_1 = 0.023;
        DPT_2_2 = 0.0496;
    }

    // By the convention on the robot q[4] is the ankle pitch and q[5] is the ankle roll; so, we switch them here!
    double qTemp = q[4];
    q[4] = q[5];
    q[5] = qTemp;


        // = = Block_0_0_0_0_0_2 = =

        // Trigonometric Variables



        // = = Block_0_0_0_0_0_3 = =

        // Trigonometric Variables
        q[4] = 0; //i.e., assuming that the ankle is fixed
        q[5] = 0;
        C13 = cos(q[0]);
        S13 = sin(q[0]);
        C14 = cos(q[1]);
        S14 = sin(q[1]);
        C15 = cos(q[2]);
        S15 = sin(q[2]);
        C16 = cos(q[3]);
        S16 = sin(q[3]);
        C17 = cos(q[4]);
        S17 = sin(q[4]);
        C18 = cos(q[5]);
        S18 = sin(q[5]);

        // = = Block_0_1_0_0_0_2 = =

        // Constraints and Constraints jacobian


        // = = Block_0_1_0_0_0_3 = =

        // Constraints and Constraints jacobian

        //
        RO_0_414 = S13*S14;
        RO_0_614 = C13*S14;
        RO_0_714 = S13*C14;
        RO_0_914 = C13*C14;
        RO_0_115 = RO_0_414*S15+C13*C15;
        RO_0_215 = C14*S15;
        RO_0_315 = RO_0_614*S15-S13*C15;
        RO_0_415 = RO_0_414*C15-C13*S15;
        RO_0_515 = C14*C15;
        RO_0_615 = RO_0_614*C15+S13*S15;
        RO_0_116 = RO_0_115*C16-RO_0_714*S16;
        RO_0_216 = RO_0_215*C16+S14*S16;
        RO_0_316 = RO_0_315*C16-RO_0_914*S16;
        RO_0_716 = RO_0_115*S16+RO_0_714*C16;
        RO_0_816 = RO_0_215*S16-S14*C16;
        RO_0_916 = RO_0_315*S16+RO_0_914*C16;
        RO_0_417 = RO_0_415*C17+RO_0_716*S17;
        RO_0_517 = RO_0_515*C17+RO_0_816*S17;
        RO_0_617 = RO_0_615*C17+RO_0_916*S17;
        RO_0_717 = -(RO_0_415*S17-RO_0_716*C17);
        RO_0_817 = -(RO_0_515*S17-RO_0_816*C17);
        RO_0_917 = -(RO_0_615*S17-RO_0_916*C17);
        RO_0_718 = RO_0_116*S18+RO_0_717*C18;
        RO_0_818 = RO_0_216*S18+RO_0_817*C18;
        RO_0_918 = RO_0_316*S18+RO_0_917*C18;
        RL_0_115 = RO_0_714*DPT_3_3;
        RL_0_215 = -DPT_3_3*S14;
        RL_0_315 = RO_0_914*DPT_3_3;
        RL_0_116 = RO_0_714*DPT_3_4;
        RL_0_216 = -DPT_3_4*S14;
        RL_0_316 = RO_0_914*DPT_3_4;
        RL_0_117 = RO_0_716*DPT_3_6;
        RL_0_217 = RO_0_816*DPT_3_6;
        RL_0_317 = RO_0_916*DPT_3_6;
        RL_0_119 = RO_0_718*DPT_3_8;
        RL_0_219 = RO_0_818*DPT_3_8;
        RL_0_319 = RO_0_918*DPT_3_8;
        JT_0_119_13 = RL_0_315+RL_0_316+RL_0_317+RL_0_319;
        JT_0_319_13 = -(RL_0_115+RL_0_116+RL_0_117+RL_0_119);
        JT_0_119_14 = S13*(RL_0_215+RL_0_216+RL_0_217+RL_0_219);
        JT_0_219_14 = -(RL_0_119*S13+RL_0_319*C13+DPT_3_3*C14+C13*(RL_0_316+RL_0_317)+S13*(RL_0_116+RL_0_117));


        // = = Block_0_1_0_0_1_0 = =

        // Constraints and Constraints jacobian

        swFtPos[0] = RL_0_115+RL_0_116+RL_0_117+RL_0_119;
        swFtPos[1] = RL_0_215+RL_0_216+RL_0_217+RL_0_219 + DPT_2_1+DPT_2_2;
        swFtPos[2] = h_3 = RL_0_315+RL_0_316+RL_0_317+RL_0_319;

        //
        h_1 = RL_0_115+RL_0_116+RL_0_117+RL_0_119-swFtPosDes[0];
        h_2 = RL_0_215+RL_0_216+RL_0_217+RL_0_219-swFtPosDes[1]+DPT_2_1+DPT_2_2;

//    // = = Block_0_3_0_0_0_0 = =

//    // Symbolic Outputs

    h[0] = h_1;
    h[1] = h_2;


    jac[0][0] = JT_0_119_13;
    jac[0][1] = JT_0_119_14;
    jac[1][1] = JT_0_219_14;
        // ====== END Task 0 ======
}
 

