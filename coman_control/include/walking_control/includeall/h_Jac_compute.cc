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
//	==> Function : F 8 : Constraints Vector (h) and Jacobian Matrix (Jac) 
//	==> Flops complexity : 236
//
//	==> Generation Time :  0.010 seconds
//	==> Post-Processing :  0.000 seconds
//
//-------------------------------------------------------------
//
#include <math.h> 
#include "user_hard_param.h"

/*! \brief compute the constraints vector h and its jacobian Jac
 * 
 * \param[in] q leg joints positions (from hip to foot: pitch, roll, yaw, pitch, roll, pitch) [rad]
 * \param[in] q_ref reference positions (x, y, z, pitch, roll, yaw) [m] or [rad]
 * \param[out] h constraints vector
 * \param[out] Jac jacobian matrix
 */
void h_Jac_compute(double q[6], double q_ref[6], double h[6], double Jac[6][6])
{ 
    // variables declaration
    double C10, S10, C11, S11, C12, S12, C13, S13, C14, S14, C15, S15, C16, S16, C17, S17, C18, S18;

    double RO_0_414, RO_0_614, RO_0_714, RO_0_914, RO_0_115, RO_0_215, RO_0_315, RO_0_415, RO_0_515, RO_0_615;
    double RO_0_116, RO_0_216, RO_0_316, RO_0_716, RO_0_816, RO_0_916, RO_0_417, RO_0_517, RO_0_617, RO_0_717;
    double RO_0_817, RO_0_917, RO_0_718, RO_0_818, RO_0_918, RL_0_115, RL_0_215, RL_0_315, RL_0_116, RL_0_216;
    double RL_0_316, RL_0_117, RL_0_217, RL_0_317, RL_0_119, RL_0_219, RL_0_319;

    double JT_0_119_13, JT_0_319_13, JT_0_119_14, JT_0_219_14, JT_0_319_14, JT_0_119_15;
    double JT_0_219_15, JT_0_319_15, JT_0_119_16, JT_0_219_16, JT_0_319_16, JT_0_119_17;
    double JT_0_219_17, JT_0_319_17, JT_0_119_18, JT_0_219_18, JT_0_319_18;

    double RO_1_411, RO_1_611, RO_1_711, RO_1_911;
    double h_1, h_2, h_3, h_4, h_5, h_6;

    // = = Block_0_0_0_0_0_2 = =
    // note need to change these if side is left
    double DPT_2_1 = -0.023;
    double DPT_2_2 = -0.0496;
    const double DPT_3_3 = -0.1024;
    const double DPT_3_4 = -0.1234;
    const double DPT_3_6 = -0.201;
    const double DPT_3_8 = -0.1;
    // Trigonometric Variables  

    C10 = cos(q_ref[3]);
    S10 = sin(q_ref[3]);
    C11 = cos(q_ref[4]);
    S11 = sin(q_ref[4]);
    C12 = cos(q_ref[5]);
    S12 = sin(q_ref[5]);

    // = = Block_0_0_0_0_0_3 = = 

    // Trigonometric Variables  

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

    // Constraints and Constraints Jacobian 

    //
    RO_1_411 = S10*S11;
    RO_1_611 = C10*S11;
    RO_1_711 = S10*C11;
    RO_1_911 = C10*C11;

    // = = Block_0_1_0_0_0_3 = = 

    // Constraints and Constraints Jacobian 

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
    JT_0_319_14 = C13*(RL_0_215+RL_0_216+RL_0_217+RL_0_219);
    JT_0_119_15 = -(RO_0_914*(RL_0_217+RL_0_219)+S14*(RL_0_317+RL_0_319));
    JT_0_219_15 = -(RO_0_714*(RL_0_317+RL_0_319)-RO_0_914*(RL_0_117+RL_0_119));
    JT_0_319_15 = RO_0_714*(RL_0_217+RL_0_219)+S14*(RL_0_117+RL_0_119);
    JT_0_119_16 = RO_0_515*(RL_0_317+RL_0_319)-RO_0_615*(RL_0_217+RL_0_219);
    JT_0_219_16 = -(RO_0_415*(RL_0_317+RL_0_319)-RO_0_615*(RL_0_117+RL_0_119));
    JT_0_319_16 = RO_0_415*(RL_0_217+RL_0_219)-RO_0_515*(RL_0_117+RL_0_119);
    JT_0_119_17 = -(RL_0_219*RO_0_316-RL_0_319*RO_0_216);
    JT_0_219_17 = RL_0_119*RO_0_316-RL_0_319*RO_0_116;
    JT_0_319_17 = -(RL_0_119*RO_0_216-RL_0_219*RO_0_116);
    JT_0_119_18 = -(RL_0_219*RO_0_617-RL_0_319*RO_0_517);
    JT_0_219_18 = RL_0_119*RO_0_617-RL_0_319*RO_0_417;
    JT_0_319_18 = -(RL_0_119*RO_0_517-RL_0_219*RO_0_417);

    // = = Block_0_1_0_0_1_0 = = 

    // Constraints and Constraints Jacobian 

    //
    h_1 = RL_0_115+RL_0_116+RL_0_117+RL_0_119-q_ref[0];
    h_2 = RL_0_215+RL_0_216+RL_0_217+RL_0_219-q_ref[1]+DPT_2_1+DPT_2_2;
    h_3 = RL_0_315+RL_0_316+RL_0_317+RL_0_319-q_ref[2];
    h_4 = RO_0_617*C11*C12-RO_0_918*S11+C11*S12*(RO_0_316*C18-RO_0_917*S18);
    h_5 = RO_0_417*(RO_1_611*C12+S10*S12)+RO_0_718*RO_1_911+(RO_0_116*C18-RO_0_717*S18)*(RO_1_611*S12-S10*C12);
    h_6 = RO_0_517*(RO_1_411*C12-C10*S12)+RO_0_818*RO_1_711+(RO_1_411*S12+C10*C12)*(RO_0_216*C18-RO_0_817*S18);

    // = = Block_0_3_0_0_0_0 = = 

    // Symbolic Outputs

    h[0] = h_1;
    h[1] = h_2;
    h[2] = h_3;
    h[3] = h_4;
    h[4] = h_5;
    h[5] = h_6;

    Jac[0][0] = JT_0_119_13;
    Jac[0][1] = JT_0_119_14;
    Jac[0][2] = JT_0_119_15;
    Jac[0][3] = JT_0_119_16;
    Jac[0][4] = JT_0_119_17;
    Jac[0][5] = JT_0_119_18;

    Jac[1][1] = JT_0_219_14;
    Jac[1][2] = JT_0_219_15;
    Jac[1][3] = JT_0_219_16;
    Jac[1][4] = JT_0_219_17;
    Jac[1][5] = JT_0_219_18;

    Jac[2][0] = JT_0_319_13;
    Jac[2][1] = JT_0_319_14;
    Jac[2][2] = JT_0_319_15;
    Jac[2][3] = JT_0_319_16;
    Jac[2][4] = JT_0_319_17;
    Jac[2][5] = JT_0_319_18;

    Jac[3][1] = C13;
    Jac[3][2] = RO_0_714;
    Jac[3][3] = RO_0_415;
    Jac[3][4] = RO_0_116;
    Jac[3][5] = RO_0_417;

    Jac[4][2] = -S14;
    Jac[4][3] = RO_0_515;
    Jac[4][4] = RO_0_216;
    Jac[4][5] = RO_0_517;

    Jac[5][1] = -S13;
    Jac[5][2] = RO_0_914;
    Jac[5][3] = RO_0_615;
    Jac[5][4] = RO_0_316;
    Jac[5][5] = RO_0_617;

    // ====== END Task 0 ====== 
}
 

