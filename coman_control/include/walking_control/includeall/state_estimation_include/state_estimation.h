#include <vector>

void state_estimation(std::vector<double> acc_vec[3], double imuAccelerations[3],
                      double imuAngRates[3], double avg_acc[3], double bias_acceleration[3], double variance[3],
                    double &mean_norm, double rotOrient[3][3], double position[3],
                    double p_left_w[3], double p_right_w[3],double orientation[4], double tm, double &tm_1,
                    double bias_orientation[3], double velocity[3], double P[21][21], double velocity_straight[3], double p_pelvis_left[3],
                    double p_pelvis_right[3], double F_left, double F_right, double qSensAbsMed[31]);
