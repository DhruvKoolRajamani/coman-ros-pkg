void compute_matrix(double imuAngRates[3], double bias_orientation[3], double imuAcc[3], double bias_acceleration[3],
                    double rotOrient[3][3], const double p_left_w[3], const double p_right_w[3], const double position[3], double D_x[21],
                    double P[21][21], double Dy[6], double F_left, double F_right, double dt);
