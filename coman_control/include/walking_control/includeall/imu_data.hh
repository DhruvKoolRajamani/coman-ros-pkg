class imu_data
{
public:
    void get_Orientation(double Trans [][3], double cdPR[][3], double Orientation[]);
    void get_AngRates(double ImuAngRates[], double cdPR[][3], double AngRates[]);
    void get_Accelerations(double ImuAccelerations[], double cdPR[][3], double Accelerations[]);
};
