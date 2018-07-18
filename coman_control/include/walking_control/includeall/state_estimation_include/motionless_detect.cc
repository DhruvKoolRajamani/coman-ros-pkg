// Computing mean and variance over a given time of the accelerometer data

#include <algorithm>
#include <vector>
#include <iostream>
void motionless_detect(std::vector<double> &vec_acc, double &variance, double &avgOfAccVec, const double &acc, const unsigned int &SIZE)
{
    double avgOfAccVecNew = 0;

        // compute mean and variance when the number of samples acquired is smaller than SIZE
        if (vec_acc.size() < SIZE){
            vec_acc.push_back(acc);
            avgOfAccVecNew = (avgOfAccVec * (vec_acc.size() - 1) + acc) / vec_acc.size();
            variance = ((variance * (vec_acc.size() - 1))+(acc - avgOfAccVecNew) * (acc - avgOfAccVecNew) \
                              + (vec_acc.size() - 1) * (acc - avgOfAccVec) \
                              *(acc-avgOfAccVec)/(vec_acc.size() * vec_acc.size())) / vec_acc.size();
            avgOfAccVec = avgOfAccVecNew;
        }

        // compute mean and variance for the last SIZE samples
        else
        {
            vec_acc.push_back(acc);
            avgOfAccVec += (acc - vec_acc[0]) / SIZE;
            variance += ((acc - avgOfAccVec)*(acc - avgOfAccVec) - (vec_acc[0] - avgOfAccVec)*(vec_acc[0] - avgOfAccVec)\
                            + (acc - vec_acc[0])*(acc - vec_acc[0]) / SIZE) / SIZE;
            vec_acc.erase(vec_acc.begin());
        }
}


