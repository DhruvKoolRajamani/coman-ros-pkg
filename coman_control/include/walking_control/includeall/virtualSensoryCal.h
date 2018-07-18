void virtualSensoryCal(std::ifstream &inputFile, double &tm, double *qSens, double *qSensAbs, double *dqSens, double *tauSens, double *forceRightAnkle, double *forceLeftAnkle,\
                       double *torqueRightAnkle, double *torqueLeftAnkle, double *forceRightHand, double *forceLeftHand, double trans[][3], \
                       double *imuAngRates, double *imuAccelerations, double ignoreVar)
{
    double n = 31;
    inputFile >> tm;
    // start_id = 2
    for (int i = 0; i < n; i++){
        inputFile >>  qSens[i];
    }
    // start_id = 2 + n
    for (int i = 0; i < n; i++){
        inputFile >>  qSensAbs[i];
    }
    // start_id = 2 + 2N
    for (int i = 0; i < n; i++)
    {
        inputFile >>  dqSens[i];
    }
    // start_id = 2 + 3N
    for (int i = 0; i < 3; i++)
    {
        inputFile >> ignoreVar;
    }
    // start_id = 5 + 3N
    for (int i = 0; i < 3; i++)
    {
        inputFile >>  ignoreVar;
    }
    // start_id = 8 + 3N
    for (int i = 0; i < 3; i++)
    {
        inputFile >>  ignoreVar;
    }
    // start_id = 11 + 3N
    for (int i = 0; i < 3; i++)
    {
        inputFile >>  ignoreVar;
    }
    // start_id = 14 + 3N
    for (int i = 0; i < 3; i++)
    {
        inputFile >>  ignoreVar;
    }
    // start_id = 17 + 3N
    for (int i = 0; i < 3; i++)
    {
        inputFile >>  forceRightAnkle[i];
    }
    // start_id = 20 + 3N
    for (int i = 0; i < 3; i++)
    {
        inputFile >>  forceLeftAnkle[i];
    }
    // start_id = 23 + 3N
    for (int i = 0; i < 3; i++)
    {
        inputFile >>  torqueRightAnkle[i];
    }
    // start_id = 26 + 3N
    for (int i = 0; i < 3; i++)
    {
        inputFile >>  torqueLeftAnkle[i];
    }
    // start_id = 29 + 3N
    for (int i = 0; i < 3; i++)
    {
        inputFile >>  forceRightHand[i];
    }
    // start_id = 32 + 3N
    for (int i = 0; i < 3; i++)
    {
        inputFile >>  forceLeftHand[i];
    }
    // start_id = 35 + 3N
    for (int i = 0; i < 3; i++)
    {
        inputFile >>  ignoreVar;
    }
    // start_id = 38 + 3N
    for (int i = 0; i < 3; i++)
    {
        inputFile >>  ignoreVar;
    }
    // start_id = 41 + 3N
    for (int i = 0; i < 3; i++)
    {
        inputFile >>  ignoreVar;
    }
    // start_id = 44 + 3N
    for (int i = 0; i < 3; i++)
    {
        inputFile >>  ignoreVar;
    }
    // start_id = 47 + 3N
    for (int i = 0; i < n; i++)
    {
        inputFile >>  ignoreVar;
    }
    // start_id = 47 + 4N
    for (int i = 0; i < n; i++)
    {
        inputFile >>  ignoreVar;
    }
    // start_id = 47 + 5N
    for (int i = 0; i < 3; i++)
    {
        inputFile >>  ignoreVar;
    }
    // start_id = 50 + 5N
    for (int i = 0; i < 3; i++)
    {
        inputFile >>  ignoreVar;
    }
    // start_id = 53 + 5N
    for (int i = 0; i < 3; i++)
    {
        inputFile >>  ignoreVar;
    }
    // start_id = 56 + 5N
    for (int i = 0; i < n; i++)
    {
        inputFile >>  tauSens[i];
    }
    // start_id = 56 + 6N
    for (int i = 0; i < n; i++)
    {
        inputFile >>  ignoreVar;
    }
    // start_id = 56 + 7N
    for (int i = 0; i < 2; i++)
    {
        inputFile >>  ignoreVar;
    }
    // start_id = 58 + 7N
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            inputFile >>  trans[i][j];
        }
    }
    // start_id = 67 + 7N
    for (int i = 0; i < 3; i++)
    {
        inputFile >>  imuAngRates[i];
    }
    // start_id = 70 + 7N
    for (int i = 0; i < 3; i++)
    {
        inputFile >>  imuAccelerations[i];
    }
    for (int i = 0; i < n; i++)
    {
        inputFile >> ignoreVar;
    }
    for (int i = 0; i < 3; i++)
    {
        inputFile >> ignoreVar;
    }
    // start_id = 73 + 7N
    inputFile >>  ignoreVar // 73 + 7N
              >>  ignoreVar // 74 + 7N
              >>  ignoreVar // 75 + 7N
              >>  ignoreVar // 76 + 7N
              >>  ignoreVar // 77 + 7N
              >>  ignoreVar // 78 + 7N
              >>  ignoreVar // 79 + 7N
              >>  ignoreVar // 80 + 7N
              >>  ignoreVar // 81 + 7N
              >>  ignoreVar // 82 + 7N
              >>  ignoreVar // 83 + 7N
              >>  ignoreVar // 84 + 7N
              >>  ignoreVar // 85 + 7N
              >>  ignoreVar // 86 + 7N
              >>  ignoreVar // 87 + 7N
              >>  ignoreVar
              >>  ignoreVar
              >>  ignoreVar
              >>  ignoreVar
              >>  ignoreVar
              >>  ignoreVar
              >>  ignoreVar
              >>  ignoreVar;
}
