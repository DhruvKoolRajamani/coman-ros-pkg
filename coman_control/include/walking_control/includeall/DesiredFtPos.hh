void DesiredFtPos(const double &pxAbs, const double &pyAbs, const double &tInStep, double pxAbs0, const double &vxDes, const double &vxAbsF, const double &vxAbs, const double &vy,\
                  const int &sg, double &deltaX, double &deltaY, double &kv, const double &frontalBias, const double &s, const double &T)
{
    double kvP = T / 2;
    double kvN = 1.5 * T /2;
    double ALPHA_kv = 100;
    if (vxDes == 0){
        pxAbs0 = 0;
    }
    double deltaP = (pxAbs - (vxDes * tInStep + pxAbs0));
//    kv = kvP + (kvN - kvP) / (1 + exp(ALPHA_kv * (vxAbs)));

    kvP = 0.3;
    kvN = 0.3; // used kvN = 0.6 for stepping in place
    double kp = kvP + (kvN - kvP) / (1 + exp(ALPHA_kv * (pxAbs)));
    double kp2 = 100 * kp;
    //temp
    kv = 0;
//    double kp = 0.2, kp2 = 20;
    if (s >= 0.15){
        deltaX = 1 * saturate(kp * deltaP + kp2 * pow(deltaP, 3) + kv * (vxAbsF - vxDes),0.15,-0.1);//TEMP 0 *
    }
    else{
        deltaX = 0;
    }
    // right leg is stance --> sg  = 1 otherwise sg = -1
    double vYdes = -0.4 + 0.8 / T * tInStep;
    if(s > 0.2){
//        deltaY = saturate(sg*saturatel(sg*0.3*(vy-sg*0.15),0),0.07,-0.07);//TEMP 0
        deltaY = 1 * sg * saturate(0.45 * (sg * vy -  (0 * vYdes) + 0 * (pyAbs - sg * 0.075)) + 3 * sg * frontalBias, 0.2, 0);
//        deltaY += 1 * sg * saturate(0.6 * (sg * pyAbs - sg * (0.0)), 0.09, 0);
    }
    else{
        deltaY = 0;
    }
}
