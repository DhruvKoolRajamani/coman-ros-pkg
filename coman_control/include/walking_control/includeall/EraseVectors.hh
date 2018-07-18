//#include "Parameters.hh"

std::vector<double> pxVec, pyVec, pxswfVec, pyswfVec, vxVec, vyVec, vxswfVec, vyswfVec, axVec, thpFVec, thrFVec, dthpFVec, dthrFVec, kVec, pxAbsVec, pyAbsVec, vxFkVec, vxAbsVec, vyAbsVec, swFtPitchVec, dswFtPitchVec, swFtRollVec, dswFtRollVec, vecPxAbs, pxAbsMedVec, vxAbsVecF, frcRVec[3], frcLVec[3], trqRVec[3], trqLVec[3];

std::vector<double> FRxVec, FRyVec, FRzVec, FLxVec, FLyVec, FLzVec, pHand, vHandVec, pHandVectTemp, relVelVec, velTimeWind, intForceVec, forceLeftAnkleZVec, forceRightAnkleZVec, sensorExtForceVec, tmVec; // frcRyVec, frcRzVec, frcLyVec, frcLzVec, trqRyVec, trqRzVec, trqLyVec, trqLzVec  

static std::vector<double> pPelvisAbsVec[3], pPelvisAbsFVec[3];

void EraseVectors()
{
    pxVec.clear();
       pxVec.clear();
       pyVec.clear();
       pxswfVec.clear();
       pyswfVec.clear();
   //    vxVec.clear();
   //    vyVec.clear();
       vxswfVec.clear();
       vyswfVec.clear();
       axVec.clear();
       thpFVec.clear();
       thrFVec.clear();
       dthpFVec.clear();
       dthrFVec.clear();
       kVec.clear();
       pxAbsVec.clear();
       pxAbsMedVec.clear();
       pyAbsVec.clear();
   //    vxFkVec.clear();
   //    vxAbsVec.clear();
   //    vyAbsVec.clear();
       swFtPitchVec.clear();
       dswFtPitchVec.clear();
       swFtRollVec.clear();
       dswFtRollVec.clear();
       vecPxAbs.clear();
    //    tmVec.clear();
    //    forceLeftAnkleZVec.clear();
    //    forceRightAnkleZVec.clear();
       for (int i(0); i < 3; i++)
       {
           pPelvisAbsVec[i].clear();
           pPelvisAbsFVec[i].clear();
        //    frcRVec[i].clear();
        //    frcLVec[i].clear();
        //    trqRVec[i].clear();
        //    trqLVec[i].clear();
       }
}
