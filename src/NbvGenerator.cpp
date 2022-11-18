//
// Created by zhjd on 11/17/22.
//

#include "NbvGenerator.h"

namespace ORB_SLAM2
{

    NbvGenerator::NbvGenerator(){}

    NbvGenerator::NbvGenerator(Map* map, Tracking *pTracking, const string &strSettingPath):
    mpMap(map), mpTracker(pTracking)
    {  }

    void NbvGenerator::run() {
        std::vector<MapPlane*> MPls = mpMap->GetAllMapPlanes();
        

    }



};

