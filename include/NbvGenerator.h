//
// Created by zhjd on 11/17/22.
//

#ifndef ACTIVE_EAO_NEW_NBVGENERATOR_H
#define ACTIVE_EAO_NEW_NBVGENERATOR_H

#include "Map.h"

namespace ORB_SLAM2
{
class Tracking;
class FrameDrawer;
class MapPublisher;
class MapDrawer;
class System;

class NbvGenerator {

public:
    NbvGenerator();
    NbvGenerator(Map* map, Tracking *pTracking, const string &strSettingPath);

    std::vector<cv::Mat> mvCandidates;

    void run();

private:
    Map* mpMap;
    Tracking* mpTracker;



};

}


#endif //ACTIVE_EAO_NEW_NBVGENERATOR_H
