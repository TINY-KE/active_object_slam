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
    NbvGenerator(Map* map);

    std::vector<cv::Mat> mvCandidates;

private:
    Map* mMap;



};

}


#endif //ACTIVE_EAO_NEW_NBVGENERATOR_H
