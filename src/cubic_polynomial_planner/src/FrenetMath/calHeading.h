//
// Created by ustb on 19-7-8.
//

#ifndef LATTICEPLANNER_CALHEADING_H
#define LATTICEPLANNER_CALHEADING_H

#include <array>
#include "selfType.h"

namespace lattice_planner {

double cal_FrenetHeading(lattice_planner::FrenetPose pose0,
                         lattice_planner::FrenetPose pose1);
}

#endif  // LATTICEPLANNER_CALHEADING_H
