//
// File: ObstacleAvoiding.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 27-Nov-2018 10:32:13
//
#ifndef OBSTACLEAVOIDING_H
#define OBSTACLEAVOIDING_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "ObstacleAvoiding_types.h"

// Function Declarations
extern double ObstacleAvoiding(double roll_FK, double pitch_FK, double yaw_FK,
  const double range_data[], const int range_size[2], const double angleV_data[],
  const int angleV_size[2], const double angleH_data[], const int angleH_size[2],
  double pitchOffset);
extern void ObstacleAvoiding_initialize();
extern void ObstacleAvoiding_terminate();

#endif

//
// File trailer for ObstacleAvoiding.h
//
// [EOF]
//
