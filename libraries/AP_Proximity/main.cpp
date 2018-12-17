//
// File: main.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 12-Nov-2018 17:08:31
//

//***********************************************************************
// This automatically generated example C main file shows how to call
// entry-point functions that MATLAB Coder generated. You must customize
// this file for your application. Do not modify this file directly.
// Instead, make a copy of this file, modify it, and integrate it into
// your development environment.
//
// This file initializes entry-point function arguments to a default
// size and value before calling the entry-point functions. It does
// not store or use any values returned from the entry-point functions.
// If necessary, it does pre-allocate memory for returned values.
// You can use this file as a starting point for a main function that
// you can deploy in your application.
//
// After you copy the file, and before you deploy it, you must make the
// following changes:
// * For variable-size function arguments, change the example sizes to
// the sizes that your application requires.
// * Change the example values of function arguments to the values that
// your application requires.
// * If the entry-point functions return values, store these values or
// otherwise use them as required by your application.
//
//***********************************************************************
// Include Files
#include "rt_nonfinite.h"
#include "ObstacleAvoiding.h"
#include "main.h"

// Function Declarations
static void argInit_1xd32_real_T(double result_data[], int result_size[2]);
static double argInit_real_T();
static void main_ObstacleAvoiding();

// Function Definitions

//
// Arguments    : double result_data[]
//                int result_size[2]
// Return Type  : void
//
static void argInit_1xd32_real_T(double result_data[], int result_size[2])
{
  int idx1;

  // Set the size of the array.
  // Change this size to the value that the application requires.
  result_size[0] = 1;
  result_size[1] = 2;

  // Loop over the array to initialize each element.
  for (idx1 = 0; idx1 < 2; idx1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result_data[idx1] = argInit_real_T();
  }
}

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : void
// Return Type  : void
//
static void main_ObstacleAvoiding()
{
  double roll_FK;
  double pitch_FK;
  double yaw_FK;
  double range_data[32];
  int range_size[2];
  double angleV_data[32];
  int angleV_size[2];
  double angleH_data[32];
  int angleH_size[2];

  // Initialize function 'ObstacleAvoiding' input arguments.
  roll_FK = argInit_real_T();
  pitch_FK = argInit_real_T();
  yaw_FK = argInit_real_T();

  // Initialize function input argument 'range'.
  argInit_1xd32_real_T(range_data, range_size);

  // Initialize function input argument 'angleV'.
  argInit_1xd32_real_T(angleV_data, angleV_size);

  // Initialize function input argument 'angleH'.
  argInit_1xd32_real_T(angleH_data, angleH_size);

  // Call the entry-point 'ObstacleAvoiding'.
  roll_FK = ObstacleAvoiding(roll_FK, pitch_FK, yaw_FK, range_data, range_size,
    angleV_data, angleV_size, angleH_data, angleH_size, argInit_real_T());
}

//
// Arguments    : int argc
//                const char * const argv[]
// Return Type  : int
//
int main(int, const char * const [])
{
  // Initialize the application.
  // You do not need to do this more than one time.
  ObstacleAvoiding_initialize();

  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_ObstacleAvoiding();

  // Terminate the application.
  // You do not need to do this more than one time.
  ObstacleAvoiding_terminate();
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
