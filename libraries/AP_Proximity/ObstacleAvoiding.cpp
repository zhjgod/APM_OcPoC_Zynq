//
// File: ObstacleAvoiding.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 27-Nov-2018 10:32:13
//

// Include Files
#include <math.h>
#include "rt_nonfinite.h"
#include <float.h>
#include <string.h>
#include "ObstacleAvoiding.h"

// Variable Definitions
static double yaw_PostProcessing[20];
static boolean_T yaw_PostProcessing_not_empty;
static double countNum;

// Function Declarations
static void b_cosd(double x_data[], int x_size[2]);
static void b_sind(double x_data[], int x_size[2]);
static void c_cosd(double *x);
static void c_sind(double *x);
static void circshift(double a[20]);
static double mean(const double x_data[], const int x_size[2]);
static void rotx(double alpha, double rotmat[9]);
static void roty(double beta, double rotmat[9]);
static void rotz(double b_gamma, double rotmat[9]);
static double rt_remd_snf(double u0, double u1);
static double sum(const boolean_T x_data[], const int x_size[2]);

// Function Definitions

//
// Arguments    : double x_data[]
//                int x_size[2]
// Return Type  : void
//
static void b_cosd(double x_data[], int x_size[2])
{
  int nx;
  int k;
  double x;
  double absx;
  signed char n;
  nx = x_size[1];
  for (k = 0; k < nx; k++) {
    if (!((!rtIsInf(x_data[k])) && (!rtIsNaN(x_data[k])))) {
      x = rtNaN;
    } else {
      x = rt_remd_snf(x_data[k], 360.0);
      absx = fabs(x);
      if (absx > 180.0) {
        if (x > 0.0) {
          x -= 360.0;
        } else {
          x += 360.0;
        }

        absx = fabs(x);
      }

      if (absx <= 45.0) {
        x *= 0.017453292519943295;
        n = 0;
      } else if (absx <= 135.0) {
        if (x > 0.0) {
          x = 0.017453292519943295 * (x - 90.0);
          n = 1;
        } else {
          x = 0.017453292519943295 * (x + 90.0);
          n = -1;
        }
      } else if (x > 0.0) {
        x = 0.017453292519943295 * (x - 180.0);
        n = 2;
      } else {
        x = 0.017453292519943295 * (x + 180.0);
        n = -2;
      }

      if (n == 0) {
        x = cos(x);
      } else if (n == 1) {
        x = -sin(x);
      } else if (n == -1) {
        x = sin(x);
      } else {
        x = -cos(x);
      }
    }

    x_data[k] = x;
  }
}

//
// Arguments    : double x_data[]
//                int x_size[2]
// Return Type  : void
//
static void b_sind(double x_data[], int x_size[2])
{
  int nx;
  int k;
  double x;
  double absx;
  signed char n;
  nx = x_size[1];
  for (k = 0; k < nx; k++) {
    if (!((!rtIsInf(x_data[k])) && (!rtIsNaN(x_data[k])))) {
      x = rtNaN;
    } else {
      x = rt_remd_snf(x_data[k], 360.0);
      absx = fabs(x);
      if (absx > 180.0) {
        if (x > 0.0) {
          x -= 360.0;
        } else {
          x += 360.0;
        }

        absx = fabs(x);
      }

      if (absx <= 45.0) {
        x *= 0.017453292519943295;
        n = 0;
      } else if (absx <= 135.0) {
        if (x > 0.0) {
          x = 0.017453292519943295 * (x - 90.0);
          n = 1;
        } else {
          x = 0.017453292519943295 * (x + 90.0);
          n = -1;
        }
      } else if (x > 0.0) {
        x = 0.017453292519943295 * (x - 180.0);
        n = 2;
      } else {
        x = 0.017453292519943295 * (x + 180.0);
        n = -2;
      }

      if (n == 0) {
        x = sin(x);
      } else if (n == 1) {
        x = cos(x);
      } else if (n == -1) {
        x = -cos(x);
      } else {
        x = -sin(x);
      }
    }

    x_data[k] = x;
  }
}

//
// Arguments    : double *x
// Return Type  : void
//
static void c_cosd(double *x)
{
  double absx;
  signed char n;
  if (!((!rtIsInf(*x)) && (!rtIsNaN(*x)))) {
    *x = rtNaN;
  } else {
    *x = rt_remd_snf(*x, 360.0);
    absx = fabs(*x);
    if (absx > 180.0) {
      if (*x > 0.0) {
        *x -= 360.0;
      } else {
        *x += 360.0;
      }

      absx = fabs(*x);
    }

    if (absx <= 45.0) {
      *x *= 0.017453292519943295;
      n = 0;
    } else if (absx <= 135.0) {
      if (*x > 0.0) {
        *x = 0.017453292519943295 * (*x - 90.0);
        n = 1;
      } else {
        *x = 0.017453292519943295 * (*x + 90.0);
        n = -1;
      }
    } else if (*x > 0.0) {
      *x = 0.017453292519943295 * (*x - 180.0);
      n = 2;
    } else {
      *x = 0.017453292519943295 * (*x + 180.0);
      n = -2;
    }

    if (n == 0) {
      *x = cos(*x);
    } else if (n == 1) {
      *x = -sin(*x);
    } else if (n == -1) {
      *x = sin(*x);
    } else {
      *x = -cos(*x);
    }
  }
}

//
// Arguments    : double *x
// Return Type  : void
//
static void c_sind(double *x)
{
  double absx;
  signed char n;
  if (!((!rtIsInf(*x)) && (!rtIsNaN(*x)))) {
    *x = rtNaN;
  } else {
    *x = rt_remd_snf(*x, 360.0);
    absx = fabs(*x);
    if (absx > 180.0) {
      if (*x > 0.0) {
        *x -= 360.0;
      } else {
        *x += 360.0;
      }

      absx = fabs(*x);
    }

    if (absx <= 45.0) {
      *x *= 0.017453292519943295;
      n = 0;
    } else if (absx <= 135.0) {
      if (*x > 0.0) {
        *x = 0.017453292519943295 * (*x - 90.0);
        n = 1;
      } else {
        *x = 0.017453292519943295 * (*x + 90.0);
        n = -1;
      }
    } else if (*x > 0.0) {
      *x = 0.017453292519943295 * (*x - 180.0);
      n = 2;
    } else {
      *x = 0.017453292519943295 * (*x + 180.0);
      n = -2;
    }

    if (n == 0) {
      *x = sin(*x);
    } else if (n == 1) {
      *x = cos(*x);
    } else if (n == -1) {
      *x = -cos(*x);
    } else {
      *x = -sin(*x);
    }
  }
}

//
// Arguments    : double a[20]
// Return Type  : void
//
static void circshift(double a[20])
{
  double unusedU0;
  int k;
  unusedU0 = a[19];
  for (k = 18; k >= 0; k--) {
    a[k + 1] = a[k];
  }

  a[0] = unusedU0;
}

//
// Arguments    : const double x_data[]
//                const int x_size[2]
// Return Type  : double
//
static double mean(const double x_data[], const int x_size[2])
{
  double y;
  int k;
  y = x_data[0];
  for (k = 2; k <= x_size[1]; k++) {
    y += x_data[k - 1];
  }

  y /= (double)x_size[1];
  return y;
}

//
// Arguments    : double alpha
//                double rotmat[9]
// Return Type  : void
//
static void rotx(double alpha, double rotmat[9])
{
  double d0;
  double d1;
  double d2;
  double d3;
  int i0;
  static const signed char iv0[3] = { 1, 0, 0 };

  d0 = alpha;
  c_cosd(&d0);
  d1 = alpha;
  c_sind(&d1);
  d2 = alpha;
  c_sind(&d2);
  d3 = alpha;
  c_cosd(&d3);
  for (i0 = 0; i0 < 3; i0++) {
    rotmat[3 * i0] = iv0[i0];
  }

  rotmat[1] = 0.0;
  rotmat[4] = d0;
  rotmat[7] = -d1;
  rotmat[2] = 0.0;
  rotmat[5] = d2;
  rotmat[8] = d3;
}

//
// Arguments    : double beta
//                double rotmat[9]
// Return Type  : void
//
static void roty(double beta, double rotmat[9])
{
  double d4;
  double d5;
  double d6;
  double d7;
  int i1;
  static const signed char iv1[3] = { 0, 1, 0 };

  d4 = beta;
  c_cosd(&d4);
  d5 = beta;
  c_sind(&d5);
  d6 = beta;
  c_sind(&d6);
  d7 = beta;
  c_cosd(&d7);
  rotmat[0] = d4;
  rotmat[3] = 0.0;
  rotmat[6] = d5;
  for (i1 = 0; i1 < 3; i1++) {
    rotmat[1 + 3 * i1] = iv1[i1];
  }

  rotmat[2] = -d6;
  rotmat[5] = 0.0;
  rotmat[8] = d7;
}

//
// Arguments    : double b_gamma
//                double rotmat[9]
// Return Type  : void
//
static void rotz(double b_gamma, double rotmat[9])
{
  double d8;
  double d9;
  double d10;
  double d11;
  int i2;
  static const signed char iv2[3] = { 0, 0, 1 };

  d8 = b_gamma;
  c_cosd(&d8);
  d9 = b_gamma;
  c_sind(&d9);
  d10 = b_gamma;
  c_sind(&d10);
  d11 = b_gamma;
  c_cosd(&d11);
  rotmat[0] = d8;
  rotmat[3] = -d9;
  rotmat[6] = 0.0;
  rotmat[1] = d10;
  rotmat[4] = d11;
  rotmat[7] = 0.0;
  for (i2 = 0; i2 < 3; i2++) {
    rotmat[2 + 3 * i2] = iv2[i2];
  }
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_remd_snf(double u0, double u1)
{
  double y;
  double b_u1;
  double q;
  if (!((!rtIsNaN(u0)) && (!rtIsInf(u0)) && ((!rtIsNaN(u1)) && (!rtIsInf(u1)))))
  {
    y = rtNaN;
  } else {
    if (u1 < 0.0) {
      b_u1 = ceil(u1);
    } else {
      b_u1 = floor(u1);
    }

    if ((u1 != 0.0) && (u1 != b_u1)) {
      q = fabs(u0 / u1);
      if (fabs(q - floor(q + 0.5)) <= DBL_EPSILON * q) {
        y = 0.0 * u0;
      } else {
        y = fmod(u0, u1);
      }
    } else {
      y = fmod(u0, u1);
    }
  }

  return y;
}

//
// Arguments    : const boolean_T x_data[]
//                const int x_size[2]
// Return Type  : double
//
static double sum(const boolean_T x_data[], const int x_size[2])
{
  double y;
  int k;
  if (x_size[1] == 0) {
    y = 0.0;
  } else {
    y = x_data[0];
    for (k = 2; k <= x_size[1]; k++) {
      y += (double)x_data[k - 1];
    }
  }

  return y;
}

//
// Arguments    : double roll_FK
//                double pitch_FK
//                double yaw_FK
//                const double range_data[]
//                const int range_size[2]
//                const double angleV_data[]
//                const int angleV_size[2]
//                const double angleH_data[]
//                const int angleH_size[2]
//                double pitchOffset
// Return Type  : double
//
double ObstacleAvoiding(double roll_FK, double pitch_FK, double yaw_FK, const
  double range_data[], const int range_size[2], const double angleV_data[],
  const int angleV_size[2], const double angleH_data[], const int angleH_size[2],
  double pitchOffset)
{
  double nearestX;
  int tmp_size[2];
  double tmp_data[1];
  int idx;
  double yaw_com;
  double b_tmp_data[20];
  int TarZ_size[2];
  double TarZ_data[32];
  int b_tmp_size[2];
  double c_tmp_data[32];
  int c_tmp_size[2];
  double d_tmp_data[32];
  int d_tmp_size[2];
  double e_tmp_data[32];
  int e_tmp_size[2];
  double f_tmp_data[32];
  int boffset;
  double oldC_data[96];
  double a[9];
  double b[9];
  double b_b[9];
  double b_a[9];
  int coffset;
  double y[9];
  int i;
  double TarPos_data[96];
  int k;
  boolean_T g_tmp_data[32];
  int aoffset;
  int f_tmp_size[2];
  boolean_T h_tmp_data[32];
  boolean_T i_tmp_data[32];
  int j_tmp_data[32];
  boolean_T exitg1;

  //  historyLen=20;
  //  persistent yaw_PostProcessing;
  //  persistent pitch_PostProcessing;
  //  persistent roll_PostProcessing;
  //  persistent countNum;
  //
  //  historyLenMid=10;
  //  persistent RangValHistorySetMid;   %用于中值滤波的中间变量
  //  persistent countNumMid; %用于中值滤波的计数量
  //  AlarmHeightThdUp=1;    %警戒相对高度上门限
  //  AlarmHeightThdDown=-1;    %警戒相对高度下门限
  // 警戒相对高度上门限
  // 警戒相对高度下门限
  // 输入为弧度，转换为度
  // 输入为弧度，转换为度
  yaw_FK = yaw_FK * 180.0 / 3.1415926535897931;

  // 输入为弧度，转换为度
  // 若雷达未装正引起的系统差。
  if (!yaw_PostProcessing_not_empty) {
    countNum = 1.0;
    memset(&yaw_PostProcessing[0], 0, 20U * sizeof(double));
    yaw_PostProcessing_not_empty = true;
    yaw_PostProcessing[0] = yaw_FK;
    tmp_size[0] = 1;
    tmp_size[1] = 1;
    tmp_data[0] = yaw_PostProcessing[0];
    yaw_com = yaw_FK - mean(tmp_data, tmp_size);
  } else {
    countNum++;
    if (countNum >= 20.0) {
      countNum = 20.0;
    }

    circshift(yaw_PostProcessing);
    yaw_PostProcessing[0] = yaw_FK;
    tmp_size[0] = 1;
    tmp_size[1] = (int)countNum;
    idx = (int)countNum;
    if (0 <= idx - 1) {
      memcpy(&b_tmp_data[0], &yaw_PostProcessing[0], (unsigned int)(idx * (int)
              sizeof(double)));
    }

    yaw_com = yaw_FK - mean(b_tmp_data, tmp_size);
  }

  //  if isempty(countNum)
  //      countNum=1;
  //      yaw_PostProcessing=zeros(1,historyLen);
  //      yaw_PostProcessing(1)=yaw_FK;
  //      yaw_mean=mean(yaw_PostProcessing(1:countNum));
  //      yaw_com=yaw_FK-yaw_mean;
  //      pitch_PostProcessing=zeros(1,historyLen);
  //      pitch_PostProcessing(1)=pitch_FK;
  //      pitch_mean=mean(pitch_PostProcessing(1:countNum));
  //      roll_PostProcessing=zeros(1,historyLen);
  //      roll_PostProcessing(1)=roll_FK;
  //      roll_mean=mean(roll_PostProcessing(1:countNum));
  //  else
  //      countNum=countNum+1;
  //      if countNum >= historyLen
  //          countNum=historyLen;
  //      end
  //      yaw_PostProcessing=circshift(yaw_PostProcessing,1);
  //      yaw_PostProcessing(1)=yaw_FK;
  //      yaw_mean=mean(yaw_PostProcessing(1:countNum));
  //      yaw_com=yaw_FK-yaw_mean;
  //      pitch_PostProcessing=circshift(pitch_PostProcessing,1);
  //      pitch_PostProcessing(1)=pitch_FK;
  //      pitch_mean=mean(pitch_PostProcessing(1:countNum));
  //
  //      roll_PostProcessing=circshift(roll_PostProcessing,1);
  //      roll_PostProcessing(1)=roll_FK;
  //      roll_mean=mean(roll_PostProcessing(1:countNum));
  //  end
  if (!(range_size[1] == 0)) {
    // 考虑水平角
    TarZ_size[0] = 1;
    TarZ_size[1] = angleV_size[1];
    idx = angleV_size[0] * angleV_size[1];
    if (0 <= idx - 1) {
      memcpy(&TarZ_data[0], &angleV_data[0], (unsigned int)(idx * (int)sizeof
              (double)));
    }

    b_cosd(TarZ_data, TarZ_size);
    b_tmp_size[0] = 1;
    b_tmp_size[1] = angleH_size[1];
    idx = angleH_size[0] * angleH_size[1];
    if (0 <= idx - 1) {
      memcpy(&c_tmp_data[0], &angleH_data[0], (unsigned int)(idx * (int)sizeof
              (double)));
    }

    b_cosd(c_tmp_data, b_tmp_size);
    c_tmp_size[0] = 1;
    c_tmp_size[1] = angleV_size[1];
    idx = angleV_size[0] * angleV_size[1];
    if (0 <= idx - 1) {
      memcpy(&d_tmp_data[0], &angleV_data[0], (unsigned int)(idx * (int)sizeof
              (double)));
    }

    b_cosd(d_tmp_data, c_tmp_size);
    d_tmp_size[0] = 1;
    d_tmp_size[1] = angleH_size[1];
    idx = angleH_size[0] * angleH_size[1];
    if (0 <= idx - 1) {
      memcpy(&e_tmp_data[0], &angleH_data[0], (unsigned int)(idx * (int)sizeof
              (double)));
    }

    b_sind(e_tmp_data, d_tmp_size);
    e_tmp_size[0] = 1;
    e_tmp_size[1] = angleV_size[1];
    idx = angleV_size[0] * angleV_size[1];
    if (0 <= idx - 1) {
      memcpy(&f_tmp_data[0], &angleV_data[0], (unsigned int)(idx * (int)sizeof
              (double)));
    }

    b_sind(f_tmp_data, e_tmp_size);
    idx = range_size[1];
    for (boffset = 0; boffset < idx; boffset++) {
      oldC_data[3 * boffset] = range_data[range_size[0] * boffset] *
        TarZ_data[TarZ_size[0] * boffset] * c_tmp_data[b_tmp_size[0] * boffset];
    }

    idx = range_size[1];
    for (boffset = 0; boffset < idx; boffset++) {
      oldC_data[1 + 3 * boffset] = range_data[range_size[0] * boffset] *
        d_tmp_data[c_tmp_size[0] * boffset] * e_tmp_data[d_tmp_size[0] * boffset];
    }

    idx = range_size[1];
    for (boffset = 0; boffset < idx; boffset++) {
      oldC_data[2 + 3 * boffset] = range_data[range_size[0] * boffset] *
        f_tmp_data[e_tmp_size[0] * boffset];
    }

    //      TarPos=rotx(roll_mean)*roty(pitch_mean)*rotz(yaw_com)*oldC;
    rotx(roll_FK * 180.0 / 3.1415926535897931, a);
    roty(pitch_FK * 180.0 / 3.1415926535897931 + pitchOffset, b);
    rotz(yaw_com, b_b);
    for (boffset = 0; boffset < 3; boffset++) {
      for (idx = 0; idx < 3; idx++) {
        b_a[boffset + 3 * idx] = 0.0;
        for (coffset = 0; coffset < 3; coffset++) {
          b_a[boffset + 3 * idx] += a[boffset + 3 * coffset] * b[coffset + 3 *
            idx];
        }
      }

      for (idx = 0; idx < 3; idx++) {
        y[boffset + 3 * idx] = 0.0;
        for (coffset = 0; coffset < 3; coffset++) {
          y[boffset + 3 * idx] += b_a[boffset + 3 * coffset] * b_b[coffset + 3 *
            idx];
        }
      }
    }

    for (idx = 0; idx < range_size[1]; idx++) {
      coffset = idx * 3;
      boffset = idx * 3;
      for (i = 0; i < 3; i++) {
        TarPos_data[coffset + i] = 0.0;
      }

      for (k = 0; k < 3; k++) {
        if (oldC_data[boffset + k] != 0.0) {
          aoffset = k * 3;
          for (i = 0; i < 3; i++) {
            TarPos_data[coffset + i] += oldC_data[boffset + k] * y[aoffset + i];
          }
        }
      }
    }

    idx = range_size[1];
    for (boffset = 0; boffset < idx; boffset++) {
      TarZ_data[boffset] = -TarPos_data[2 + 3 * boffset];
    }

    idx = range_size[1];
    for (boffset = 0; boffset < idx; boffset++) {
      g_tmp_data[boffset] = (TarZ_data[boffset] < 3.0);
    }

    idx = range_size[1];
    for (boffset = 0; boffset < idx; boffset++) {
      h_tmp_data[boffset] = (TarZ_data[boffset] > -0.5);
    }

    f_tmp_size[0] = 1;
    f_tmp_size[1] = range_size[1];
    idx = range_size[1];
    for (boffset = 0; boffset < idx; boffset++) {
      i_tmp_data[boffset] = (g_tmp_data[boffset] && h_tmp_data[boffset]);
    }

    if (sum(i_tmp_data, f_tmp_size) == 0.0) {
      nearestX = 999.0;
    } else {
      idx = range_size[1];
      for (boffset = 0; boffset < idx; boffset++) {
        TarZ_data[boffset] = TarPos_data[3 * boffset];
      }

      coffset = range_size[1] - 1;
      boffset = 0;
      for (i = 0; i <= coffset; i++) {
        if (g_tmp_data[i] && h_tmp_data[i]) {
          boffset++;
        }
      }

      idx = 0;
      for (i = 0; i <= coffset; i++) {
        if (g_tmp_data[i] && h_tmp_data[i]) {
          j_tmp_data[idx] = i + 1;
          idx++;
        }
      }

      if (boffset <= 2) {
        if (boffset == 1) {
          nearestX = TarZ_data[j_tmp_data[0] - 1];
        } else if ((TarZ_data[j_tmp_data[0] - 1] > TarZ_data[j_tmp_data[1] - 1])
                   || (rtIsNaN(TarZ_data[j_tmp_data[0] - 1]) && (!rtIsNaN
                     (TarZ_data[j_tmp_data[1] - 1])))) {
          nearestX = TarZ_data[j_tmp_data[1] - 1];
        } else {
          nearestX = TarZ_data[j_tmp_data[0] - 1];
        }
      } else {
        if (!rtIsNaN(TarZ_data[j_tmp_data[0] - 1])) {
          idx = 1;
        } else {
          idx = 0;
          k = 2;
          exitg1 = false;
          while ((!exitg1) && (k <= boffset)) {
            if (!rtIsNaN(TarZ_data[j_tmp_data[k - 1] - 1])) {
              idx = k;
              exitg1 = true;
            } else {
              k++;
            }
          }
        }

        if (idx == 0) {
          nearestX = TarZ_data[j_tmp_data[0] - 1];
        } else {
          nearestX = TarZ_data[j_tmp_data[idx - 1] - 1];
          while (idx + 1 <= boffset) {
            if (nearestX > TarZ_data[j_tmp_data[idx] - 1]) {
              nearestX = TarZ_data[j_tmp_data[idx] - 1];
            }

            idx++;
          }
        }
      }
    }
  } else {
    nearestX = 999.0;
  }

  //  % %=====================中值滤波================
  //  % if isempty(countNumMid)
  //  %     countNumMid=1;
  //  %     RangValHistorySetMid=zeros(1,historyLenMid);
  //  %     RangValHistorySetMid(1)=nearestX_temp;
  //  %     %         RangVal_mean=sum(RangValHistorySetMid)/countNumMid;  % 平均滤波 
  //  %     nearestX=median(RangValHistorySetMid(1:countNumMid));  % 中值滤波
  //  % else
  //  %     countNumMid=countNumMid+1;
  //  %     if countNumMid >= historyLenMid
  //  %         countNumMid=historyLenMid;
  //  %     end
  //  %     RangValHistorySetMid=circshift(RangValHistorySetMid,1);
  //  %     RangValHistorySetMid(1)=nearestX_temp;
  //  %     %         RangVal_mean=sum(RangValHistorySetMid)/countNumMid; % 平均滤波 
  //  %     nearestX=median(RangValHistorySetMid(1:countNumMid));  % 中值滤波
  //  % end
  return nearestX;
}

//
// Arguments    : void
// Return Type  : void
//
void ObstacleAvoiding_initialize()
{
  rt_InitInfAndNaN(8U);
  yaw_PostProcessing_not_empty = false;
}

//
// Arguments    : void
// Return Type  : void
//
void ObstacleAvoiding_terminate()
{
  // (no terminate code required)
}

//
// File trailer for ObstacleAvoiding.cpp
//
// [EOF]
//
