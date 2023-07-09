/* Simscape target specific file.
 * This file is generated for the Simscape network associated with the solver block 'simcape_real_time_V2/Solver Configuration'.
 */

#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "sm_ssci_run_time_errors.h"
#include "sm_RuntimeDerivedValuesBundle.h"
#include "simcape_real_time_V2_e8621d4_1_geometries.h"

PmfMessageId simcape_real_time_V2_e8621d4_1_recordLog(const
  RuntimeDerivedValuesBundle *rtdv, const int *eqnEnableFlags, const double
  *state, const int *modeVector, const double *input, const double *inputDot,
  const double *inputDdot, double *logVector, double *errorResult,
  NeuDiagnosticManager *neDiagMgr)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[86];
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  (void) modeVector;
  (void) inputDot;
  (void) neDiagMgr;
  xx[0] = 57.29577951308232;
  xx[1] = 0.4999998320075168;
  xx[2] = 0.5;
  xx[3] = xx[2] * state[12];
  xx[4] = cos(xx[3]);
  xx[5] = 0.5000002051767252;
  xx[6] = sin(xx[3]);
  xx[3] = xx[1] * xx[4] - xx[5] * xx[6];
  xx[7] = 0.499999917555661;
  xx[8] = 0.5000000452600188;
  xx[9] = xx[7] * xx[4] - xx[8] * xx[6];
  xx[10] = xx[1] * xx[6] + xx[5] * xx[4];
  xx[11] = xx[8] * xx[4] + xx[7] * xx[6];
  xx[12] = xx[3];
  xx[13] = xx[9];
  xx[14] = xx[10];
  xx[15] = xx[11];
  xx[4] = 2.0;
  xx[6] = 2.454648318984227e-7;
  xx[16] = xx[2] * state[5];
  xx[17] = sin(xx[16]);
  xx[18] = xx[6] * xx[17];
  xx[19] = 7.436862347702244e-8;
  xx[20] = xx[19] * xx[17];
  xx[21] = xx[18];
  xx[22] = xx[20];
  xx[23] = xx[17];
  xx[24] = 7.436850053016743e-8;
  xx[25] = xx[24] * xx[20];
  xx[26] = xx[25] + xx[17];
  xx[27] = xx[24] * xx[18];
  xx[28] = 5.008735751212025e-7;
  xx[29] = xx[28] * xx[17];
  xx[30] = xx[27] - xx[29];
  xx[31] = xx[28] * xx[20];
  xx[32] = xx[18] + xx[31];
  xx[33] = - xx[26];
  xx[34] = xx[30];
  xx[35] = xx[32];
  pm_math_Vector3_cross_ra(xx + 21, xx + 33, xx + 36);
  xx[21] = cos(xx[16]);
  xx[16] = (xx[4] * (xx[36] + xx[21] * xx[26]) - xx[28]) * state[10];
  xx[22] = 0.707106604100972;
  xx[23] = xx[2] * state[3];
  xx[26] = cos(xx[23]);
  xx[33] = 1.130781874238157e-7;
  xx[34] = sin(xx[23]);
  xx[23] = 0.7071069582720672;
  xx[35] = 6.049167292397939e-8;
  xx[39] = xx[22] * xx[26] - xx[33] * xx[34];
  xx[40] = xx[23] * xx[34] - xx[35] * xx[26];
  xx[41] = - (xx[33] * xx[26] + xx[22] * xx[34]);
  xx[42] = - (xx[23] * xx[26] + xx[35] * xx[34]);
  xx[22] = xx[2] * state[4];
  xx[23] = sin(xx[22]);
  xx[43] = cos(xx[22]);
  xx[44] = - (xx[28] * xx[23]);
  xx[45] = xx[23];
  xx[46] = - (xx[24] * xx[23]);
  pm_math_Quaternion_compose_ra(xx + 39, xx + 43, xx + 47);
  xx[39] = xx[21];
  xx[40] = xx[18];
  xx[41] = xx[20];
  xx[42] = xx[17];
  pm_math_Quaternion_compose_ra(xx + 47, xx + 39, xx + 43);
  xx[22] = xx[43] * xx[46];
  xx[23] = xx[44] * xx[45];
  xx[26] = xx[22] + xx[23];
  xx[33] = xx[26] * xx[4];
  xx[34] = xx[33] * state[9];
  xx[35] = xx[6] * state[11];
  xx[39] = xx[46] * xx[46];
  xx[40] = xx[44] * xx[44];
  xx[41] = 1.0;
  xx[42] = (xx[39] + xx[40]) * xx[4] - xx[41];
  xx[47] = xx[42] * state[9];
  xx[48] = (xx[41] + xx[4] * (xx[37] - xx[21] * xx[30])) * state[10];
  xx[30] = xx[19] * state[11];
  xx[49] = xx[43] * xx[44];
  xx[50] = xx[45] * xx[46];
  xx[51] = xx[49] - xx[50];
  xx[52] = xx[4] * xx[51];
  xx[53] = xx[52] * state[9];
  xx[36] = (xx[4] * (xx[38] - xx[21] * xx[32]) - xx[24]) * state[10];
  xx[54] = xx[16] - xx[34] + xx[35];
  xx[55] = xx[47] + xx[48] + xx[30];
  xx[56] = xx[53] + xx[36] + state[11];
  pm_math_Quaternion_inverseXform_ra(xx + 12, xx + 54, xx + 57);
  xx[32] = xx[58] + state[13];
  xx[60] = xx[57];
  xx[61] = xx[32];
  xx[62] = xx[59];
  xx[37] = 1.720624935967864e-4;
  xx[38] = 1.666733335e-4;
  xx[58] = 1.286950159678639e-5;
  xx[63] = xx[37] * xx[57];
  xx[64] = xx[32] * xx[38];
  xx[65] = xx[58] * xx[59];
  pm_math_Vector3_cross_ra(xx + 60, xx + 63, xx + 66);
  xx[32] = 9.766669284693602e-20;
  xx[57] = 5.721451332149614e-3;
  xx[59] = xx[11] * xx[57];
  xx[60] = xx[57] * xx[9];
  xx[61] = 0.2250904050306244 - xx[4] * (xx[59] * xx[3] - xx[10] * xx[60]);
  xx[62] = 1.19047008300415e-7 - ((xx[11] * xx[59] + xx[60] * xx[9]) * xx[4] -
    xx[57]);
  xx[63] = 0.03674263500980687 + (xx[60] * xx[3] + xx[10] * xx[59]) * xx[4];
  pm_math_Vector3_cross_ra(xx + 54, xx + 61, xx + 9);
  pm_math_Vector3_cross_ra(xx + 54, xx + 9, xx + 64);
  pm_math_Quaternion_inverseXform_ra(xx + 12, xx + 64, xx + 9);
  xx[3] = xx[25] + xx[17];
  xx[64] = - xx[18];
  xx[65] = - xx[20];
  xx[66] = - xx[17];
  xx[10] = xx[27] - xx[29];
  xx[11] = xx[18] + xx[31];
  xx[68] = xx[3];
  xx[69] = - xx[10];
  xx[70] = - xx[11];
  pm_math_Vector3_cross_ra(xx + 64, xx + 68, xx + 71);
  xx[17] = (xx[3] * xx[21] + xx[71]) * xx[4] - xx[28];
  xx[27] = - xx[34];
  xx[28] = xx[47];
  xx[29] = xx[53];
  pm_math_Vector3_cross_ra(xx + 54, xx + 27, xx + 64);
  xx[27] = xx[35];
  xx[28] = xx[30];
  xx[29] = state[11];
  xx[68] = xx[16];
  xx[69] = xx[48];
  xx[70] = xx[36];
  pm_math_Vector3_cross_ra(xx + 27, xx + 68, xx + 34);
  xx[3] = xx[64] + xx[34];
  xx[16] = xx[65] + xx[35];
  xx[18] = xx[41] + xx[4] * (xx[72] - xx[10] * xx[21]);
  xx[10] = xx[66] + xx[36];
  xx[20] = (xx[73] - xx[21] * xx[11]) * xx[4] - xx[24];
  xx[27] = xx[17] * inputDdot[4] - (xx[3] + xx[4] * xx[26] * inputDdot[3]) + xx
    [6] * inputDdot[5];
  xx[28] = xx[42] * inputDdot[3] - xx[16] + xx[18] * inputDdot[4] + xx[19] *
    inputDdot[5];
  xx[29] = xx[4] * xx[51] * inputDdot[3] - xx[10] + xx[20] * inputDdot[4] +
    inputDdot[5];
  pm_math_Quaternion_inverseXform_ra(xx + 12, xx + 27, xx + 34);
  xx[6] = 5.859767174269423e-16;
  xx[11] = 9.81;
  xx[19] = xx[11] * xx[46];
  xx[21] = xx[11] * xx[44];
  xx[64] = - 6.95258920558239e-7;
  xx[65] = 5.242912850533018e-9;
  xx[66] = - 0.01600730973823735;
  pm_math_Vector3_cross_ra(xx + 64, xx + 54, xx + 68);
  xx[71] = state[7];
  xx[72] = - state[6];
  xx[73] = state[8];
  pm_math_Quaternion_inverseXform_ra(xx + 43, xx + 71, xx + 74);
  xx[71] = xx[68] + xx[74];
  xx[72] = xx[69] + xx[75];
  xx[73] = xx[70] + xx[76];
  pm_math_Vector3_cross_ra(xx + 54, xx + 71, xx + 68);
  xx[71] = - xx[3];
  xx[72] = - xx[16];
  xx[73] = - xx[10];
  pm_math_Vector3_cross_ra(xx + 64, xx + 71, xx + 77);
  pm_math_Vector3_cross_ra(xx + 54, xx + 74, xx + 71);
  xx[3] = xx[45] * xx[45];
  xx[10] = xx[44] * xx[46];
  xx[16] = xx[43] * xx[45];
  xx[74] = - xx[33];
  xx[75] = xx[42];
  xx[76] = xx[52];
  pm_math_Vector3_cross_ra(xx + 64, xx + 74, xx + 80);
  xx[74] = xx[17];
  xx[75] = xx[18];
  xx[76] = xx[20];
  pm_math_Vector3_cross_ra(xx + 64, xx + 74, xx + 83);
  xx[17] = (xx[43] * xx[19] + xx[45] * xx[21]) * xx[4] + xx[68] + xx[77] - xx[71]
    - xx[4] * xx[26] * inputDdot[0] + (xx[41] - (xx[3] + xx[39]) * xx[4]) *
    inputDdot[1] + xx[4] * (xx[10] - xx[16]) * inputDdot[2] + xx[80] *
    inputDdot[3] + xx[83] * inputDdot[4] + 6.433354441336066e-9 * inputDdot[5];
  pm_math_Vector3_cross_ra(xx + 27, xx + 61, xx + 24);
  xx[18] = xx[69] + xx[78] - xx[72] + xx[42] * inputDdot[0] + xx[4] * (xx[23] -
    xx[22]) * inputDdot[1] + xx[4] * (xx[49] + xx[50]) * inputDdot[2] + xx[81] *
    inputDdot[3] + xx[84] * inputDdot[4] + 6.913296889641965e-7 * inputDdot[5] -
    (xx[46] * xx[19] + xx[44] * xx[21]) * xx[4] + xx[11];
  xx[11] = xx[4] * (xx[45] * xx[19] - xx[43] * xx[21]) + xx[70] + xx[79] - xx[73]
    + xx[4] * xx[51] * inputDdot[0] + xx[4] * (xx[16] + xx[10]) * inputDdot[1] +
    (xx[41] - (xx[40] + xx[3]) * xx[4]) * inputDdot[2] + xx[82] * inputDdot[3] +
    xx[85] * inputDdot[4] - 5.299239960355089e-14 * inputDdot[5];
  xx[19] = xx[17] + xx[24];
  xx[20] = xx[18] + xx[25];
  xx[21] = xx[11] + xx[26];
  pm_math_Quaternion_inverseXform_ra(xx + 12, xx + 19, xx + 22);
  xx[3] = 2.638704776895453e-7;
  xx[10] = xx[2] * state[14];
  xx[12] = cos(xx[10]);
  xx[13] = 0.7071068074797768;
  xx[14] = sin(xx[10]);
  xx[10] = xx[3] * xx[12] + xx[13] * xx[14];
  xx[15] = 9.030061734175021e-8;
  xx[16] = 0.7071067548932624;
  xx[19] = xx[15] * xx[12] + xx[16] * xx[14];
  xx[20] = xx[3] * xx[14] - xx[13] * xx[12];
  xx[21] = xx[15] * xx[14] - xx[16] * xx[12];
  xx[23] = xx[10];
  xx[24] = xx[19];
  xx[25] = xx[20];
  xx[26] = xx[21];
  pm_math_Quaternion_inverseXform_ra(xx + 23, xx + 54, xx + 39);
  xx[12] = xx[40] + state[15];
  xx[42] = xx[39];
  xx[43] = xx[12];
  xx[44] = xx[41];
  xx[45] = xx[37] * xx[39];
  xx[46] = xx[12] * xx[38];
  xx[47] = xx[58] * xx[41];
  pm_math_Vector3_cross_ra(xx + 42, xx + 45, xx + 39);
  xx[12] = xx[21] * xx[57];
  xx[14] = xx[57] * xx[19];
  xx[41] = - (6.862628603528332e-8 + xx[4] * (xx[12] * xx[10] - xx[20] * xx[14]));
  xx[42] = 0.2250903713431784 - ((xx[21] * xx[12] + xx[14] * xx[19]) * xx[4] -
    xx[57]);
  xx[43] = 0.03174267352194981 + (xx[14] * xx[10] + xx[20] * xx[12]) * xx[4];
  pm_math_Vector3_cross_ra(xx + 54, xx + 41, xx + 19);
  pm_math_Vector3_cross_ra(xx + 54, xx + 19, xx + 44);
  pm_math_Quaternion_inverseXform_ra(xx + 23, xx + 44, xx + 19);
  pm_math_Quaternion_inverseXform_ra(xx + 23, xx + 27, xx + 44);
  pm_math_Vector3_cross_ra(xx + 27, xx + 41, xx + 46);
  xx[41] = xx[17] + xx[46];
  xx[42] = xx[18] + xx[47];
  xx[43] = xx[11] + xx[48];
  pm_math_Quaternion_inverseXform_ra(xx + 23, xx + 41, xx + 46);
  xx[10] = xx[2] * state[16];
  xx[12] = cos(xx[10]);
  xx[14] = sin(xx[10]);
  xx[10] = xx[5] * xx[12] + xx[1] * xx[14];
  xx[20] = xx[8] * xx[12] + xx[7] * xx[14];
  xx[21] = xx[1] * xx[12] - xx[5] * xx[14];
  xx[1] = xx[7] * xx[12] - xx[8] * xx[14];
  xx[23] = - xx[10];
  xx[24] = - xx[20];
  xx[25] = xx[21];
  xx[26] = xx[1];
  pm_math_Quaternion_inverseXform_ra(xx + 23, xx + 54, xx + 41);
  xx[5] = xx[42] + state[17];
  xx[47] = xx[41];
  xx[48] = xx[5];
  xx[49] = xx[43];
  xx[50] = xx[37] * xx[41];
  xx[51] = xx[5] * xx[38];
  xx[52] = xx[58] * xx[43];
  pm_math_Vector3_cross_ra(xx + 47, xx + 50, xx + 41);
  xx[5] = xx[57] * xx[20];
  xx[7] = xx[1] * xx[57];
  xx[47] = - (0.2250903195760352 + xx[4] * (xx[21] * xx[5] - xx[7] * xx[10]));
  xx[48] = - (1.005658294890382e-7 + (xx[1] * xx[7] + xx[5] * xx[20]) * xx[4] -
              xx[57]);
  xx[49] = 0.03874274551335916 + (xx[5] * xx[10] + xx[21] * xx[7]) * xx[4];
  pm_math_Vector3_cross_ra(xx + 54, xx + 47, xx + 50);
  pm_math_Vector3_cross_ra(xx + 54, xx + 50, xx + 59);
  pm_math_Quaternion_inverseXform_ra(xx + 23, xx + 59, xx + 50);
  pm_math_Quaternion_inverseXform_ra(xx + 23, xx + 27, xx + 51);
  pm_math_Vector3_cross_ra(xx + 27, xx + 47, xx + 59);
  xx[47] = xx[17] + xx[59];
  xx[48] = xx[18] + xx[60];
  xx[49] = xx[11] + xx[61];
  pm_math_Quaternion_inverseXform_ra(xx + 23, xx + 47, xx + 59);
  xx[1] = xx[2] * state[18];
  xx[2] = sin(xx[1]);
  xx[5] = cos(xx[1]);
  xx[1] = xx[3] * xx[2] - xx[13] * xx[5];
  xx[7] = xx[15] * xx[2] - xx[16] * xx[5];
  xx[8] = xx[13] * xx[2] + xx[3] * xx[5];
  xx[3] = xx[15] * xx[5] + xx[16] * xx[2];
  xx[12] = xx[1];
  xx[13] = xx[7];
  xx[14] = - xx[8];
  xx[15] = - xx[3];
  pm_math_Quaternion_inverseXform_ra(xx + 12, xx + 54, xx + 23);
  xx[2] = xx[24] + state[19];
  xx[47] = xx[23];
  xx[48] = xx[2];
  xx[49] = xx[25];
  xx[60] = xx[37] * xx[23];
  xx[61] = xx[2] * xx[38];
  xx[62] = xx[58] * xx[25];
  pm_math_Vector3_cross_ra(xx + 47, xx + 60, xx + 23);
  xx[2] = xx[57] * xx[7];
  xx[5] = xx[3] * xx[57];
  xx[47] = 1.523626214063616e-7 - xx[4] * (xx[8] * xx[2] - xx[5] * xx[1]);
  xx[48] = - (0.22509035338258 + (xx[3] * xx[5] + xx[2] * xx[7]) * xx[4] - xx[57]);
  xx[49] = 0.03674270700121652 + (xx[2] * xx[1] + xx[8] * xx[5]) * xx[4];
  pm_math_Vector3_cross_ra(xx + 54, xx + 47, xx + 1);
  pm_math_Vector3_cross_ra(xx + 54, xx + 1, xx + 60);
  pm_math_Quaternion_inverseXform_ra(xx + 12, xx + 60, xx + 1);
  pm_math_Quaternion_inverseXform_ra(xx + 12, xx + 27, xx + 2);
  pm_math_Vector3_cross_ra(xx + 27, xx + 47, xx + 53);
  xx[25] = xx[17] + xx[53];
  xx[26] = xx[18] + xx[54];
  xx[27] = xx[11] + xx[55];
  pm_math_Quaternion_inverseXform_ra(xx + 12, xx + 25, xx + 16);
  logVector[0] = state[0];
  logVector[1] = state[1];
  logVector[2] = state[2];
  logVector[3] = xx[0] * state[3];
  logVector[4] = xx[0] * state[4];
  logVector[5] = xx[0] * state[5];
  logVector[6] = state[6];
  logVector[7] = state[7];
  logVector[8] = state[8];
  logVector[9] = xx[0] * state[9];
  logVector[10] = xx[0] * state[10];
  logVector[11] = xx[0] * state[11];
  logVector[12] = xx[0] * state[12];
  logVector[13] = xx[0] * state[13];
  logVector[14] = xx[0] * state[14];
  logVector[15] = xx[0] * state[15];
  logVector[16] = xx[0] * state[16];
  logVector[17] = xx[0] * state[17];
  logVector[18] = xx[0] * state[18];
  logVector[19] = xx[0] * state[19];
  logVector[20] = inputDdot[0];
  logVector[21] = inputDdot[1];
  logVector[22] = inputDdot[2];
  logVector[23] = xx[0] * inputDdot[3];
  logVector[24] = xx[0] * inputDdot[4];
  logVector[25] = xx[0] * inputDdot[5];
  logVector[26] = xx[0] * ((input[8] - 1.0e-5 * state[13] - (xx[67] + xx[32] *
    xx[9])) / xx[38] - (xx[35] + xx[6] * xx[22]));
  logVector[27] = xx[0] * ((input[9] - (xx[40] + xx[32] * xx[19])) / xx[38] -
    (xx[45] + xx[6] * xx[46]));
  logVector[28] = xx[0] * ((input[6] - (xx[42] + xx[32] * xx[50])) / xx[38] -
    (xx[52] + xx[6] * xx[59]));
  logVector[29] = xx[0] * ((input[7] - (xx[24] + xx[32] * xx[1])) / xx[38] -
    (xx[3] + xx[6] * xx[16]));
  errorResult[0] = 0.0;
  return NULL;
}
