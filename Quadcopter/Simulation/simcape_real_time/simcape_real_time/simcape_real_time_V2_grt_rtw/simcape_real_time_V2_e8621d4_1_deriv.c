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

PmfMessageId simcape_real_time_V2_e8621d4_1_compDerivs(const
  RuntimeDerivedValuesBundle *rtdv, const int *eqnEnableFlags, const double
  *state, const int *modeVector, const double *input, const double *inputDot,
  const double *inputDdot, const double *discreteState, double *deriv, double
  *errorResult, NeuDiagnosticManager *neDiagMgr)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[85];
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  (void) modeVector;
  (void) discreteState;
  (void) neDiagMgr;
  xx[0] = 0.4999998320075168;
  xx[1] = 0.5;
  xx[2] = xx[1] * state[0];
  xx[3] = cos(xx[2]);
  xx[4] = 0.5000002051767252;
  xx[5] = sin(xx[2]);
  xx[2] = xx[0] * xx[3] - xx[4] * xx[5];
  xx[6] = 0.499999917555661;
  xx[7] = 0.5000000452600188;
  xx[8] = xx[6] * xx[3] - xx[7] * xx[5];
  xx[9] = xx[0] * xx[5] + xx[4] * xx[3];
  xx[10] = xx[7] * xx[3] + xx[6] * xx[5];
  xx[11] = xx[2];
  xx[12] = xx[8];
  xx[13] = xx[9];
  xx[14] = xx[10];
  xx[3] = 2.0;
  xx[5] = 2.454648318984227e-7;
  xx[15] = xx[1] * input[5];
  xx[16] = sin(xx[15]);
  xx[17] = xx[5] * xx[16];
  xx[18] = 7.436862347702244e-8;
  xx[19] = xx[18] * xx[16];
  xx[20] = xx[17];
  xx[21] = xx[19];
  xx[22] = xx[16];
  xx[23] = 7.436850053016743e-8;
  xx[24] = xx[23] * xx[19];
  xx[25] = xx[24] + xx[16];
  xx[26] = xx[23] * xx[17];
  xx[27] = 5.008735751212025e-7;
  xx[28] = xx[27] * xx[16];
  xx[29] = xx[26] - xx[28];
  xx[30] = xx[27] * xx[19];
  xx[31] = xx[17] + xx[30];
  xx[32] = - xx[25];
  xx[33] = xx[29];
  xx[34] = xx[31];
  pm_math_Vector3_cross_ra(xx + 20, xx + 32, xx + 35);
  xx[20] = cos(xx[15]);
  xx[15] = (xx[3] * (xx[35] + xx[20] * xx[25]) - xx[27]) * inputDot[4];
  xx[21] = 0.707106604100972;
  xx[22] = xx[1] * input[3];
  xx[25] = cos(xx[22]);
  xx[32] = 1.130781874238157e-7;
  xx[33] = sin(xx[22]);
  xx[22] = 0.7071069582720672;
  xx[34] = 6.049167292397939e-8;
  xx[38] = xx[21] * xx[25] - xx[32] * xx[33];
  xx[39] = xx[22] * xx[33] - xx[34] * xx[25];
  xx[40] = - (xx[32] * xx[25] + xx[21] * xx[33]);
  xx[41] = - (xx[22] * xx[25] + xx[34] * xx[33]);
  xx[21] = xx[1] * input[4];
  xx[22] = sin(xx[21]);
  xx[42] = cos(xx[21]);
  xx[43] = - (xx[27] * xx[22]);
  xx[44] = xx[22];
  xx[45] = - (xx[23] * xx[22]);
  pm_math_Quaternion_compose_ra(xx + 38, xx + 42, xx + 46);
  xx[38] = xx[20];
  xx[39] = xx[17];
  xx[40] = xx[19];
  xx[41] = xx[16];
  pm_math_Quaternion_compose_ra(xx + 46, xx + 38, xx + 42);
  xx[21] = xx[42] * xx[45];
  xx[22] = xx[43] * xx[44];
  xx[25] = xx[21] + xx[22];
  xx[32] = xx[25] * xx[3];
  xx[33] = xx[32] * inputDot[3];
  xx[34] = xx[5] * inputDot[5];
  xx[38] = xx[45] * xx[45];
  xx[39] = xx[43] * xx[43];
  xx[40] = 1.0;
  xx[41] = (xx[38] + xx[39]) * xx[3] - xx[40];
  xx[46] = xx[41] * inputDot[3];
  xx[47] = (xx[40] + xx[3] * (xx[36] - xx[20] * xx[29])) * inputDot[4];
  xx[29] = xx[18] * inputDot[5];
  xx[48] = xx[42] * xx[43];
  xx[49] = xx[44] * xx[45];
  xx[50] = xx[48] - xx[49];
  xx[51] = xx[3] * xx[50];
  xx[52] = xx[51] * inputDot[3];
  xx[35] = (xx[3] * (xx[37] - xx[20] * xx[31]) - xx[23]) * inputDot[4];
  xx[53] = xx[15] - xx[33] + xx[34];
  xx[54] = xx[46] + xx[47] + xx[29];
  xx[55] = xx[52] + xx[35] + inputDot[5];
  pm_math_Quaternion_inverseXform_ra(xx + 11, xx + 53, xx + 56);
  xx[31] = xx[57] + state[1];
  xx[59] = xx[56];
  xx[60] = xx[31];
  xx[61] = xx[58];
  xx[36] = 1.720624935967864e-4;
  xx[37] = 1.666733335e-4;
  xx[57] = 1.286950159678639e-5;
  xx[62] = xx[36] * xx[56];
  xx[63] = xx[31] * xx[37];
  xx[64] = xx[57] * xx[58];
  pm_math_Vector3_cross_ra(xx + 59, xx + 62, xx + 65);
  xx[31] = 9.766669284693602e-20;
  xx[56] = 5.721451332149614e-3;
  xx[58] = xx[10] * xx[56];
  xx[59] = xx[56] * xx[8];
  xx[60] = 0.2250904050306244 - xx[3] * (xx[58] * xx[2] - xx[9] * xx[59]);
  xx[61] = 1.19047008300415e-7 - ((xx[10] * xx[58] + xx[59] * xx[8]) * xx[3] -
    xx[56]);
  xx[62] = 0.03674263500980687 + (xx[59] * xx[2] + xx[9] * xx[58]) * xx[3];
  pm_math_Vector3_cross_ra(xx + 53, xx + 60, xx + 8);
  pm_math_Vector3_cross_ra(xx + 53, xx + 8, xx + 63);
  pm_math_Quaternion_inverseXform_ra(xx + 11, xx + 63, xx + 8);
  xx[2] = xx[24] + xx[16];
  xx[63] = - xx[17];
  xx[64] = - xx[19];
  xx[65] = - xx[16];
  xx[9] = xx[26] - xx[28];
  xx[10] = xx[17] + xx[30];
  xx[67] = xx[2];
  xx[68] = - xx[9];
  xx[69] = - xx[10];
  pm_math_Vector3_cross_ra(xx + 63, xx + 67, xx + 70);
  xx[16] = (xx[2] * xx[20] + xx[70]) * xx[3] - xx[27];
  xx[26] = - xx[33];
  xx[27] = xx[46];
  xx[28] = xx[52];
  pm_math_Vector3_cross_ra(xx + 53, xx + 26, xx + 63);
  xx[26] = xx[34];
  xx[27] = xx[29];
  xx[28] = inputDot[5];
  xx[67] = xx[15];
  xx[68] = xx[47];
  xx[69] = xx[35];
  pm_math_Vector3_cross_ra(xx + 26, xx + 67, xx + 33);
  xx[2] = xx[63] + xx[33];
  xx[15] = xx[64] + xx[34];
  xx[17] = xx[40] + xx[3] * (xx[71] - xx[9] * xx[20]);
  xx[9] = xx[65] + xx[35];
  xx[19] = (xx[72] - xx[20] * xx[10]) * xx[3] - xx[23];
  xx[26] = xx[16] * inputDdot[4] - (xx[2] + xx[3] * xx[25] * inputDdot[3]) + xx
    [5] * inputDdot[5];
  xx[27] = xx[41] * inputDdot[3] - xx[15] + xx[17] * inputDdot[4] + xx[18] *
    inputDdot[5];
  xx[28] = xx[3] * xx[50] * inputDdot[3] - xx[9] + xx[19] * inputDdot[4] +
    inputDdot[5];
  pm_math_Quaternion_inverseXform_ra(xx + 11, xx + 26, xx + 33);
  xx[5] = 5.859767174269423e-16;
  xx[10] = 9.81;
  xx[18] = xx[10] * xx[45];
  xx[20] = xx[10] * xx[43];
  xx[63] = - 6.95258920558239e-7;
  xx[64] = 5.242912850533018e-9;
  xx[65] = - 0.01600730973823735;
  pm_math_Vector3_cross_ra(xx + 63, xx + 53, xx + 67);
  xx[70] = inputDot[1];
  xx[71] = - inputDot[0];
  xx[72] = inputDot[2];
  pm_math_Quaternion_inverseXform_ra(xx + 42, xx + 70, xx + 73);
  xx[70] = xx[67] + xx[73];
  xx[71] = xx[68] + xx[74];
  xx[72] = xx[69] + xx[75];
  pm_math_Vector3_cross_ra(xx + 53, xx + 70, xx + 67);
  xx[70] = - xx[2];
  xx[71] = - xx[15];
  xx[72] = - xx[9];
  pm_math_Vector3_cross_ra(xx + 63, xx + 70, xx + 76);
  pm_math_Vector3_cross_ra(xx + 53, xx + 73, xx + 70);
  xx[2] = xx[44] * xx[44];
  xx[9] = xx[43] * xx[45];
  xx[15] = xx[42] * xx[44];
  xx[73] = - xx[32];
  xx[74] = xx[41];
  xx[75] = xx[51];
  pm_math_Vector3_cross_ra(xx + 63, xx + 73, xx + 79);
  xx[73] = xx[16];
  xx[74] = xx[17];
  xx[75] = xx[19];
  pm_math_Vector3_cross_ra(xx + 63, xx + 73, xx + 82);
  xx[16] = (xx[42] * xx[18] + xx[44] * xx[20]) * xx[3] + xx[67] + xx[76] - xx[70]
    - xx[3] * xx[25] * inputDdot[0] + (xx[40] - (xx[2] + xx[38]) * xx[3]) *
    inputDdot[1] + xx[3] * (xx[9] - xx[15]) * inputDdot[2] + xx[79] * inputDdot
    [3] + xx[82] * inputDdot[4] + 6.433354441336066e-9 * inputDdot[5];
  pm_math_Vector3_cross_ra(xx + 26, xx + 60, xx + 23);
  xx[17] = xx[68] + xx[77] - xx[71] + xx[41] * inputDdot[0] + xx[3] * (xx[22] -
    xx[21]) * inputDdot[1] + xx[3] * (xx[48] + xx[49]) * inputDdot[2] + xx[80] *
    inputDdot[3] + xx[83] * inputDdot[4] + 6.913296889641965e-7 * inputDdot[5] -
    (xx[45] * xx[18] + xx[43] * xx[20]) * xx[3] + xx[10];
  xx[10] = xx[3] * (xx[44] * xx[18] - xx[42] * xx[20]) + xx[69] + xx[78] - xx[72]
    + xx[3] * xx[50] * inputDdot[0] + xx[3] * (xx[15] + xx[9]) * inputDdot[1] +
    (xx[40] - (xx[39] + xx[2]) * xx[3]) * inputDdot[2] + xx[81] * inputDdot[3] +
    xx[84] * inputDdot[4] - 5.299239960355089e-14 * inputDdot[5];
  xx[18] = xx[16] + xx[23];
  xx[19] = xx[17] + xx[24];
  xx[20] = xx[10] + xx[25];
  pm_math_Quaternion_inverseXform_ra(xx + 11, xx + 18, xx + 21);
  xx[2] = 2.638704776895453e-7;
  xx[9] = xx[1] * state[2];
  xx[11] = cos(xx[9]);
  xx[12] = 0.7071068074797768;
  xx[13] = sin(xx[9]);
  xx[9] = xx[2] * xx[11] + xx[12] * xx[13];
  xx[14] = 9.030061734175021e-8;
  xx[15] = 0.7071067548932624;
  xx[18] = xx[14] * xx[11] + xx[15] * xx[13];
  xx[19] = xx[2] * xx[13] - xx[12] * xx[11];
  xx[20] = xx[14] * xx[13] - xx[15] * xx[11];
  xx[22] = xx[9];
  xx[23] = xx[18];
  xx[24] = xx[19];
  xx[25] = xx[20];
  pm_math_Quaternion_inverseXform_ra(xx + 22, xx + 53, xx + 38);
  xx[11] = xx[39] + state[3];
  xx[41] = xx[38];
  xx[42] = xx[11];
  xx[43] = xx[40];
  xx[44] = xx[36] * xx[38];
  xx[45] = xx[11] * xx[37];
  xx[46] = xx[57] * xx[40];
  pm_math_Vector3_cross_ra(xx + 41, xx + 44, xx + 38);
  xx[11] = xx[20] * xx[56];
  xx[13] = xx[56] * xx[18];
  xx[40] = - (6.862628603528332e-8 + xx[3] * (xx[11] * xx[9] - xx[19] * xx[13]));
  xx[41] = 0.2250903713431784 - ((xx[20] * xx[11] + xx[13] * xx[18]) * xx[3] -
    xx[56]);
  xx[42] = 0.03174267352194981 + (xx[13] * xx[9] + xx[19] * xx[11]) * xx[3];
  pm_math_Vector3_cross_ra(xx + 53, xx + 40, xx + 18);
  pm_math_Vector3_cross_ra(xx + 53, xx + 18, xx + 43);
  pm_math_Quaternion_inverseXform_ra(xx + 22, xx + 43, xx + 18);
  pm_math_Quaternion_inverseXform_ra(xx + 22, xx + 26, xx + 43);
  pm_math_Vector3_cross_ra(xx + 26, xx + 40, xx + 45);
  xx[40] = xx[16] + xx[45];
  xx[41] = xx[17] + xx[46];
  xx[42] = xx[10] + xx[47];
  pm_math_Quaternion_inverseXform_ra(xx + 22, xx + 40, xx + 45);
  xx[9] = xx[1] * state[4];
  xx[11] = cos(xx[9]);
  xx[13] = sin(xx[9]);
  xx[9] = xx[4] * xx[11] + xx[0] * xx[13];
  xx[19] = xx[7] * xx[11] + xx[6] * xx[13];
  xx[20] = xx[0] * xx[11] - xx[4] * xx[13];
  xx[0] = xx[6] * xx[11] - xx[7] * xx[13];
  xx[22] = - xx[9];
  xx[23] = - xx[19];
  xx[24] = xx[20];
  xx[25] = xx[0];
  pm_math_Quaternion_inverseXform_ra(xx + 22, xx + 53, xx + 40);
  xx[4] = xx[41] + state[5];
  xx[46] = xx[40];
  xx[47] = xx[4];
  xx[48] = xx[42];
  xx[49] = xx[36] * xx[40];
  xx[50] = xx[4] * xx[37];
  xx[51] = xx[57] * xx[42];
  pm_math_Vector3_cross_ra(xx + 46, xx + 49, xx + 40);
  xx[4] = xx[56] * xx[19];
  xx[6] = xx[0] * xx[56];
  xx[46] = - (0.2250903195760352 + xx[3] * (xx[20] * xx[4] - xx[6] * xx[9]));
  xx[47] = - (1.005658294890382e-7 + (xx[0] * xx[6] + xx[4] * xx[19]) * xx[3] -
              xx[56]);
  xx[48] = 0.03874274551335916 + (xx[4] * xx[9] + xx[20] * xx[6]) * xx[3];
  pm_math_Vector3_cross_ra(xx + 53, xx + 46, xx + 49);
  pm_math_Vector3_cross_ra(xx + 53, xx + 49, xx + 58);
  pm_math_Quaternion_inverseXform_ra(xx + 22, xx + 58, xx + 49);
  pm_math_Quaternion_inverseXform_ra(xx + 22, xx + 26, xx + 50);
  pm_math_Vector3_cross_ra(xx + 26, xx + 46, xx + 58);
  xx[46] = xx[16] + xx[58];
  xx[47] = xx[17] + xx[59];
  xx[48] = xx[10] + xx[60];
  pm_math_Quaternion_inverseXform_ra(xx + 22, xx + 46, xx + 58);
  xx[0] = xx[1] * state[6];
  xx[1] = sin(xx[0]);
  xx[4] = cos(xx[0]);
  xx[0] = xx[2] * xx[1] - xx[12] * xx[4];
  xx[6] = xx[14] * xx[1] - xx[15] * xx[4];
  xx[7] = xx[12] * xx[1] + xx[2] * xx[4];
  xx[2] = xx[14] * xx[4] + xx[15] * xx[1];
  xx[11] = xx[0];
  xx[12] = xx[6];
  xx[13] = - xx[7];
  xx[14] = - xx[2];
  pm_math_Quaternion_inverseXform_ra(xx + 11, xx + 53, xx + 22);
  xx[1] = xx[23] + state[7];
  xx[46] = xx[22];
  xx[47] = xx[1];
  xx[48] = xx[24];
  xx[59] = xx[36] * xx[22];
  xx[60] = xx[1] * xx[37];
  xx[61] = xx[57] * xx[24];
  pm_math_Vector3_cross_ra(xx + 46, xx + 59, xx + 22);
  xx[1] = xx[56] * xx[6];
  xx[4] = xx[2] * xx[56];
  xx[46] = 1.523626214063616e-7 - xx[3] * (xx[7] * xx[1] - xx[4] * xx[0]);
  xx[47] = - (0.22509035338258 + (xx[2] * xx[4] + xx[1] * xx[6]) * xx[3] - xx[56]);
  xx[48] = 0.03674270700121652 + (xx[1] * xx[0] + xx[7] * xx[4]) * xx[3];
  pm_math_Vector3_cross_ra(xx + 53, xx + 46, xx + 0);
  pm_math_Vector3_cross_ra(xx + 53, xx + 0, xx + 59);
  pm_math_Quaternion_inverseXform_ra(xx + 11, xx + 59, xx + 0);
  pm_math_Quaternion_inverseXform_ra(xx + 11, xx + 26, xx + 1);
  pm_math_Vector3_cross_ra(xx + 26, xx + 46, xx + 52);
  xx[24] = xx[16] + xx[52];
  xx[25] = xx[17] + xx[53];
  xx[26] = xx[10] + xx[54];
  pm_math_Quaternion_inverseXform_ra(xx + 11, xx + 24, xx + 15);
  deriv[0] = state[1];
  deriv[1] = (input[8] - 1.0e-5 * state[1] - (xx[66] + xx[31] * xx[8])) / xx[37]
    - (xx[34] + xx[5] * xx[21]);
  deriv[2] = state[3];
  deriv[3] = (input[9] - (xx[39] + xx[31] * xx[18])) / xx[37] - (xx[44] + xx[5] *
    xx[45]);
  deriv[4] = state[5];
  deriv[5] = (input[6] - (xx[41] + xx[31] * xx[49])) / xx[37] - (xx[51] + xx[5] *
    xx[58]);
  deriv[6] = state[7];
  deriv[7] = (input[7] - (xx[23] + xx[31] * xx[0])) / xx[37] - (xx[2] + xx[5] *
    xx[15]);
  errorResult[0] = 0.0;
  return NULL;
}

PmfMessageId simcape_real_time_V2_e8621d4_1_numJacPerturbLoBounds(const
  RuntimeDerivedValuesBundle *rtdv, const int *eqnEnableFlags, const double
  *state, const int *modeVector, const double *input, const double *inputDot,
  const double *inputDdot, const double *discreteState, double *bounds, double
  *errorResult, NeuDiagnosticManager *neDiagMgr)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[1];
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  (void) state;
  (void) modeVector;
  (void) input;
  (void) inputDot;
  (void) inputDdot;
  (void) discreteState;
  (void) neDiagMgr;
  xx[0] = 1.0e-8;
  bounds[0] = xx[0];
  bounds[1] = xx[0];
  bounds[2] = xx[0];
  bounds[3] = xx[0];
  bounds[4] = xx[0];
  bounds[5] = xx[0];
  bounds[6] = xx[0];
  bounds[7] = xx[0];
  errorResult[0] = 0.0;
  return NULL;
}

PmfMessageId simcape_real_time_V2_e8621d4_1_numJacPerturbHiBounds(const
  RuntimeDerivedValuesBundle *rtdv, const int *eqnEnableFlags, const double
  *state, const int *modeVector, const double *input, const double *inputDot,
  const double *inputDdot, const double *discreteState, double *bounds, double
  *errorResult, NeuDiagnosticManager *neDiagMgr)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[2];
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  (void) state;
  (void) modeVector;
  (void) input;
  (void) inputDot;
  (void) inputDdot;
  (void) discreteState;
  (void) neDiagMgr;
  xx[0] = 1.0;
  xx[1] = +pmf_get_inf();
  bounds[0] = xx[0];
  bounds[1] = xx[1];
  bounds[2] = xx[0];
  bounds[3] = xx[1];
  bounds[4] = xx[0];
  bounds[5] = xx[1];
  bounds[6] = xx[0];
  bounds[7] = xx[1];
  errorResult[0] = 0.0;
  return NULL;
}
