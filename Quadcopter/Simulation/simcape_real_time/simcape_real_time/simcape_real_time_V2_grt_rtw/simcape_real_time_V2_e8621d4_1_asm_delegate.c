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
#include "sm_CTarget.h"

void simcape_real_time_V2_e8621d4_1_setTargets(const RuntimeDerivedValuesBundle *
  rtdv, CTarget *targets)
{
  (void) rtdv;
  (void) targets;
}

void simcape_real_time_V2_e8621d4_1_resetAsmStateVector(const void *mech, double
  *state)
{
  double xx[1];
  (void) mech;
  xx[0] = 0.0;
  state[0] = xx[0];
  state[1] = xx[0];
  state[2] = xx[0];
  state[3] = xx[0];
  state[4] = xx[0];
  state[5] = xx[0];
  state[6] = xx[0];
  state[7] = xx[0];
  state[8] = xx[0];
  state[9] = xx[0];
  state[10] = xx[0];
  state[11] = xx[0];
  state[12] = xx[0];
  state[13] = xx[0];
  state[14] = xx[0];
  state[15] = xx[0];
  state[16] = xx[0];
  state[17] = xx[0];
  state[18] = xx[0];
  state[19] = xx[0];
}

void simcape_real_time_V2_e8621d4_1_initializeTrackedAngleState(const void *mech,
  const RuntimeDerivedValuesBundle *rtdv, const int *modeVector, const double
  *motionData, double *state)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
  (void) modeVector;
  (void) motionData;
}

void simcape_real_time_V2_e8621d4_1_computeDiscreteState(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, double *state)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
}

void simcape_real_time_V2_e8621d4_1_adjustPosition(const void *mech, const
  double *dofDeltas, double *state)
{
  (void) mech;
  state[0] = state[0] + dofDeltas[0];
  state[1] = state[1] + dofDeltas[1];
  state[2] = state[2] + dofDeltas[2];
  state[3] = state[3] + dofDeltas[3];
  state[4] = state[4] + dofDeltas[4];
  state[5] = state[5] + dofDeltas[5];
  state[12] = state[12] + dofDeltas[6];
  state[14] = state[14] + dofDeltas[7];
  state[16] = state[16] + dofDeltas[8];
  state[18] = state[18] + dofDeltas[9];
}

static void perturbAsmJointPrimitiveState_0_0(double mag, double *state)
{
  state[0] = state[0] + mag;
}

static void perturbAsmJointPrimitiveState_0_0v(double mag, double *state)
{
  state[0] = state[0] + mag;
  state[6] = state[6] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_0_1(double mag, double *state)
{
  state[1] = state[1] + mag;
}

static void perturbAsmJointPrimitiveState_0_1v(double mag, double *state)
{
  state[1] = state[1] + mag;
  state[7] = state[7] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_0_2(double mag, double *state)
{
  state[2] = state[2] + mag;
}

static void perturbAsmJointPrimitiveState_0_2v(double mag, double *state)
{
  state[2] = state[2] + mag;
  state[8] = state[8] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_0_3(double mag, double *state)
{
  state[3] = state[3] + mag;
}

static void perturbAsmJointPrimitiveState_0_3v(double mag, double *state)
{
  state[3] = state[3] + mag;
  state[9] = state[9] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_0_4(double mag, double *state)
{
  state[4] = state[4] + mag;
}

static void perturbAsmJointPrimitiveState_0_4v(double mag, double *state)
{
  state[4] = state[4] + mag;
  state[10] = state[10] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_0_5(double mag, double *state)
{
  state[5] = state[5] + mag;
}

static void perturbAsmJointPrimitiveState_0_5v(double mag, double *state)
{
  state[5] = state[5] + mag;
  state[11] = state[11] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_1_0(double mag, double *state)
{
  state[12] = state[12] + mag;
}

static void perturbAsmJointPrimitiveState_1_0v(double mag, double *state)
{
  state[12] = state[12] + mag;
  state[13] = state[13] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_2_0(double mag, double *state)
{
  state[14] = state[14] + mag;
}

static void perturbAsmJointPrimitiveState_2_0v(double mag, double *state)
{
  state[14] = state[14] + mag;
  state[15] = state[15] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_3_0(double mag, double *state)
{
  state[16] = state[16] + mag;
}

static void perturbAsmJointPrimitiveState_3_0v(double mag, double *state)
{
  state[16] = state[16] + mag;
  state[17] = state[17] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_4_0(double mag, double *state)
{
  state[18] = state[18] + mag;
}

static void perturbAsmJointPrimitiveState_4_0v(double mag, double *state)
{
  state[18] = state[18] + mag;
  state[19] = state[19] - 0.875 * mag;
}

void simcape_real_time_V2_e8621d4_1_perturbAsmJointPrimitiveState(const void
  *mech, size_t stageIdx, size_t primIdx, double mag, boolean_T
  doPerturbVelocity, double *state)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) mag;
  (void) doPerturbVelocity;
  (void) state;
  switch ((stageIdx * 6 + primIdx) * 2 + (doPerturbVelocity ? 1 : 0))
  {
   case 0:
    perturbAsmJointPrimitiveState_0_0(mag, state);
    break;

   case 1:
    perturbAsmJointPrimitiveState_0_0v(mag, state);
    break;

   case 2:
    perturbAsmJointPrimitiveState_0_1(mag, state);
    break;

   case 3:
    perturbAsmJointPrimitiveState_0_1v(mag, state);
    break;

   case 4:
    perturbAsmJointPrimitiveState_0_2(mag, state);
    break;

   case 5:
    perturbAsmJointPrimitiveState_0_2v(mag, state);
    break;

   case 6:
    perturbAsmJointPrimitiveState_0_3(mag, state);
    break;

   case 7:
    perturbAsmJointPrimitiveState_0_3v(mag, state);
    break;

   case 8:
    perturbAsmJointPrimitiveState_0_4(mag, state);
    break;

   case 9:
    perturbAsmJointPrimitiveState_0_4v(mag, state);
    break;

   case 10:
    perturbAsmJointPrimitiveState_0_5(mag, state);
    break;

   case 11:
    perturbAsmJointPrimitiveState_0_5v(mag, state);
    break;

   case 12:
    perturbAsmJointPrimitiveState_1_0(mag, state);
    break;

   case 13:
    perturbAsmJointPrimitiveState_1_0v(mag, state);
    break;

   case 24:
    perturbAsmJointPrimitiveState_2_0(mag, state);
    break;

   case 25:
    perturbAsmJointPrimitiveState_2_0v(mag, state);
    break;

   case 36:
    perturbAsmJointPrimitiveState_3_0(mag, state);
    break;

   case 37:
    perturbAsmJointPrimitiveState_3_0v(mag, state);
    break;

   case 48:
    perturbAsmJointPrimitiveState_4_0(mag, state);
    break;

   case 49:
    perturbAsmJointPrimitiveState_4_0v(mag, state);
    break;
  }
}

void simcape_real_time_V2_e8621d4_1_computePosDofBlendMatrix(const void *mech,
  size_t stageIdx, size_t primIdx, const double *state, int partialType, double *
  matrix)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) state;
  (void) partialType;
  (void) matrix;
  switch ((stageIdx * 6 + primIdx))
  {
  }
}

void simcape_real_time_V2_e8621d4_1_computeVelDofBlendMatrix(const void *mech,
  size_t stageIdx, size_t primIdx, const double *state, int partialType, double *
  matrix)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) state;
  (void) partialType;
  (void) matrix;
  switch ((stageIdx * 6 + primIdx))
  {
  }
}

void simcape_real_time_V2_e8621d4_1_projectPartiallyTargetedPos(const void *mech,
  size_t stageIdx, size_t primIdx, const double *origState, int partialType,
  double *state)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) origState;
  (void) partialType;
  (void) state;
  switch ((stageIdx * 6 + primIdx))
  {
  }
}

void simcape_real_time_V2_e8621d4_1_propagateMotion(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const double *state, double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[82];
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  xx[0] = 0.707106604100972;
  xx[1] = 0.5;
  xx[2] = xx[1] * state[3];
  xx[3] = cos(xx[2]);
  xx[4] = 1.130781874238157e-7;
  xx[5] = sin(xx[2]);
  xx[2] = 0.7071069582720672;
  xx[6] = 6.049167292397939e-8;
  xx[7] = xx[0] * xx[3] - xx[4] * xx[5];
  xx[8] = xx[2] * xx[5] - xx[6] * xx[3];
  xx[9] = - (xx[4] * xx[3] + xx[0] * xx[5]);
  xx[10] = - (xx[2] * xx[3] + xx[6] * xx[5]);
  xx[0] = xx[1] * state[4];
  xx[2] = 5.008735751212025e-7;
  xx[3] = sin(xx[0]);
  xx[4] = 7.436850053016743e-8;
  xx[11] = cos(xx[0]);
  xx[12] = - (xx[2] * xx[3]);
  xx[13] = xx[3];
  xx[14] = - (xx[4] * xx[3]);
  pm_math_Quaternion_compose_ra(xx + 7, xx + 11, xx + 15);
  xx[0] = xx[1] * state[5];
  xx[3] = cos(xx[0]);
  xx[5] = 2.454648318984227e-7;
  xx[6] = sin(xx[0]);
  xx[0] = xx[5] * xx[6];
  xx[7] = 7.436862347702244e-8;
  xx[8] = xx[7] * xx[6];
  xx[9] = xx[3];
  xx[10] = xx[0];
  xx[11] = xx[8];
  xx[12] = xx[6];
  pm_math_Quaternion_compose_ra(xx + 15, xx + 9, xx + 19);
  xx[9] = - 6.95258920558239e-7;
  xx[10] = 5.242912850533018e-9;
  xx[11] = - 0.01600730973823735;
  pm_math_Quaternion_xform_ra(xx + 19, xx + 9, xx + 12);
  xx[15] = 0.4999998320075168;
  xx[16] = xx[1] * state[12];
  xx[17] = cos(xx[16]);
  xx[18] = 0.5000002051767252;
  xx[23] = sin(xx[16]);
  xx[16] = xx[15] * xx[17] - xx[18] * xx[23];
  xx[24] = 0.499999917555661;
  xx[25] = 0.5000000452600188;
  xx[26] = xx[24] * xx[17] - xx[25] * xx[23];
  xx[27] = xx[15] * xx[23] + xx[18] * xx[17];
  xx[28] = xx[25] * xx[17] + xx[24] * xx[23];
  xx[17] = 2.0;
  xx[23] = 5.721451332149614e-3;
  xx[29] = xx[28] * xx[23];
  xx[30] = xx[23] * xx[26];
  xx[31] = 0.2250904050306244 - xx[17] * (xx[29] * xx[16] - xx[27] * xx[30]);
  xx[32] = 1.19047008300415e-7 - ((xx[28] * xx[29] + xx[30] * xx[26]) * xx[17] -
    xx[23]);
  xx[33] = 0.03674263500980687 + (xx[30] * xx[16] + xx[27] * xx[29]) * xx[17];
  xx[29] = 2.638704776895453e-7;
  xx[30] = xx[1] * state[14];
  xx[34] = cos(xx[30]);
  xx[35] = 0.7071068074797768;
  xx[36] = sin(xx[30]);
  xx[30] = xx[29] * xx[34] + xx[35] * xx[36];
  xx[37] = 9.030061734175021e-8;
  xx[38] = 0.7071067548932624;
  xx[39] = xx[37] * xx[34] + xx[38] * xx[36];
  xx[40] = xx[29] * xx[36] - xx[35] * xx[34];
  xx[41] = xx[37] * xx[36] - xx[38] * xx[34];
  xx[34] = xx[41] * xx[23];
  xx[36] = xx[23] * xx[39];
  xx[42] = - (6.862628603528332e-8 + xx[17] * (xx[34] * xx[30] - xx[40] * xx[36]));
  xx[43] = 0.2250903713431784 - ((xx[41] * xx[34] + xx[36] * xx[39]) * xx[17] -
    xx[23]);
  xx[44] = 0.03174267352194981 + (xx[36] * xx[30] + xx[40] * xx[34]) * xx[17];
  xx[34] = xx[1] * state[16];
  xx[36] = cos(xx[34]);
  xx[45] = sin(xx[34]);
  xx[34] = xx[18] * xx[36] + xx[15] * xx[45];
  xx[46] = - xx[34];
  xx[47] = xx[25] * xx[36] + xx[24] * xx[45];
  xx[48] = - xx[47];
  xx[49] = xx[15] * xx[36] - xx[18] * xx[45];
  xx[15] = xx[24] * xx[36] - xx[25] * xx[45];
  xx[18] = xx[23] * xx[47];
  xx[24] = xx[15] * xx[23];
  xx[25] = - (0.2250903195760352 + xx[17] * (xx[49] * xx[18] - xx[24] * xx[34]));
  xx[36] = - (1.005658294890382e-7 + (xx[15] * xx[24] + xx[18] * xx[47]) * xx[17]
              - xx[23]);
  xx[45] = 0.03874274551335916 + (xx[18] * xx[34] + xx[49] * xx[24]) * xx[17];
  xx[18] = xx[1] * state[18];
  xx[1] = sin(xx[18]);
  xx[24] = cos(xx[18]);
  xx[18] = xx[29] * xx[1] - xx[35] * xx[24];
  xx[34] = xx[37] * xx[1] - xx[38] * xx[24];
  xx[47] = xx[35] * xx[1] + xx[29] * xx[24];
  xx[29] = - xx[47];
  xx[35] = xx[37] * xx[24] + xx[38] * xx[1];
  xx[1] = - xx[35];
  xx[24] = xx[23] * xx[34];
  xx[37] = xx[35] * xx[23];
  xx[38] = 1.523626214063616e-7 - xx[17] * (xx[47] * xx[24] - xx[37] * xx[18]);
  xx[50] = - (0.22509035338258 + (xx[35] * xx[37] + xx[24] * xx[34]) * xx[17] -
              xx[23]);
  xx[23] = 0.03674270700121652 + (xx[24] * xx[18] + xx[47] * xx[37]) * xx[17];
  xx[51] = xx[0];
  xx[52] = xx[8];
  xx[53] = xx[6];
  xx[24] = xx[4] * xx[8] + xx[6];
  xx[35] = xx[4] * xx[0] - xx[2] * xx[6];
  xx[6] = xx[0] + xx[2] * xx[8];
  xx[54] = - xx[24];
  xx[55] = xx[35];
  xx[56] = xx[6];
  pm_math_Vector3_cross_ra(xx + 51, xx + 54, xx + 57);
  xx[0] = (xx[17] * (xx[57] + xx[3] * xx[24]) - xx[2]) * state[10] - xx[17] *
    (xx[19] * xx[22] + xx[20] * xx[21]) * state[9] + xx[5] * state[11];
  xx[2] = 1.0;
  xx[5] = ((xx[22] * xx[22] + xx[20] * xx[20]) * xx[17] - xx[2]) * state[9] +
    (xx[2] + xx[17] * (xx[58] - xx[3] * xx[35])) * state[10] + xx[7] * state[11];
  xx[2] = xx[17] * (xx[19] * xx[20] - xx[21] * xx[22]) * state[9] + (xx[17] *
    (xx[59] - xx[3] * xx[6]) - xx[4]) * state[10] + state[11];
  xx[6] = xx[0];
  xx[7] = xx[5];
  xx[8] = xx[2];
  pm_math_Vector3_cross_ra(xx + 9, xx + 6, xx + 51);
  xx[9] = state[7];
  xx[10] = - state[6];
  xx[11] = state[8];
  pm_math_Quaternion_inverseXform_ra(xx + 19, xx + 9, xx + 54);
  xx[3] = xx[51] + xx[54];
  xx[4] = xx[52] + xx[55];
  xx[9] = xx[53] + xx[56];
  xx[51] = xx[16];
  xx[52] = xx[26];
  xx[53] = xx[27];
  xx[54] = xx[28];
  pm_math_Quaternion_inverseXform_ra(xx + 51, xx + 6, xx + 55);
  pm_math_Vector3_cross_ra(xx + 6, xx + 31, xx + 58);
  xx[61] = xx[58] + xx[3];
  xx[62] = xx[59] + xx[4];
  xx[63] = xx[60] + xx[9];
  pm_math_Quaternion_inverseXform_ra(xx + 51, xx + 61, xx + 58);
  xx[10] = 1.270417400644955e-18;
  xx[51] = xx[30];
  xx[52] = xx[39];
  xx[53] = xx[40];
  xx[54] = xx[41];
  pm_math_Quaternion_inverseXform_ra(xx + 51, xx + 6, xx + 61);
  pm_math_Vector3_cross_ra(xx + 6, xx + 42, xx + 64);
  xx[67] = xx[64] + xx[3];
  xx[68] = xx[65] + xx[4];
  xx[69] = xx[66] + xx[9];
  pm_math_Quaternion_inverseXform_ra(xx + 51, xx + 67, xx + 64);
  xx[51] = xx[46];
  xx[52] = xx[48];
  xx[53] = xx[49];
  xx[54] = xx[15];
  pm_math_Quaternion_inverseXform_ra(xx + 51, xx + 6, xx + 67);
  xx[70] = xx[25];
  xx[71] = xx[36];
  xx[72] = xx[45];
  pm_math_Vector3_cross_ra(xx + 6, xx + 70, xx + 73);
  xx[70] = xx[73] + xx[3];
  xx[71] = xx[74] + xx[4];
  xx[72] = xx[75] + xx[9];
  pm_math_Quaternion_inverseXform_ra(xx + 51, xx + 70, xx + 73);
  xx[51] = xx[18];
  xx[52] = xx[34];
  xx[53] = xx[29];
  xx[54] = xx[1];
  pm_math_Quaternion_inverseXform_ra(xx + 51, xx + 6, xx + 70);
  xx[76] = xx[38];
  xx[77] = xx[50];
  xx[78] = xx[23];
  pm_math_Vector3_cross_ra(xx + 6, xx + 76, xx + 79);
  xx[6] = xx[79] + xx[3];
  xx[7] = xx[80] + xx[4];
  xx[8] = xx[81] + xx[9];
  pm_math_Quaternion_inverseXform_ra(xx + 51, xx + 6, xx + 76);
  motionData[0] = xx[19];
  motionData[1] = xx[20];
  motionData[2] = xx[21];
  motionData[3] = xx[22];
  motionData[4] = state[1] - xx[12] + 1.4;
  motionData[5] = - (1.400000000000001 + xx[13] + state[0]);
  motionData[6] = state[2] - xx[14] + 0.1;
  motionData[7] = xx[16];
  motionData[8] = xx[26];
  motionData[9] = xx[27];
  motionData[10] = xx[28];
  motionData[11] = xx[31];
  motionData[12] = xx[32];
  motionData[13] = xx[33];
  motionData[14] = xx[30];
  motionData[15] = xx[39];
  motionData[16] = xx[40];
  motionData[17] = xx[41];
  motionData[18] = xx[42];
  motionData[19] = xx[43];
  motionData[20] = xx[44];
  motionData[21] = xx[46];
  motionData[22] = xx[48];
  motionData[23] = xx[49];
  motionData[24] = xx[15];
  motionData[25] = xx[25];
  motionData[26] = xx[36];
  motionData[27] = xx[45];
  motionData[28] = xx[18];
  motionData[29] = xx[34];
  motionData[30] = xx[29];
  motionData[31] = xx[1];
  motionData[32] = xx[38];
  motionData[33] = xx[50];
  motionData[34] = xx[23];
  motionData[35] = xx[0];
  motionData[36] = xx[5];
  motionData[37] = xx[2];
  motionData[38] = xx[3];
  motionData[39] = xx[4];
  motionData[40] = xx[9];
  motionData[41] = xx[55];
  motionData[42] = xx[56] + state[13];
  motionData[43] = xx[57];
  motionData[44] = xx[58] + xx[10] * state[13];
  motionData[45] = xx[59];
  motionData[46] = xx[60];
  motionData[47] = xx[61];
  motionData[48] = xx[62] + state[15];
  motionData[49] = xx[63];
  motionData[50] = xx[64] + xx[10] * state[15];
  motionData[51] = xx[65];
  motionData[52] = xx[66];
  motionData[53] = xx[67];
  motionData[54] = xx[68] + state[17];
  motionData[55] = xx[69];
  motionData[56] = xx[73] + xx[10] * state[17];
  motionData[57] = xx[74];
  motionData[58] = xx[75];
  motionData[59] = xx[70];
  motionData[60] = xx[71] + state[19];
  motionData[61] = xx[72];
  motionData[62] = xx[76] + xx[10] * state[19];
  motionData[63] = xx[77];
  motionData[64] = xx[78];
}

size_t simcape_real_time_V2_e8621d4_1_computeAssemblyError(const void *mech,
  const RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, const int
  *modeVector, const double *motionData, double *error)
{
  (void) mech;
  (void)rtdv;
  (void) modeVector;
  (void) motionData;
  (void) error;
  switch (constraintIdx)
  {
  }

  return 0;
}

size_t simcape_real_time_V2_e8621d4_1_computeAssemblyJacobian(const void *mech,
  const RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, boolean_T
  forVelocitySatisfaction, const double *state, const int *modeVector, const
  double *motionData, double *J)
{
  (void) mech;
  (void) rtdv;
  (void) state;
  (void) modeVector;
  (void) forVelocitySatisfaction;
  (void) motionData;
  (void) J;
  switch (constraintIdx)
  {
  }

  return 0;
}

size_t simcape_real_time_V2_e8621d4_1_computeFullAssemblyJacobian(const void
  *mech, const RuntimeDerivedValuesBundle *rtdv, const double *state, const int *
  modeVector, const double *motionData, double *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
  (void) modeVector;
  (void) motionData;
  (void) J;
  return 0;
}

boolean_T simcape_real_time_V2_e8621d4_1_isInKinematicSingularity(const void
  *mech, const RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, const int
  *modeVector, const double *motionData)
{
  (void) mech;
  (void) rtdv;
  (void) modeVector;
  (void) motionData;
  switch (constraintIdx)
  {
  }

  return 0;
}

void simcape_real_time_V2_e8621d4_1_convertStateVector(const void *asmMech,
  const RuntimeDerivedValuesBundle *rtdv, const void *simMech, const double
  *asmState, const int *asmModeVector, const int *simModeVector, double
  *simState)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) asmMech;
  (void) rtdvd;
  (void) rtdvi;
  (void) simMech;
  (void) asmModeVector;
  (void) simModeVector;
  simState[0] = asmState[0];
  simState[1] = asmState[1];
  simState[2] = asmState[2];
  simState[3] = asmState[3];
  simState[4] = asmState[4];
  simState[5] = asmState[5];
  simState[6] = asmState[6];
  simState[7] = asmState[7];
  simState[8] = asmState[8];
  simState[9] = asmState[9];
  simState[10] = asmState[10];
  simState[11] = asmState[11];
  simState[12] = asmState[12];
  simState[13] = asmState[13];
  simState[14] = asmState[14];
  simState[15] = asmState[15];
  simState[16] = asmState[16];
  simState[17] = asmState[17];
  simState[18] = asmState[18];
  simState[19] = asmState[19];
}
