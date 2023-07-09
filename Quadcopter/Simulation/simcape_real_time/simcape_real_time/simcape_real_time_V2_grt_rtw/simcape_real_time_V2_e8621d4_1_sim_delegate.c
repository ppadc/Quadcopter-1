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

void simcape_real_time_V2_e8621d4_1_resetSimStateVector(const void *mech, double
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

static void perturbSimJointPrimitiveState_0_0(double mag, double *state)
{
  state[0] = state[0] + mag;
}

static void perturbSimJointPrimitiveState_0_0v(double mag, double *state)
{
  state[0] = state[0] + mag;
  state[6] = state[6] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_0_1(double mag, double *state)
{
  state[1] = state[1] + mag;
}

static void perturbSimJointPrimitiveState_0_1v(double mag, double *state)
{
  state[1] = state[1] + mag;
  state[7] = state[7] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_0_2(double mag, double *state)
{
  state[2] = state[2] + mag;
}

static void perturbSimJointPrimitiveState_0_2v(double mag, double *state)
{
  state[2] = state[2] + mag;
  state[8] = state[8] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_0_3(double mag, double *state)
{
  state[3] = state[3] + mag;
}

static void perturbSimJointPrimitiveState_0_3v(double mag, double *state)
{
  state[3] = state[3] + mag;
  state[9] = state[9] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_0_4(double mag, double *state)
{
  state[4] = state[4] + mag;
}

static void perturbSimJointPrimitiveState_0_4v(double mag, double *state)
{
  state[4] = state[4] + mag;
  state[10] = state[10] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_0_5(double mag, double *state)
{
  state[5] = state[5] + mag;
}

static void perturbSimJointPrimitiveState_0_5v(double mag, double *state)
{
  state[5] = state[5] + mag;
  state[11] = state[11] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_1_0(double mag, double *state)
{
  state[12] = state[12] + mag;
}

static void perturbSimJointPrimitiveState_1_0v(double mag, double *state)
{
  state[12] = state[12] + mag;
  state[13] = state[13] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_2_0(double mag, double *state)
{
  state[14] = state[14] + mag;
}

static void perturbSimJointPrimitiveState_2_0v(double mag, double *state)
{
  state[14] = state[14] + mag;
  state[15] = state[15] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_3_0(double mag, double *state)
{
  state[16] = state[16] + mag;
}

static void perturbSimJointPrimitiveState_3_0v(double mag, double *state)
{
  state[16] = state[16] + mag;
  state[17] = state[17] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_4_0(double mag, double *state)
{
  state[18] = state[18] + mag;
}

static void perturbSimJointPrimitiveState_4_0v(double mag, double *state)
{
  state[18] = state[18] + mag;
  state[19] = state[19] - 0.875 * mag;
}

void simcape_real_time_V2_e8621d4_1_perturbSimJointPrimitiveState(const void
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
    perturbSimJointPrimitiveState_0_0(mag, state);
    break;

   case 1:
    perturbSimJointPrimitiveState_0_0v(mag, state);
    break;

   case 2:
    perturbSimJointPrimitiveState_0_1(mag, state);
    break;

   case 3:
    perturbSimJointPrimitiveState_0_1v(mag, state);
    break;

   case 4:
    perturbSimJointPrimitiveState_0_2(mag, state);
    break;

   case 5:
    perturbSimJointPrimitiveState_0_2v(mag, state);
    break;

   case 6:
    perturbSimJointPrimitiveState_0_3(mag, state);
    break;

   case 7:
    perturbSimJointPrimitiveState_0_3v(mag, state);
    break;

   case 8:
    perturbSimJointPrimitiveState_0_4(mag, state);
    break;

   case 9:
    perturbSimJointPrimitiveState_0_4v(mag, state);
    break;

   case 10:
    perturbSimJointPrimitiveState_0_5(mag, state);
    break;

   case 11:
    perturbSimJointPrimitiveState_0_5v(mag, state);
    break;

   case 12:
    perturbSimJointPrimitiveState_1_0(mag, state);
    break;

   case 13:
    perturbSimJointPrimitiveState_1_0v(mag, state);
    break;

   case 24:
    perturbSimJointPrimitiveState_2_0(mag, state);
    break;

   case 25:
    perturbSimJointPrimitiveState_2_0v(mag, state);
    break;

   case 36:
    perturbSimJointPrimitiveState_3_0(mag, state);
    break;

   case 37:
    perturbSimJointPrimitiveState_3_0v(mag, state);
    break;

   case 48:
    perturbSimJointPrimitiveState_4_0(mag, state);
    break;

   case 49:
    perturbSimJointPrimitiveState_4_0v(mag, state);
    break;
  }
}

void simcape_real_time_V2_e8621d4_1_perturbFlexibleBodyState(const void *mech,
  size_t stageIdx, double mag, boolean_T doPerturbVelocity, double *state)
{
  (void) mech;
  (void) stageIdx;
  (void) mag;
  (void) doPerturbVelocity;
  (void) state;
  switch (stageIdx * 2 + (doPerturbVelocity ? 1 : 0))
  {
  }
}

void simcape_real_time_V2_e8621d4_1_constructStateVector(const void *mech, const
  double *solverState, const double *u, const double *uDot, double
  *discreteState, double *fullState)
{
  (void) mech;
  (void) discreteState;
  fullState[0] = u[0];
  fullState[1] = u[1];
  fullState[2] = u[2];
  fullState[3] = u[3];
  fullState[4] = u[4];
  fullState[5] = u[5];
  fullState[6] = uDot[0];
  fullState[7] = uDot[1];
  fullState[8] = uDot[2];
  fullState[9] = uDot[3];
  fullState[10] = uDot[4];
  fullState[11] = uDot[5];
  fullState[12] = solverState[0];
  fullState[13] = solverState[1];
  fullState[14] = solverState[2];
  fullState[15] = solverState[3];
  fullState[16] = solverState[4];
  fullState[17] = solverState[5];
  fullState[18] = solverState[6];
  fullState[19] = solverState[7];
}

void simcape_real_time_V2_e8621d4_1_extractSolverStateVector(const void *mech,
  const double *fullState, double *solverState)
{
  (void) mech;
  solverState[0] = fullState[12];
  solverState[1] = fullState[13];
  solverState[2] = fullState[14];
  solverState[3] = fullState[15];
  solverState[4] = fullState[16];
  solverState[5] = fullState[17];
  solverState[6] = fullState[18];
  solverState[7] = fullState[19];
}

boolean_T simcape_real_time_V2_e8621d4_1_isPositionViolation(const void *mech,
  const RuntimeDerivedValuesBundle *rtdv, const int *eqnEnableFlags, const
  double *state, const int *modeVector)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  (void) state;
  (void) modeVector;
  return 0;
}

boolean_T simcape_real_time_V2_e8621d4_1_isVelocityViolation(const void *mech,
  const RuntimeDerivedValuesBundle *rtdv, const int *eqnEnableFlags, const
  double *state, const int *modeVector)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  (void) state;
  (void) modeVector;
  return 0;
}

PmfMessageId simcape_real_time_V2_e8621d4_1_projectStateSim(const void *mech,
  const RuntimeDerivedValuesBundle *rtdv, const int *eqnEnableFlags, const int
  *modeVector, double *state, void *neDiagMgr0)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  NeuDiagnosticManager *neDiagMgr = (NeuDiagnosticManager *) neDiagMgr0;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  (void) modeVector;
  (void) state;
  (void) neDiagMgr;
  return NULL;
}

void simcape_real_time_V2_e8621d4_1_computeConstraintError(const void *mech,
  const RuntimeDerivedValuesBundle *rtdv, const double *state, const int
  *modeVector, double *error)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
  (void) modeVector;
  (void) error;
}

void simcape_real_time_V2_e8621d4_1_resetModeVector(const void *mech, int
  *modeVector)
{
  (void) mech;
  (void) modeVector;
}

boolean_T simcape_real_time_V2_e8621d4_1_hasJointDisToNormModeChange(const void *
  mech, const int *prevModeVector, const int *modeVector)
{
  (void) mech;
  (void) prevModeVector;
  (void) modeVector;
  return 0;
}

PmfMessageId simcape_real_time_V2_e8621d4_1_performJointDisToNormModeChange(
  const void *mech, const RuntimeDerivedValuesBundle *rtdv, const int
  *eqnEnableFlags, const int *prevModeVector, const int *modeVector, const
  double *input, double *state, void *neDiagMgr0)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  NeuDiagnosticManager *neDiagMgr = (NeuDiagnosticManager *) neDiagMgr0;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  (void) prevModeVector;
  (void) modeVector;
  (void) input;
  (void) state;
  (void) neDiagMgr;
  return NULL;
}

void simcape_real_time_V2_e8621d4_1_onModeChangedCutJoints(const void *mech,
  const int *prevModeVector, const int *modeVector, double *state)
{
  (void) mech;
  (void) prevModeVector;
  (void) modeVector;
  (void) state;
}
