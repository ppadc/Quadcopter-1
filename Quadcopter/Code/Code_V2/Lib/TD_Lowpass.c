/**
 * Applies a lowpass filter to a given input signal.
 *
 * @param in The input signal to be filtered.
 * @param filter A pointer to the lowpass filter struct.
 * @param iCutOffFrequency The cutoff frequency of the filter.
 *
 * @returns None
 */
#include "TD_Lowpass.h"

/**
 * Applies a low-pass filter to an input signal.
 *
 * @param in The input signal to be filtered.
 * @param filter A pointer to the low-pass filter struct.
 * @param iCutOffFrequency The cutoff frequency of the filter.
 *
 * @returns None
 */
//1. Lowpass filter
void Lowpass(float in,Lowpass_t *filter, float iCutOffFrequency)
{
	filter->ePow = (1-exp(-DELTA_T * 2 * PI * iCutOffFrequency));
	filter->out += (in - filter->out) * filter->ePow;
}
//2. Lowpass filter for Accel
void Lowpass4MPU(MPU6050_t input,MPU6050_t *output)
{

	static float ePow_Accel = 1-exp(-DELTA_T * 2 * PI * 2);
//	float ePow_Gyro 	=	1-exp(-DELTA_T * 2 * PI * 1.5);
//	float ePow_Magn	= 1-exp(-DELTA_T * 2 * PI * 1);

	output->Ax += (input.Ax - output->Ax) * ePow_Accel;
	output->Ay += (input.Ay - output->Ay) * ePow_Accel;
	output->Az += (input.Az - output->Az) * ePow_Accel;

//	output->Gx += (input.Gx - output->Gx) * ePow_Gyro;
//	output->Gy += (input.Gy - output->Gy) * ePow_Gyro;
//	output->Gz += (input.Gz - output->Gz) * ePow_Gyro;
//
//	output->Mx += (input.Mx - output->Mx) * ePow_Magn;
//	output->My += (input.My - output->My) * ePow_Magn;
//	output->Mz += (input.Mz - output->Mz) * ePow_Magn;

	output->Gx = input.Gx;
	output->Gy = input.Gy;
	output->Gz = input.Gz;

	output->Mx = input.Mx;
	output->My = input.My;
	output->Mz = input.Mz;
}
