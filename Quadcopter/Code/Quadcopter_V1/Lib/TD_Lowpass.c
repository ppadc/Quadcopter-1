/**
 * * * @author Tan Dat
 * @date 2023-6
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

	float ePow_Accel = 0.0609;
	float ePow_Gyro  = 0.9190;
//	float ePow_Magn	= 1-exp(-DELTA_T * 2 * PI * f3);

	output->Ax += (input.Ax - output->Ax) * ePow_Accel;
	output->Ay += (input.Ay - output->Ay) * ePow_Accel;
	output->Az += (input.Az - output->Az) * ePow_Accel;

//	output->Gx += (input.Gx - output->Gx) * ePow_Gyro;
//	output->Gy += (input.Gy - output->Gy) * ePow_Gyro;
//	output->Gz += (input.Gz - output->Gz) * ePow_Gyro;

//	output->Mx += (input.Mx - output->Mx) * ePow_Magn;
//	output->My += (input.My - output->My) * ePow_Magn;
//	output->Mz += (input.Mz - output->Mz) * ePow_Magn;

//	output->Ax = input.Ax;
//	output->Ay = input.Ay;
//	output->Az = input.Az;

	output->Gx = input.Gx;
	output->Gy = input.Gy;
	output->Gz = input.Gz;

	output->Mx = input.Mx;
	output->My = input.My;
	output->Mz = input.Mz;
}
//3. Average filter
double Average_filter(double value)
{
	static double arr[100];
	static uint8_t index = 0;
    arr[index] = value;
    index++;
    if (index > 99) index = 0;
    return Average_array(arr,100);

}
//4. Function to calculate the sum of the quartiles in the gill
double Sum_array(double arr[], uint8_t n)
{
    double sum = 0;
    for (int i = 0; i < n; i++) {
        sum += arr[i];
    }
    return sum;
}

//5. Declare a function to calculate the average of the elements in the array by dividing the sum by 100 and return the result
double Average_array(double arr[], uint8_t n)
{
    double sum = Sum_array(arr, n);
    return (double)sum / n;
}
