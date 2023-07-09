
/**
 * This is a file for the TD_LQR , which implements the Linear Quadratic Regulator algorithm.
 * @author Tan Dat
 * @date 2023-6
 */
#include "TD_LQR.h"

/**
 * Computes the control input for a given state and control matrix.
 *
 * @param x An array of size 6 representing the current state.
 * @param K A 4x6 matrix representing the control matrix.
 * @param u An array of size 4 to store the computed control input.
 *
 * @returns None
 */
//1. LQR controller
void compute_control(double x[6], double K[4][6], double u[4])
{
    int m = 4, n = 6;
    // Compute u = -K * x
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
        {
            u[i] += K[i][j] * x[j];
        }
    }
}

