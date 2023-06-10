
/**
 * This is a header file for the TD_LQR class, which implements the Temporal Difference Linear Quadratic Regulator algorithm.
 * The TD_LQR algorithm is a reinforcement learning algorithm used to solve optimal control problems.
 * This header file contains the class definition and function prototypes for the TD_LQR class.
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
