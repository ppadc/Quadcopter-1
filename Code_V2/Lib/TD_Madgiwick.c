/**
 * TD_Madgwick is a header file that contains the TD_Madgwick class. This class implements the Madgwick algorithm for 
 * sensor fusion of IMU data. The algorithm fuses accelerometer, gyroscope, and magnetometer data to estimate the 
 * orientation of the sensor. The TD_Madgwick class provides methods to set the sensor data, update the orientation 
 * estimate, and retrieve the orientation estimate.
 */
#include "TD_Madgwick.h"

/**
 * Multiplies two quaternions and returns the result.
 *
 * @param L The left quaternion to be multiplied.
 * @param R The right quaternion to be multiplied.
 *
 * @returns The product of the two quaternions.
 */
//1. Multiply two quaternions and return a copy of the result, prod = L * R
Quaternion_t quat_mult(Quaternion_t L, Quaternion_t R)
{
    Quaternion_t product;
    product.q1 = (L.q1 * R.q1) - (L.q2 * R.q2) - (L.q3 * R.q3) - (L.q4 * R.q4);
    product.q2 = (L.q1 * R.q2) + (L.q2 * R.q1) + (L.q3 * R.q4) - (L.q4 * R.q3);
    product.q3 = (L.q1 * R.q3) - (L.q2 * R.q4) + (L.q3 * R.q1) + (L.q4 * R.q2);
    product.q4 = (L.q1 * R.q4) + (L.q2 * R.q3) - (L.q3 * R.q2) + (L.q4 * R.q1);

    return product;
}
/**
 * Multiplies a quaternion by a scalar value.
 *
 * @param q A pointer to the quaternion to be multiplied.
 * @param scalar The scalar value to multiply the quaternion by.
 *
 * @returns None
 */
//2. Multiply a reference of a quaternion by a scalar, q = s*q
void quat_scalar(Quaternion_t *q, float scalar)
{
    q->q1 *= scalar;
    q->q2 *= scalar;
    q->q3 *= scalar;
    q->q4 *= scalar;
}
/**
 * Adds two quaternions and stores the result in a third quaternion.
 *
 * @param Sum Pointer to the resulting quaternion.
 * @param L The first quaternion to be added.
 * @param R The second quaternion to be added.
 *
 * @returns None
 */
//3. Adds two quaternions together and the sum is the pointer to another quaternion, Sum = L + R
void quat_add(Quaternion_t *Sum, Quaternion_t L, Quaternion_t R)
{
    Sum->q1 = L.q1 + R.q1;
    Sum->q2 = L.q2 + R.q2;
    Sum->q3 = L.q3 + R.q3;
    Sum->q4 = L.q4 + R.q4;
}
/**
 * Subtracts two quaternions and stores the result in a third quaternion.
 *
 * @param Sum Pointer to the resulting quaternion.
 * @param L The left operand quaternion.
 * @param R The right operand quaternion.
 *
 * @returns None
 */
//4. Subtracts two quaternions together and the sum is the pointer to another quaternion, sum = L - R
void quat_sub(Quaternion_t *Sum, Quaternion_t L, Quaternion_t R)
{
    Sum->q1 = L.q1 - R.q1;
    Sum->q2 = L.q2 - R.q2;
    Sum->q3 = L.q3 - R.q3;
    Sum->q4 = L.q4 - R.q4;
}
/**
 * Computes the conjugate of a quaternion.
 *
 * @param q The quaternion to compute the conjugate of.
 *
 * @returns The conjugate of the input quaternion.
 */
//5. the conjugate of a quaternion is it's imaginary component sign changed  q* = [s, -v] if q = [s, v]
Quaternion_t quat_conjugate(Quaternion_t q)
{
    q.q2 = -q.q2;
    q.q3 = -q.q3;
    q.q4 = -q.q4;
    return q;
}
/**
 * Computes the norm of a quaternion.
 *
 * @param q The quaternion to compute the norm of.
 *
 * @returns The norm of the quaternion.
 */
//6. norm of a quaternion is the same as a complex number
float quat_Norm(Quaternion_t q)
{
    return sqrt(q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3 + q.q4 * q.q4);
}
/**
 * Normalizes a quaternion.
 *
 * @param q A pointer to the quaternion to be normalized.
 *
 * @returns None
 */
//7. Normalizes pointer q by calling quat_Norm(q),
void quat_Normalization(Quaternion_t *q)
{
    float norm = quat_Norm(*q);
    q->q1 /= norm;
    q->q2 /= norm;
    q->q3 /= norm;
    q->q4 /= norm;
}
/**
 * Updates the orientation estimate using the Madgwick algorithm.
 *
 * @param DataStruct A pointer to the MPU6050 data structure.
 * @param q_est A pointer to the current orientation estimate as a quaternion.
 *
 * @returns None
 */
//8. Madgwick filter for MARG
void Madgwick_update(MPU6050_t *DataStruct, Quaternion_t *q_est)
{
    float ax = DataStruct->Ax;
    float ay = DataStruct->Ay;
    float az = DataStruct->Az;
    float gx = DataStruct->Gx;
    float gy = DataStruct->Gy;
    float gz = DataStruct->Gz;
    float mx = DataStruct->Mx;
    float my = DataStruct->My;
    float mz = DataStruct->Mz;
    // Variables and constants
    float F_g[3] = {0};    // eq(15/21/25) objective function for gravity
    float J_g[3][4] = {0}; // jacobian matrix for gravity
    float F_b[3] = {0};    // eq(15/21/29) objective function for magnetic
    float J_b[3][4] = {0}; // jacobian matrix for magnetic
    static Quaternion_t q_w_gradient_integral = {0};
    Quaternion_t q_est_prev = *q_est;
    Quaternion_t q_est_dot = {0}; // eq 42 and 43
    Quaternion_t gradient = {0};

    Quaternion_t q_a = {0, ax, ay, az}; // eq (24)
    if (quat_Norm(q_a) == 0)
        return;
    quat_Normalization(&q_a); // normalize the acceleration quaternion to be a unit quaternion
    // const Quaternion_t q_g_ref = {0, 0, 0, 1};// eq (23) not needed because I used eq 25 instead of eq 21

    Quaternion_t q_m = {0, mx, my, mz}; // eq (28)
    if (quat_Norm(q_m) == 0)
    {
        Madgwick_imu(DataStruct, q_est);
        return;
    }
    quat_Normalization(&q_m); // normalize the magnetic quaternion to be a unit quaternion
    // const Quaternion_t q_b_ref = {0, 0.99, 0, -0.13};
    //  Magnetic distortion compensation
    Quaternion_t h = quat_mult(q_est_prev, quat_mult(q_m, quat_conjugate(q_est_prev))); // eq(45) (Group 1)
    Quaternion_t b = {0, sqrt(h.q2 * h.q2 + h.q3 * h.q3), 0, h.q4};                     // {0, 0.99, 0, -0.13} // eq(46)

    Quaternion_t q_w = {0, gx, gy, gz}; // eq (10), places gyroscope readings in a quaternion

    // Compute the objective function for gravity, simplified to equation (25) due to the 0's in the acceleration reference quaternion
    F_g[0] = 2 * (q_est_prev.q2 * q_est_prev.q4 - q_est_prev.q1 * q_est_prev.q3) - q_a.q2;
    F_g[1] = 2 * (q_est_prev.q1 * q_est_prev.q2 + q_est_prev.q3 * q_est_prev.q4) - q_a.q3;
    F_g[2] = 2 * (0.5 - q_est_prev.q2 * q_est_prev.q2 - q_est_prev.q3 * q_est_prev.q3) - q_a.q4;

    // Compute the Jacobian matrix, equation (26), for gravity
    J_g[0][0] = -2 * q_est_prev.q3;
    J_g[0][1] = 2 * q_est_prev.q4;
    J_g[0][2] = -2 * q_est_prev.q1;
    J_g[0][3] = 2 * q_est_prev.q2;

    J_g[1][0] = 2 * q_est_prev.q2;
    J_g[1][1] = 2 * q_est_prev.q1;
    J_g[1][2] = 2 * q_est_prev.q4;
    J_g[1][3] = 2 * q_est_prev.q3;

    J_g[2][0] = 0;
    J_g[2][1] = -4 * q_est_prev.q2;
    J_g[2][2] = -4 * q_est_prev.q3;
    J_g[2][3] = 0;

    // Compute the objective function for magnetic, simplified to equation (29) due to the 0's in the magnetic reference quaternion
    F_b[0] = 2 * b.q2 * (0.5 - q_est_prev.q3 * q_est_prev.q3 - q_est_prev.q4 * q_est_prev.q4) + 2 * b.q4 * (q_est_prev.q2 * q_est_prev.q4 - q_est_prev.q1 * q_est_prev.q3) - q_m.q2;
    F_b[1] = 2 * b.q2 * (q_est_prev.q2 * q_est_prev.q3 - q_est_prev.q1 * q_est_prev.q4) + 2 * b.q4 * (q_est_prev.q1 * q_est_prev.q2 + q_est_prev.q3 * q_est_prev.q4) - q_m.q3;
    F_b[2] = 2 * b.q2 * (q_est_prev.q1 * q_est_prev.q3 + q_est_prev.q2 * q_est_prev.q4) + 2 * b.q4 * (0.5 - q_est_prev.q2 * q_est_prev.q2 - q_est_prev.q3 * q_est_prev.q3) - q_m.q4;

    // Compute the Jacobian matrix, equation (26), for magnetic

    J_b[0][0] = -2 * b.q4 * q_est_prev.q3;
    J_b[0][1] = 2 * b.q4 * q_est_prev.q4;
    J_b[0][2] = -4 * b.q2 * q_est_prev.q3 - 2 * b.q4 * q_est_prev.q1;
    J_b[0][3] = -4 * b.q2 * q_est_prev.q4 + 2 * b.q4 * q_est_prev.q2;

    J_b[1][0] = -2 * b.q2 * q_est_prev.q4 + 2 * b.q4 * q_est_prev.q2;
    J_b[1][1] = 2 * b.q2 * q_est_prev.q3 + 2 * b.q4 * q_est_prev.q1;
    J_b[1][2] = 2 * b.q2 * q_est_prev.q2 + 2 * b.q4 * q_est_prev.q4;
    J_b[1][3] = -2 * b.q2 * q_est_prev.q1 + 2 * b.q4 * q_est_prev.q3;

    J_b[2][0] = 2 * b.q2 * q_est_prev.q3;
    J_b[2][1] = 2 * b.q2 * q_est_prev.q4 - 4 * b.q4 * q_est_prev.q2;
    J_b[2][2] = 2 * b.q2 * q_est_prev.q1 - 4 * b.q4 * q_est_prev.q3;
    J_b[2][3] = 2 * b.q2 * q_est_prev.q2;

    // now computer the gradient, equation (20), gradient = J_g'*F_g + J_b'*F_b
    gradient.q1 = J_g[0][0] * F_g[0] + J_g[1][0] * F_g[1] + J_g[2][0] * F_g[2] + J_b[0][0] * F_b[0] + J_b[1][0] * F_b[1] + J_b[2][0] * F_b[2];
    gradient.q2 = J_g[0][1] * F_g[0] + J_g[1][1] * F_g[1] + J_g[2][1] * F_g[2] + J_b[0][1] * F_b[0] + J_b[1][1] * F_b[1] + J_b[2][1] * F_b[2];
    gradient.q3 = J_g[0][2] * F_g[0] + J_g[1][2] * F_g[1] + J_g[2][2] * F_g[2] + J_b[0][2] * F_b[0] + J_b[1][2] * F_b[1] + J_b[2][2] * F_b[2];
    gradient.q4 = J_g[0][3] * F_g[0] + J_g[1][3] * F_g[1] + J_g[2][3] * F_g[2] + J_b[0][3] * F_b[0] + J_b[1][3] * F_b[1] + J_b[2][3] * F_b[2];

    // Normalize the gradient, equation (44)
    quat_Normalization(&gradient);

    // Gyroscope bias drift compensation (Group 2)
    Quaternion_t q_w_gradient = quat_mult(quat_conjugate(q_est_prev), gradient);
    quat_scalar(&q_w_gradient, 2 * DELTA_T);
    quat_add(&q_w_gradient_integral, q_w_gradient_integral, q_w_gradient);
    quat_scalar(&q_w_gradient_integral, ZETA);
    quat_sub(&q_w, q_w, q_w_gradient_integral);
    // Orientation from angular rate
    quat_scalar(&q_w, 0.5);           // equation (12) dq/dt = (1/2)q*w
    q_w = quat_mult(q_est_prev, q_w); // equation (12)
    // quat_scalar(&q_w, deltaT);             // eq (13) integrates the angles velocity to position
    // quat_add(&q_w, q_w, q_est_prev);       // addition part of equation (13)

    // Combining
    quat_scalar(&gradient, BETA);        // multiply normalized gradient by beta
    quat_sub(&q_est_dot, q_w, gradient); // subtract above from q_w, the integrated gyro quaternion
    quat_scalar(&q_est_dot, DELTA_T);
    quat_add(q_est, q_est_prev, q_est_dot); // Integrate orientation rate to find position
    quat_Normalization(q_est);              // normalize the orientation of the estimate
                                            //(shown in diagram, plus always use unit quaternions for orientation)
}
/**
 * Computes the orientation of an IMU using the Madgwick algorithm.
 *
 * @param DataStruct A pointer to the MPU6050 data structure containing the accelerometer and gyroscope data.
 * @param q_est A pointer to the estimated quaternion representing the orientation of the IMU.
 *
 * @returns None
 */
//9. Madgwick filter for IMU
void Madgwick_imu(MPU6050_t *DataStruct, Quaternion_t *q_est)
{
    float ax = DataStruct->Ax;
    float ay = DataStruct->Ay;
    float az = DataStruct->Az;
    float gx = DataStruct->Gx;
    float gy = DataStruct->Gy;
    float gz = DataStruct->Gz;
    // Variables and constants
    float F_g[3] = {0};    // eq(15/21/25) objective function for gravity
    float J_g[3][4] = {0}; // jacobian matrix for gravity
    Quaternion_t q_est_prev = *q_est;
    Quaternion_t q_est_dot = {0}; // eq 42 and 43
    Quaternion_t gradient = {0};
    // const Quaternion_t q_g_ref = {0, 0, 0, 1};// eq (23) not needed because I used eq 25 instead of eq 21

    Quaternion_t q_a = {0, ax, ay, az}; // eq (24) raw acceleration values, needs to be normalized
    if (quat_Norm(q_a) == 0)
        return;
    quat_Normalization(&q_a); // normalize the acceleration quaternion to be a unit quaternion

    Quaternion_t q_w = {0, gx, gy, gz}; // equation (10), places gyroscope readings in a quaternion

    quat_scalar(&q_w, 0.5);           // equation (12) dq/dt = (1/2)q*w
    q_w = quat_mult(q_est_prev, q_w); // equation (12)

    // quat_scalar(&q_w, deltaT);             // eq (13) integrates the angles velocity to position
    // quat_add(&q_w, q_w, q_est_prev);       // addition part of equation (13)

    // Compute the objective function for gravity, equation(15), simplified to equation (25) due to the 0's in the acceleration reference quaternion
    F_g[0] = 2 * (q_est_prev.q2 * q_est_prev.q4 - q_est_prev.q1 * q_est_prev.q3) - q_a.q2;
    F_g[1] = 2 * (q_est_prev.q1 * q_est_prev.q2 + q_est_prev.q3 * q_est_prev.q4) - q_a.q3;
    F_g[2] = 2 * (0.5 - q_est_prev.q2 * q_est_prev.q2 - q_est_prev.q3 * q_est_prev.q3) - q_a.q4;

    // Compute the Jacobian matrix, equation (26), for gravity
    J_g[0][0] = -2 * q_est_prev.q3;
    J_g[0][1] = 2 * q_est_prev.q4;
    J_g[0][2] = -2 * q_est_prev.q1;
    J_g[0][3] = 2 * q_est_prev.q2;

    J_g[1][0] = 2 * q_est_prev.q2;
    J_g[1][1] = 2 * q_est_prev.q1;
    J_g[1][2] = 2 * q_est_prev.q4;
    J_g[1][3] = 2 * q_est_prev.q3;

    J_g[2][0] = 0;
    J_g[2][1] = -4 * q_est_prev.q2;
    J_g[2][2] = -4 * q_est_prev.q3;
    J_g[2][3] = 0;

    // now computer the gradient, equation (20), gradient = J_g'*F_g
    gradient.q1 = J_g[0][0] * F_g[0] + J_g[1][0] * F_g[1] + J_g[2][0] * F_g[2];
    gradient.q2 = J_g[0][1] * F_g[0] + J_g[1][1] * F_g[1] + J_g[2][1] * F_g[2];
    gradient.q3 = J_g[0][2] * F_g[0] + J_g[1][2] * F_g[1] + J_g[2][2] * F_g[2];
    gradient.q4 = J_g[0][3] * F_g[0] + J_g[1][3] * F_g[1] + J_g[2][3] * F_g[2];

    // Normalize the gradient, equation (44)
    quat_Normalization(&gradient);
    // Combining
    quat_scalar(&gradient, BETA);        // multiply normalized gradient by beta
    quat_sub(&q_est_dot, q_w, gradient); // subtract above from q_w, the integrated gyro quaternion
    quat_scalar(&q_est_dot, DELTA_T);
    quat_add(q_est, q_est_prev, q_est_dot); // Integrate orientation rate to find position
    quat_Normalization(q_est);              // normalize the orientation of the estimate
                                            //(shown in diagram, plus always use unit quaternions for orientation)
}
/**
 * Converts a quaternion to Euler angles.
 *
 * @param q The quaternion to convert.
 * @param Angle A pointer to an Euler_t struct to store the resulting Euler angles.
 *
 * @returns None
 */
//10. Calculator Euler angle from quaternions
void Quat2Angle(Quaternion_t q, Euler_t *Angle)
{
    Angle->roll = atan2(2 * (q.q1 * q.q2 + q.q3 * q.q4), q.q1 * q.q1 - q.q2 * q.q2 - q.q3 * q.q3 + q.q4 * q.q4);
    Angle->pitch = asin(2 * (q.q1 * q.q3 - q.q2 * q.q4));
    Angle->yaw = atan2(2 * (q.q1 * q.q4 + q.q2 * q.q3), q.q1 * q.q1 + q.q2 * q.q2 - q.q3 * q.q3 - q.q4 * q.q4);
    if (Angle->pitch == PI / 2)
    {
        Angle->roll = 0;
        Angle->yaw = -2 * atan2(q.q2, q.q1);
    }
    if (Angle->pitch == -PI / 2)
    {
        Angle->roll = 0;
        Angle->yaw = 2 * atan2(q.q2, q.q1);
    }
    Angle->roll *= RAD_TO_DEG;
    Angle->pitch *= RAD_TO_DEG;
    Angle->yaw *= RAD_TO_DEG;
}

