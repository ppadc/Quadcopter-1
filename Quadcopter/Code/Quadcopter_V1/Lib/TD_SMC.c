
/**
 * Contains functions for sliding control and calculating desired angles for a quadcopter.
 * * @author Tan Dat
 * @date 2023-6
 */

#include "TD_SMC.h"
/**
 * Computes the sliding control for a system.
 * @returns None
 */
/**
 * Implements sliding control for a single axis.
 *
 * @param U1 The input force.
 * @param x_2dot_SP The desired acceleration.
 * @param x_dot_SP The desired velocity.
 * @param x_SP The desired position.
 * @param x The current position.
 * @param x_dot The current velocity.
 * @param kd The derivative gain.
 * @param kp The proportional gain.
 * @param anpha The boundary layer thickness.
 * @param Ux The output force for the given axis.
 *
 * @returns None
 */
//1. SMC khai báo hàm bộ điều khiển trượt theo trục z
void sliding_control_z(double roll, double pitch, double z_2dot_SP, double z_dot_SP, double z_SP, double a0, double K, double z, double z_dot, double *Uz) {
  // khai báo các biến hệ thống
  const double g = 9.81; // gia tốc trọng trường
  const double m = 1.7; // khối lượng
  static double Sz = 0;
  static double VSz_dot = 0;
  // phương trình trạng thái
  // z_2dot = ((U1 * cos(pitch) * cos(roll)) / m) - g;

  // Chọn phương trình mặt trượt có dạng
  Sz = (z_dot_SP - z_dot) + a0 * (z_SP - z);

  // Chọn tiêu chí tốc độ hội tụ, trong đó K là hệ số dương Nếu K càng lớn thì độ hội tụ về 0 càng giảm
  VSz_dot = -K * copysign(1.0, Sz);

  // Thiết kế tín hiệu điều khiển U1
  *Uz = (m / (cos(roll) * cos(pitch))) * (z_2dot_SP - VSz_dot + g + a0 * (z_dot_SP - z_dot));
}

/**
 * Calculates the desired roll and pitch angles for a given set of velocity and yaw values.
 *
 * @param Ux The velocity in the x direction.
 * @param Uy The velocity in the y direction.
 * @param yaw The yaw angle.
 * @param roll_SP A pointer to the variable that will store the calculated roll angle.
 * @param pitch_SP A pointer to the variable that will store the calculated pitch angle.
 *
 * @returns None
 */
//3.tính góc quay mong muốn theo Ux Uy yaw
void calculate_desired_angle(double Ux, double Uy, double yaw, double *roll_SP, double *pitch_SP) 
{
  // tính toán góc quay mong muốn theo trục x (roll)
  *roll_SP = asin((Uy * cos(yaw) + Ux * sin(yaw)) / (cos(yaw) * cos(yaw) + sin(yaw) * sin(yaw)));

  // tính toán góc quay mong muốn theo trục y (pitch)
  *pitch_SP = asin((Uy * sin(yaw) - Ux * cos(yaw)) / (cos(*roll_SP) * (cos(yaw) * cos(yaw) + sin(yaw) * sin(yaw))));
}
