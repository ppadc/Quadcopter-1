/**
 * Header file for the TD LQR controller.
 *
 * This header file defines the function `compute_control` which computes the control input for a given state using the TD LQR algorithm.
 * 
 * The TD LQR algorithm is a type of linear quadratic regulator that uses temporal difference learning to update the cost-to-go function.
 * 
 * This header file includes the necessary dependencies and defines the function signature for `compute_control`.
 */
#ifndef TD_LQR_H // Kiểm tra xem FILE_NAME_H đã được định nghĩa chưa
#define TD_LQR_H // Nếu chưa thì định nghĩa nó

/* Includes ------------------------------------------------------------------*/
#include "define.h"
#include <math.h>

/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void compute_control(double x[6], double K[4][6], double u[4]);

#endif // Kết thúc phần định nghĩa
