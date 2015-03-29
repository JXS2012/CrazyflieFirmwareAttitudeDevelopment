/*
 * kalman.h
 *
 *  Created on: Mar 19, 2015
 *      Author: jianxin
 */

#ifndef KALMAN_H_
#define KALMAN_H_
#include "better_matrix.h"

void kalman_updateMatrix(Matrix xhat_k_1, Matrix z_k,float u_k_1, Matrix P_k_1,
		Matrix *xhat_k, Matrix *P_k);

#endif /* KALMAN_H_ */
