/*
 * kalman.h
 *
 *  Created on: Mar 19, 2015
 *      Author: jianxin
 */

#ifndef KALMAN_H_
#define KALMAN_H_

void kalman_update(float xhat_k_1[2], float z_k[2],float u_k_1, float P_k_1[2][2],
		float xhat_k[2], float P_k[2][2]);

#endif /* KALMAN_H_ */
