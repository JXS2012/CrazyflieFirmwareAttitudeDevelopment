/*
 * attitude_controller.h
 *
 *  Created on: Mar 13, 2015
 *      Author: jianxin
 */

#ifndef ATTITUDE_CONTROLLER_H_
#define ATTITUDE_CONTROLLER_H_

void attitudeControl(float R[3][3], float D[3][3], float d[3], float ddot[3], float w[3], float u[3]);

#endif /* ATTITUDE_CONTROLLER_H_ */
