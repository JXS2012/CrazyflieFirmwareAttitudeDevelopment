/*
 * traj_planner.h
 *
 *  Created on: Mar 16, 2015
 *      Author: jianxin
 */

#ifndef TRAJ_PLANNER_H_
#define TRAJ_PLANNER_H_
#include "imu.h"

void trajectory_init(float paramMatrix[3][3], float a0[3], float T);

void get_trajectory_rotation(float D[3][3], float *azref);

void get_trajectory_angular_velocity(float invR[3][3], float force2, float w[3]);


#endif /* TRAJ_PLANNER_H_ */
