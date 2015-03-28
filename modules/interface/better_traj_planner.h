/*
 * traj_planner.h
 *
 *  Created on: Mar 16, 2015
 *      Author: jianxin
 */

#ifndef TRAJ_PLANNER_H_
#define TRAJ_PLANNER_H_
#include "imu.h"

void trajectory_initMatrix(Matrix paramMatrix, Matrix a0, float T);

Matrix get_trajectory_rotationMatrix(float *azref);

Matrix get_trajectory_angular_velocityMatrix(Matrix invR, float force2);


#endif /* TRAJ_PLANNER_H_ */
