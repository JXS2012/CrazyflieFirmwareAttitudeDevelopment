/*
 * traj_planner.c
 *
 *  Created on: Mar 16, 2015
 *      Author: jianxin
 */

#include "better_matrix.h"
#include "better_traj_planner.h"
#include <math.h>

#include "stm32f10x_conf.h"
#include "FreeRTOS.h"
#include "task.h"

static Matrix traj_paramMatrix;
static Matrix traj_a0;
static float traj_T;
static float t0;

static float r = 1;
//static float omega = 0.785;
//static float omega3 = 0.4837;
static float omega = 1.5;
static float omega3 = 2.25*1.5;

void trajectory_initMatrix(Matrix paramMatrix, Matrix a0, float T)
{
	t0 = xTaskGetTickCount();
	traj_T = T;

	traj_a0 = a0;
	traj_paramMatrix = paramMatrix;

}

Matrix get_trajectory_rotationMatrix(float *azref)
{
	float t = (xTaskGetTickCount() - t0)*0.001;

	Matrix acc = initMatrix(3,1);
	/*
	Matrix tVector = initMatrix(1,3);
	*tVector.head = 1/6.*t*t*t;
	*(tVector.head + 1) = 0.5*t*t;
	*(tVector.head + 2) = t;
	matrixProductBetter(tVector, traj_paramMatrix);

	acc = vectorPlus(matrixProductBetter(tVector, traj_paramMatrix), traj_a0);
	*(acc.head+2) +=9.81;
	*/

	*acc.head = -r*omega*omega*sin(omega*t);
	*(acc.head+1) = -r*omega*omega*cos(omega*t);
	*(acc.head+2) = 9.81;

	*azref = *(acc.head+2) - 9.81;

	acc = normalizeMatrix(acc);

	float theta;

	theta = atan2(*acc.head, *(acc.head + 2));

	float ctheta = cos(theta);
	float stheta = sin(theta);
	float sphi = -*(acc.head + 1);
	float cphi = sqrt(1-sphi*sphi);

	if (*(acc.head + 2)*ctheta < 0) cphi = -cphi;

	Matrix D = initMatrix(3,3);
	*D.head = ctheta;
	*(D.head + 1) = sphi*stheta;
	*(D.head + 2) = *acc.head;
	*(D.head + 3) = 0;
	*(D.head + 4) = cphi;
	*(D.head + 5) = *(acc.head + 1);
	*(D.head + 6) = -stheta;
	*(D.head + 7) = sphi*ctheta;
	*(D.head + 8) = *(acc.head + 2);

	return D;

}

Matrix get_trajectory_angular_velocityMatrix(Matrix invR, float force2)
{
	float t = (xTaskGetTickCount() - t0)*0.001;

	Matrix jerk = initMatrix(3,1);
/*
	Matrix tVector = initMatrix(1,3);
	*tVector.head = 1/2.*t*t;
	*(tVector.head + 1) = t;
	*(tVector.head + 2) = 1;
	jerk = matrixProductBetter(tVector, traj_paramMatrix);
*/


	*jerk.head = -r*omega3*cos(omega*t);
	*(jerk.head+1) = r*omega3*sin(omega*t);
	*(jerk.head+2) = 0;

	//compute the angular veloctiy needed to track the jerk
	return scalarMatrixProductBetter(invSqrtBetter(force2)/9.81, matrixProductBetter(invR, jerk));
}

