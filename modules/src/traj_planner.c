/*
 * traj_planner.c
 *
 *  Created on: Mar 16, 2015
 *      Author: jianxin
 */

#include "matrix.h"
#include "traj_planner.h"
#include <math.h>

#include "stm32f10x_conf.h"
#include "FreeRTOS.h"
#include "task.h"

static float traj_paramMatrix[3][3];
static float traj_a0[3];
static float traj_T;
static float t0;

static float r = 1;
//static float omega = 0.785;
//static float omega3 = 0.4837;
static float omega = 1.5;
static float omega3 = 2.25*1.5;

void trajectory_init(float paramMatrix[3][3], float a0[3], float T)
{
	t0 = xTaskGetTickCount();
	traj_T = T;

	int i,j;

	for (i = 0; i < 3; i ++)
	{
		traj_a0[i] = a0[i];
		for (j = 0; j < 3; j ++)
			traj_paramMatrix[i][j] = paramMatrix[i][j];
	}
}

void get_trajectory_rotation(float D[3][3], float *azref)
{
	float t = (xTaskGetTickCount() - t0)*0.001;

	float acc[3] = {0,0,0};
	/*
	float pre_acc[3] = {0,0,0};
	float tVector[3] = {1/6.*t*t*t, 0.5*t*t, t};
	vectorMatrixProduct(tVector, traj_paramMatrix, pre_acc);

	vectorPlus(pre_acc, traj_a0, acc);
	acc[2] +=9.81;
	*/

	acc[0] = -r*omega*omega*sin(omega*t);
	acc[1] = -r*omega*omega*cos(omega*t);
	acc[2] = 9.81;

	*azref = acc[2] - 9.81;
	float unit_acc[3] = {0,0,0};
	normalizeVector(acc, unit_acc);

	float theta;

	theta = atan2(unit_acc[0], unit_acc[2]);

	float ctheta = cos(theta);
	float stheta = sin(theta);
	float sphi = -unit_acc[1];
	float cphi = sqrt(1-sphi*sphi);
	//if (ctheta > stheta) cphi= unit_acc[2]/ctheta; else cphi = unit_acc[0]/stheta;
	if (unit_acc[2]*ctheta < 0) cphi = -cphi;


	D[0][0] = ctheta;
	D[0][1] = sphi*stheta;
	//D[0][2] = cos(phi)*sin(theta);
	D[0][2] = unit_acc[0];
	D[1][0] = 0;
	D[1][1] = cphi;
	//D[1][2]	= -sin(phi);
	D[1][2] = unit_acc[1];
	D[2][0] = -stheta;
	D[2][1] = sphi*ctheta;
	//D[2][2] = cos(phi)*cos(theta);
	D[2][2] = unit_acc[2];

}

void get_trajectory_angular_velocity(float invR[3][3], float force2, float w[3])
{
	float t = (xTaskGetTickCount() - t0)*0.001;

	float jerk[3];
/*
	float tVector[3] = {0.5*t*t, t, 1};
	vectorMatrixProduct(tVector, traj_paramMatrix, jerk);
*/


	jerk[0] = -r*omega3*cos(omega*t);
	jerk[1] = r*omega3*sin(omega*t);
	jerk[2] = 0;

	//compute the angular veloctiy needed to track the jerk
	float pre_w[3] = {0,0,0};
	matrixVectorProduct(invR, jerk, pre_w);
	scalarVectorProduct(invSqrt(force2)/9.81, pre_w, w);
}

