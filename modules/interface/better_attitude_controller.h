/*
 * attitude_controller.h
 *
 *  Created on: Mar 13, 2015
 *      Author: jianxin
 */

#ifndef ATTITUDE_CONTROLLER_H_
#define ATTITUDE_CONTROLLER_H_

Matrix attitudeControlMatrix(Matrix R, Matrix D, Matrix d, Matrix ddot, Matrix w);

#endif /* ATTITUDE_CONTROLLER_H_ */
