/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
#include "stm32f10x_conf.h"
#include "FreeRTOS.h"
#include "task.h"

#include <math.h>

#include "system.h"
#include "pm.h"
#include "stabilizer.h"
#include "commander.h"
#include "controller.h"
#include "sensfusion6.h"
#include "imu.h"
#include "motors.h"
#include "log.h"
#include "pid.h"
#include "ledseq.h"
#include "param.h"
#include "ms5611.h"

#include "better_matrix.h"
#include "better_attitude_controller.h"
#include "better_traj_planner.h"
#include "better_kalman.h"
#undef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#undef min
#define min(a,b) ((a) < (b) ? (a) : (b))

/**
 * Defines in what divided update rate should the attitude
 * control loop run relative the rate control loop.
 */
#define ATTITUDE_UPDATE_RATE_DIVIDER  2
#define FUSION_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER)) // 250hz

// Barometer/ Altitude hold stuff
#define ALTHOLD_UPDATE_RATE_DIVIDER  5 // 500hz/5 = 100hz for barometer measurements
#define ALTHOLD_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ALTHOLD_UPDATE_RATE_DIVIDER))   // 500hz

static Axis3f gyro; // Gyro axis data in deg/s
static Axis3f acc;  // Accelerometer axis data in mG
static Axis3f mag;  // Magnetometer axis data in testla

// Baro variables
static float temperature; // temp from barometer
static float pressure;    // pressure from barometer
static float aslRaw;  // raw asl


#define TRUNCATE_SINT16(out, in) (out = (in<INT16_MIN)?INT16_MIN:((in>INT16_MAX)?INT16_MAX:in) )

RPYType rollType;
RPYType pitchType;
RPYType yawType;

uint16_t actuatorThrust;
int16_t  actuatorRoll;
int16_t  actuatorPitch;
int16_t  actuatorYaw;

uint32_t motorPowerM4;
uint32_t motorPowerM2;
uint32_t motorPowerM1;
uint32_t motorPowerM3;

static bool isInit;

//My added variables
//Landing
static bool landing = FALSE;
static float tl = 0;
static float landingt0 = 0;

//Trajectory folllowing
static bool traj_init = FALSE;
static float force2 = 0;

Matrix acc_body;
Matrix ddot;
Matrix d;
Matrix D;
Matrix u;

//Controller
static float  uz;
static float cmd_actuation = 0;

//Kalman filter for z direction
static float u_k_1 = 0;
Matrix xhat_k_1;
Matrix z_k;
Matrix P_k_1;
Matrix xhat_k;
Matrix P_k;

static bool kalman_init = FALSE;
static float kalman_vz = 0;
static float kalman_z = 0;
static float ground_z = 0;
static float baro_ratio = 1;
static float vol = 0;

static void distributePower(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw);
static uint16_t limitThrust(int32_t value);
static void stabilizerTask(void* param);
static float constrain(float value, const float minVal, const float maxVal);
static float deadband(float value, const float threshold);

static void trajectory_attitude_controller();

static void init_trajectory();
static void trajectoryTracking(Matrix R);
static void trajectoryController(Matrix R);

static void init_landing();
static void landing_contoller();

static void kalman_filter_update();

void stabilizerInit(void)
 {
	if (isInit)
		return;

	motorsInit();
	imu6Init();
	sensfusion6Init();
	controllerInit();

	xTaskCreate(stabilizerTask, (const signed char * const )"STABILIZER",
			2*configMINIMAL_STACK_SIZE, NULL, /*Piority*/2, NULL);

	acc_body = initMatrix(3, 1);
	ddot = initMatrix(3, 1);
	d = initMatrix(3, 1);
	D = initMatrix(3, 3);
	u = initMatrix(3, 1);
	xhat_k_1 = initMatrix(2, 1);
	z_k = initMatrix(2, 1);
	P_k_1 = initMatrix(2, 2);
	xhat_k = initMatrix(2, 2);
	P_k = initMatrix(2, 2);

	isInit = TRUE;
}

bool stabilizerTest(void)
{
  bool pass = TRUE;

  pass &= motorsTest();
  pass &= imu6Test();
  pass &= sensfusion6Test();
  pass &= controllerTest();

  return pass;
}

static void stabilizerTask(void* param)
{
  uint32_t attitudeCounter = 0;
  uint32_t altHoldCounter = 0;
  uint32_t lastWakeTime;

  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount ();

  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(IMU_UPDATE_FREQ)); // 500Hz

    // Magnetometer not yet used more then for logging.
    imu9Read(&gyro, &acc, &mag);
    gyro.z /= 2;

    if (imu6IsCalibrated())
    {
      // 100HZ Kalman filter on z direction
      if (imuHasBarometer() && (++altHoldCounter >= ALTHOLD_UPDATE_RATE_DIVIDER))
      {
    	  kalman_filter_update();
    	  altHoldCounter = 0;
      }
      // 250HZ Trajectory and attitude controller
      if (++attitudeCounter >= ATTITUDE_UPDATE_RATE_DIVIDER)
      {
    	  trajectory_attitude_controller();
    	  attitudeCounter = 0;
      }

      actuatorRoll = *(u.head)*50;
      actuatorPitch = *(u.head + 1)*50;
      actuatorYaw = *(u.head + 2)*50;
      commanderGetThrust(&actuatorThrust);

      if (actuatorThrust > 0)
      {
    	  if (landing) landing = FALSE;
    	  if (!traj_init) init_trajectory();
    	  vol = pmGetBatteryVoltage();
    	  cmd_actuation = actuatorThrust*sqrt(force2)*(-5000./43000.*(vol-3.4)+1);
    	  distributePower(cmd_actuation, actuatorRoll, -actuatorPitch, -actuatorYaw);
      }
      else
      {
    	  if (traj_init && !landing)  init_landing();
    	  traj_init = FALSE;
      }

      if (landing)  landing_contoller();
    }
  }
}

static void kalman_filter_update()
{
	ms5611GetData(&pressure, &temperature, &aslRaw);
	if (kalman_init) {
		*z_k.head = baro_ratio * aslRaw;
		u_k_1 = *(acc_body.head + 2) * 9.81;

		kalman_updateMatrix(xhat_k_1, z_k, u_k_1, P_k_1, &xhat_k, &P_k);
		int i, j;
		for (i = 0; i < 2; i++) {
			*(xhat_k_1.head + i) = *(xhat_k.head + i);
			for (j = 0; j < 2; j++)
				*(P_k_1.head + i*P_k_1.column + j) = *(P_k.head + i*P_k.column + j);
		}
		kalman_vz = *(xhat_k.head + 1);
		kalman_z = *(xhat_k.head);
	} else if (traj_init && aslRaw != 0) {
		ground_z = baro_ratio * aslRaw;
		*xhat_k_1.head = baro_ratio * aslRaw;
		u_k_1 = *(acc_body.head + 2) * 9.81;
		kalman_init = TRUE;
	}
}

static void trajectory_attitude_controller() {
	//update R, omega
	float R_value[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
	float omega_value[3] = { gyro.x, gyro.y, gyro.z };
	Matrix omega = initMatrixValue(3, 1, omega_value);
	sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z,
			FUSION_UPDATE_DT);
	sensfusion6GetR(R_value);

	// Compute acc in odom coordinates
	float acc_vector_value[3] = { acc.x, acc.y, acc.z };

	Matrix R = initMatrixValue(3, 3, R_value);
	Matrix acc_vector = initMatrixValue(3, 1, acc_vector_value);

	acc_body = matrixProductBetter(R, acc_vector);
	*(acc_body.head + 2) = *(acc_body.head + 2) - 1;

	// Trajectory Control
	trajectoryController(R);
	u = attitudeControlMatrix(R, D, d, ddot, omega);

}

static void landing_contoller()
{
	tl = (xTaskGetTickCount() - landingt0)*0.001;
	cmd_actuation = (41000-10000*tl)*(1 - 0.1*kalman_vz);
	if (cmd_actuation < 20000)
		distributePower(0,0,0,0);
	else
	    distributePower(cmd_actuation, actuatorRoll, -actuatorPitch, -actuatorYaw);
}

static void init_landing() {
	int i, j;
	for (i = 0; i < 3; i++) {
		*(d.head + i) = 0;
		for (j = 0; j < 3; j++)
			if (i != j)
				*(D.head + i * D.column + j) = 0;
			else
				*(D.head + i * D.column + j) = 1;
	}
	landingt0 = xTaskGetTickCount();
	landing = TRUE;
}

static void init_trajectory()
{
	Matrix acc_0 = initMatrix(3,1);
	//float param_value[9] = {45,0,0,-45,0,0,15,0,0};
	float param_value[9] = {0,0,0,0,0,0,0,0,0};
	Matrix param = initMatrixValue(3,3,param_value);
	trajectory_initMatrix(param, acc_0, 2.);
	traj_init = TRUE;
}

static void trajectoryController(Matrix R)
{
    if (traj_init && !landing)	trajectoryTracking(R);
}

static void trajectoryTracking(Matrix R) {
	float azref = 0;
	D = get_trajectory_rotationMatrix(&azref);
	Matrix invR = transposeMatrixBetter(R);
	uz = (-1.0 * (*(acc_body.head + 2) * 9.81 - azref) - 0.1 * (kalman_vz - 0)
			- 0.2 * (kalman_z - ground_z - 0.8) + 9.81 + azref) / 9.81;
	force2 = *(acc_body.head) * (*(acc_body.head))
			+ *(acc_body.head + 1) * (*(acc_body.head + 1)) + uz * uz;
	d = get_trajectory_angular_velocityMatrix(invR, force2);
}

static void distributePower(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw)
{
#ifdef QUAD_FORMATION_X
  roll = roll >> 1;
  pitch = pitch >> 1;
  motorPowerM1 = limitThrust(thrust - roll + pitch + yaw);
  motorPowerM2 = limitThrust(thrust - roll - pitch - yaw);
  motorPowerM3 =  limitThrust(thrust + roll - pitch + yaw);
  motorPowerM4 =  limitThrust(thrust + roll + pitch - yaw);
#else // QUAD_FORMATION_NORMAL
  motorPowerM1 = limitThrust(thrust + pitch + yaw);
  motorPowerM2 = limitThrust(thrust - roll - yaw);
  motorPowerM3 =  limitThrust(thrust - pitch + yaw);
  motorPowerM4 =  limitThrust(thrust + roll - yaw);
#endif

  motorsSetRatio(MOTOR_M1, motorPowerM1);
  motorsSetRatio(MOTOR_M2, motorPowerM2);
  motorsSetRatio(MOTOR_M3, motorPowerM3);
  motorsSetRatio(MOTOR_M4, motorPowerM4);
}

static uint16_t limitThrust(int32_t value)
{
  if(value > UINT16_MAX)
  {
    value = UINT16_MAX;
  }
  else if(value < 0)
  {
    value = 0;
  }

  return (uint16_t)value;
}

// Constrain value between min and max
static float constrain(float value, const float minVal, const float maxVal)
{
  return min(maxVal, max(minVal,value));
}

// Deadzone
static float deadband(float value, const float threshold)
{
  if (fabs(value) < threshold)
  {
    value = 0;
  }
  else if (value > 0)
  {
    value -= threshold;
  }
  else if (value < 0)
  {
    value += threshold;
  }
  return value;
}

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &aslRaw)
LOG_ADD(LOG_FLOAT, y, &acc.z)
LOG_ADD(LOG_FLOAT, z, &kalman_vz)
LOG_ADD(LOG_FLOAT, zw, &kalman_z)
LOG_GROUP_STOP(acc)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &gyro.x)
LOG_ADD(LOG_FLOAT, y, &gyro.y)
LOG_ADD(LOG_FLOAT, z, &gyro.z)
LOG_GROUP_STOP(gyro)

LOG_GROUP_START(mag)
LOG_ADD(LOG_INT16, x, &actuatorRoll)
LOG_ADD(LOG_INT16, y, &actuatorPitch)
LOG_ADD(LOG_INT16, z, &actuatorYaw)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m4, &motorPowerM4)
LOG_ADD(LOG_INT32, m1, &motorPowerM1)
LOG_ADD(LOG_INT32, m2, &motorPowerM2)
LOG_ADD(LOG_INT32, m3, &motorPowerM3)
LOG_GROUP_STOP(motor)

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, aslRaw, &aslRaw)
LOG_ADD(LOG_FLOAT, temp, &temperature)
LOG_ADD(LOG_FLOAT, pressure, &pressure)
LOG_GROUP_STOP(baro)

