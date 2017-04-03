/*
 * joint_controls.h
 *
 *  Created on: Feb 5, 2017
 *      Author: Phanomphon Yotchon
 */

#include "comm.h"

#ifndef SRC_LEGS_CONTROLS_JOINT_CONTROLS_H_
#define SRC_LEGS_CONTROLS_JOINT_CONTROLS_H_

// Dynamixel AX-12A Register
/** EEPROM AREA **/
#define MODEL_NUMBER             0
#define VERSION                  2
#define ID                       3
#define BAUD_RATE                4
#define RETURN_DELAY_TIME        5
#define CW_ANGLE_LIMIT           6
#define CCW_ANGLE_LIMIT          8
#define LIMIT_TEMPERATURE        11
#define DOWN_LIMIT_VOLTAGE       12
#define UP_LIMIT_VOLTAGE         13
#define MAX_TORQUE               14
#define RETURN_LEVEL             16
#define ALARM_LED                17
#define ALARM_SHUTDOWN           18
#define DOWN_CALIBRATION         20
#define UP_CALIBRATION           22

/** RAM AREA **/
#define TORQUE_ENABLE            24
#define AX_LED                   25
#define CW_COMPLIANCE_MARGIN     26
#define CCW_COMPLIANCE_MARGIN    27
#define CW_COMPLIANCE_SLOPE      28
#define CCW_COMPLIANCE_SLOPE     29
#define GOAL_POSITION            30
#define MOVING_SPEED             32
#define TORQUE_LIMIT             34
#define PRESENT_POSITION         36
#define PRESENT_SPEED            38
#define PRESENT_LOAD             40
#define PRESENT_VOLTAGE          42
#define PRESENT_TEMPERATURE      43
#define REGISTERED_INSTRUCTION   44
#define MOVING                   46
#define LOCK                     47
#define PUNCH                    48

/************************** Function Prototypes ******************************/
void JointTorqueEnable(int Id);
void JointTorqueDisable(int Id);

void JointSetGoalTorque(int Id, int Torque);
int JointGetLoad(int Id);

void JointSetGoalPosition(int Id, int Position);
void JointSetGoalPositionDeg(int Id, float Deg);

int JointGetGoalPosition(int Id);
float JointGetGoalPositionDeg(int Id);

float JointGetPresentPositionDeg(int Id);
int JointGetPresentPosition(int Id);


void JointSetMovingSpeed(int Id, int Speed);
void JointSetMovingSpeedDeg(int Id, float DegPerSec);
void JointSetMovingSpeedRPM(int Id, float RPM);

int JointGetMovingSpeed(int Id);
float JointGetMovingSpeedDeg(int Id);
float JointGetMovingSpeedRPM(int Id);


int JointGetPresentSpeed(int Id);
float JointGetPresentSpeedRPM(int Id);
float JointGetPresentSpeedDeg(int Id);

float Moving(int Id);

int JointIsMoving(int Id);

void JointLedOn(int Id);
void JointLedOff(int Id);


#endif /* SRC_LEGS_CONTROLS_JOINT_CONTROLS_H_ */
