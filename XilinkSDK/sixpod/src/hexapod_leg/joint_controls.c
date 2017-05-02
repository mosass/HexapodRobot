/*
 * joint_controls.c
 *
 *  Created on: Feb 5, 2017
 *      Author: Phanomphon Yotchon
 */

#include "joint_controls.h"


/**************************** Torque and Load function *******************************/
/*
 * Enable Torque.
 */
void JointTorqueEnable(int Id) {
	CommPortWriteByte(Id, TORQUE_ENABLE, 1);
}

/*
 * Disable Torque.
 */
void JointTorqueDisable(int Id) {
	CommPortWriteByte(Id, TORQUE_ENABLE, 0);
	return;
}

/*
 * Set goal torque.
 */
void JointSetGoalTorque(int Id, int Torque) {
	CommPortWriteWord(Id, TORQUE_LIMIT, Torque);
	return;
}

/*
 * Get present load.
 */
int JointGetLoad(int Id) {
	return CommPortReadWord(Id, PRESENT_LOAD);
}

/**************************** Position function *******************************/
/*
 * Set goal position.
 */
void JointSetGoalPosition(int Id, int Position) {
	CommPortWriteWord(Id, GOAL_POSITION, Position);
	return;
}

/*
 * Set goal position in degree.
 */
void JointSetGoalPositionDeg(int Id, float Deg) {
	int position = (int) (Deg / 0.29f);
	CommPortWriteWord(Id, GOAL_POSITION, position);
}

/*
 * Get goal position.
 */
int JointGetGoalPosition(int Id) {
	return CommPortReadWord(Id, GOAL_POSITION);
}

/*
 * Get goal position in degree.
 */
float JointGetGoalPositionDeg(int Id){
	int position = CommPortReadWord(Id, GOAL_POSITION);
	return (float) position * 0.29f;
}

/*
 * Get present position.
 */
int JointGetPresentPosition(int Id) {
	return CommPortReadWord(Id, PRESENT_POSITION);
}

/*
 * Get present position in degree.
 */
float JointGetPresentPositionDeg(int Id){
	int position = CommPortReadWord(Id, PRESENT_POSITION);
	return (float) position * 0.29f;
}

/**************************** Speed function *******************************/
/*
 * Set moving speed
 */
void JointSetMovingSpeed(int Id, int Speed) {
	if(Speed < 20)
		Speed = 20;

	CommPortWriteWord(Id, MOVING_SPEED, Speed);
	return;
}

/*
 * Set moving speed in degree per second.
 */
void JointSetMovingSpeedDeg(int Id, float DegPerSec) {
	int speed = (int) ((DegPerSec * 60.0f) / (360.0f * 0.111f));
	JointSetMovingSpeed(Id, speed);
	return;
}

/*
 * Set moving speed in RPM.
 */
void JointSetMovingSpeedRPM(int Id, float RPM) {
	int speed = (int) (RPM / 0.111f);
	JointSetMovingSpeed(Id, speed);
	return;
}

/*
 * Get moving speed
 */
int JointGetMovingSpeed(int Id) {
	int speed = CommPortReadWord(Id, MOVING_SPEED);
	if(speed > 1023){
		speed -= 1023;
	}
	return speed;
}

/*
 * Get moving speed in degree per second.
 */
float JointGetMovingSpeedDeg(int Id) {
	int speed = JointGetMovingSpeed(Id);
	return (float)(speed * 0.111f * 360.0f) / 60.0f;
}

/*
 * Get moving speed in RPM.
 */
float JointGetMovingSpeedRPM(int Id) {
	int speed = JointGetMovingSpeed(Id);
	return (float) speed * 0.111f;
}

/*
 * Get present speed
 */
int JointGetPresentSpeed(int Id) {
	int speed = CommPortReadWord(Id, PRESENT_SPEED);
	if(speed > 1023){
		speed -= 1023;
	}
	return speed;
}

/*
 * Get present speed in degree per second.
 */
float JointGetPresentSpeedDeg(int Id) {
	int speed = JointGetPresentSpeed(Id);
	return (float)(speed * 0.111f * 360.0f) / 60.0f;
}

/*
 * Get present speed in RPM.
 */
float JointGetPresentSpeedRPM(int Id) {
	int speed = JointGetPresentSpeed(Id);
	return (float) speed * 0.111f;
}

/*
 * Check is moving.
 */
int JointIsMoving(int Id) {
	return CommPortReadByte(Id, MOVING);
}

/**************************** LED function *******************************/
/*
 * Turn on LED.
 */
void JointLedOn(int Id) {
	CommPortWriteByte(Id, AX_LED, 1);
	return;
}

/*
 * Turn off LED.
 */
void JointLedOff(int Id) {
	CommPortWriteByte(Id, AX_LED, 0);
	return;
}





