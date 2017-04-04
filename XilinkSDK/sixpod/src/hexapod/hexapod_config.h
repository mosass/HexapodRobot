/*
 * hexapod_config.h
 *
 *  Created on: Apr 2, 2017
 *      Author: mosas
 */

#ifndef SRC_HEXAPOD_CONFIG_H_
#define SRC_HEXAPOD_CONFIG_H_

/*
 * Pin definition.
 */
#define MPU_INTR_PIN 	12

/*
 * Hexapod constants definition.
 */
#define COXA  		7.5
#define FEMUR 		5.6
#define TIBIA 		7.5

#define H_XBODY 	15.75
#define H_YBODY 	4.375

/*
 * Legs configuration
 */
#define LEG1_ID_A		10
#define LEG1_ID_B		11
#define LEG1_ID_C		12
#define LEG1_ID_INVA	false
#define LEG1_ID_INVB	false
#define LEG1_ID_INVC	true

#define LEG2_ID_A		13
#define LEG2_ID_B		14
#define LEG2_ID_C		15
#define LEG2_ID_INVA	false
#define LEG2_ID_INVB	false
#define LEG2_ID_INVC	true

#define LEG3_ID_A		16
#define LEG3_ID_B		17
#define LEG3_ID_C		18
#define LEG3_ID_INVA	false
#define LEG3_ID_INVB	false
#define LEG3_ID_INVC	true

#define LEG4_ID_A		1
#define LEG4_ID_B		2
#define LEG4_ID_C		3
#define LEG4_ID_INVA	true
#define LEG4_ID_INVB	true
#define LEG4_ID_INVC	false

#define LEG5_ID_A		4
#define LEG5_ID_B		5
#define LEG5_ID_C		6
#define LEG5_ID_INVA	true
#define LEG5_ID_INVB	true
#define LEG5_ID_INVC	false

#define LEG6_ID_A		7
#define LEG6_ID_B		8
#define LEG6_ID_C		9
#define LEG6_ID_INVA	true
#define LEG6_ID_INVB	true
#define LEG6_ID_INVC	false

/*
 * Hexapod configuration.
 */
#define INITIAL_FOOTTIP_X		0
#define INITIAL_FOOTTIP_Y		14
#define INITIAL_FOOTTIP_Z		0
#define INITIAL_FOOTTIP_Z_OFF	7

#define INITIAL_ROT_Y		0
#define INITIAL_ROT_P		0
#define INITIAL_ROT_R		0

#define INITIAL_TIME_STEP		0.5	// sec

#define ALLOW_ROT_ERROR 		1 	// +-deg
#define SETUP_TIME				2   // sec

#endif /* SRC_HEXAPOD_CONFIG_H_ */
