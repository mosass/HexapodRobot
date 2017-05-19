/*
 * task.h
 *
 *  Created on: Apr 13, 2017
 *      Author: Phanomphon Yotchon
 */

#ifndef SRC_TASKFUNCTION_H_
#define SRC_TASKFUNCTION_H_

TickType_t stepTime;

void prinfloat(float fval){
	int whole = fval;
	int thousandths = (fval - whole) * 1000;
	thousandths = thousandths > 0 ? thousandths : -thousandths;
	xil_printf("%d.%3d", whole, thousandths);
}

static void hexapodWalkingTask( void * );
static void hexapodLegGaitTask( void * );
static void hexapodMovingTask( void * );

static void hexapodMovingTask( void *pvParameters ){
	TickType_t dt;
	for(;;){
		dt = pdMS_TO_TICKS( Hexapod.dt * 1000 );
		if(Hexapod.readIMU()){
			if(Hexapod.improvePitch || Hexapod.improveRoll || Hexapod.improveYaw){
				// balance mode
				if(Hexapod.balance()){
					xil_printf("B\r\n");
//					Hexapod.updateGoalPosition();
				}
			}
			vTaskDelay( dt );
		}
	}
}

static void hexapodWalkingTask( void *pvParameters ){
	for(;;){
		TickType_t dt = pdMS_TO_TICKS( Hexapod.stepTime * 1000 );
		vTaskDelay( dt );
	}

	Trajectory3d tp1 = {-5, 14.0, 0, 0.5};
	Trajectory3d tp2 = {0, 14.0, 4, 0.25};
	Trajectory3d tp4 = {5, 14.0, 0, 0.25};

	Trajectory3d tpf1 = {-5, 14.0, 0, 0.5};
	Trajectory3d tpf4 = {5, 14.0, 0, 0.5};

	Hexapod.stepTime = 4;
	Hexapod.dt = 0.5;
//	Hexapod.bodyRotTarget.p = Rot3d::toReg(5.0);
	Hexapod.improvePitch = true;
	Hexapod.improveRoll = true;
	Hexapod.improveYaw = true;

	xQueueSend(xTrajQueue[0], (void *)&tpf1, 0UL);
	xQueueSend(xTrajQueue[2], (void *)&tpf1, 0UL);
	xQueueSend(xTrajQueue[4], (void *)&tpf1, 0UL);

	xQueueSend(xTrajQueue[1], (void *)&tpf4, 0UL);
	xQueueSend(xTrajQueue[3], (void *)&tpf4, 0UL);
	xQueueSend(xTrajQueue[5], (void *)&tpf4, 0UL);

	stepTime = pdMS_TO_TICKS( Hexapod.stepTime * 1000 );
	vTaskDelay( stepTime );

	for( ;; ){
		xil_printf("Gait\r\n");

		xQueueSend(xTrajQueue[0], (void *)&tp2, 0UL);
		xQueueSend(xTrajQueue[2], (void *)&tp2, 0UL);
		xQueueSend(xTrajQueue[4], (void *)&tp2, 0UL);

		xQueueSend(xTrajQueue[0], (void *)&tp4, 0UL);
		xQueueSend(xTrajQueue[2], (void *)&tp4, 0UL);
		xQueueSend(xTrajQueue[4], (void *)&tp4, 0UL);

		xQueueSend(xTrajQueue[1], (void *)&tp1, 0UL);
		xQueueSend(xTrajQueue[3], (void *)&tp1, 0UL);
		xQueueSend(xTrajQueue[5], (void *)&tp1, 0UL);

		ulTaskNotifyTake( pdFALSE, portMAX_DELAY );
		ulTaskNotifyTake( pdFALSE, portMAX_DELAY );
		ulTaskNotifyTake( pdFALSE, portMAX_DELAY );

		xQueueSend(xTrajQueue[0], (void *)&tp1, 0UL);
		xQueueSend(xTrajQueue[2], (void *)&tp1, 0UL);
		xQueueSend(xTrajQueue[4], (void *)&tp1, 0UL);

		xQueueSend(xTrajQueue[1], (void *)&tp2, 0UL);
		xQueueSend(xTrajQueue[3], (void *)&tp2, 0UL);
		xQueueSend(xTrajQueue[5], (void *)&tp2, 0UL);

		xQueueSend(xTrajQueue[1], (void *)&tp4, 0UL);
		xQueueSend(xTrajQueue[3], (void *)&tp4, 0UL);
		xQueueSend(xTrajQueue[5], (void *)&tp4, 0UL);

		ulTaskNotifyTake( pdFALSE, portMAX_DELAY );
		ulTaskNotifyTake( pdFALSE, portMAX_DELAY );
		ulTaskNotifyTake( pdFALSE, portMAX_DELAY );

		xil_printf("Gaited\r\n");
	}
}

//static void hexapodLegGaitTask( void *pvParameters ){
//
//}

static void hexapodLegGaitTask( void *pvParameters ){
	int id = (uint32_t) pvParameters;
	int legId = id + 1;
	bool footDown = false;

	for(;;){
		Trajectory3d targetPos, oldTargetPos;

		/* Wait without a timeout for data. */
		xQueueReceive(xTrajQueue[id], (void *) &targetPos, portMAX_DELAY );
		if(oldTargetPos.z > 0.0 && targetPos.z <= 0.0){
			footDown = true;
		}
		oldTargetPos = targetPos;

//		FootTip new_pos =
//
//		FootTip new_pos(targetPos.x, targetPos.y, targetPos.z);
		Hexapod.targetFootTip[id] = Hexapod.applyRotToGait(legId, targetPos);

		Hexapod.footTip[id] = Hexapod.targetFootTip[id];

		xil_printf("L%d{%d %d %d}\r\n", id, (int) roundf(Hexapod.footTip[id].x),
							(int) roundf(Hexapod.footTip[id].y),
							(int) roundf(Hexapod.footTip[id].z));

		float time = Hexapod.stepTime * targetPos.duration;
		Hexapod.leg[id].moveTo(Hexapod.footTip[id], time);

		TickType_t wait = pdMS_TO_TICKS( time * 1000 );
		vTaskDelay( wait );

		if(footDown){
			footDown = false;
			xTaskNotifyGive(xWalkingTask);
			xil_printf("L%d Down\r\n", id);
		}
	}
}

#endif /* SRC_TASKFUNCTION_H_ */
