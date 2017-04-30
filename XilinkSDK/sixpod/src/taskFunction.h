/*
 * task.h
 *
 *  Created on: Apr 13, 2017
 *      Author: Phanomphon Yotchon
 */

#ifndef SRC_TASKFUNCTION_H_
#define SRC_TASKFUNCTION_H_

TickType_t stepTime;

typedef struct trajectory3d {
	float x;
	float y;
	float z;
	float duration;
} Trajectory3d;


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
				xil_printf("B");
				// balance mode
				Hexapod.balance();
			}
		}

		xil_printf("M");
		Hexapod.moving();

		for(int i = 0; i < 6; i++){
			xTaskNotifyGive(xLegGait[i]);
		}
		vTaskDelay( dt );
	}
}

static void hexapodWalkingTask( void *pvParameters ){
	const char * taskName[6] = {
			"Leg1Gait",
			"Leg2Gait",
			"Leg3Gait",
			"Leg4Gait",
			"Leg5Gait",
			"Leg6Gait"
	};

	for(int i = 0; i < 6; i++){
		xQueueReset(xTrajQueue[i]);
		xTaskCreate( hexapodLegGaitTask,
					taskName[i], configMINIMAL_STACK_SIZE,
					(void *) i, DEFAULT_THREAD_PRIO, &xLegGait[i] );
	}

	Trajectory3d tp1 = {-3, 14.0, 0, 0.5};
	Trajectory3d tp2 = {0, 14.0, 2, 0.25};
	Trajectory3d tp4 = {3, 14.0, 0, 0.25};

	Hexapod.stepTime = 2;
	Hexapod.dt = 0.1;

	Trajectory3d tpf1 = {-3, 14.0, 0, 0.5};
	xQueueSend(xTrajQueue[0], (void *)&tpf1, 0UL);
	xQueueSend(xTrajQueue[2], (void *)&tpf1, 0UL);
	xQueueSend(xTrajQueue[4], (void *)&tpf1, 0UL);

	Trajectory3d tpf4 = {3, 14.0, 0, 0.5};
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

static void hexapodLegGaitTask( void *pvParameters ){
	int id = (uint32_t) pvParameters;
	bool footDown = false;

	for(;;){
		Trajectory3d targetPos;

		/* Wait without a timeout for data. */
		xQueueReceive(xTrajQueue[id], (void *) &targetPos, portMAX_DELAY );
		if(Hexapod.targetFootTip[id].z > 0.0 && targetPos.z <= 0.0){
			footDown = true;
		}

		float r = roundf(targetPos.duration * Hexapod.stepTime / Hexapod.dt) - 1;
		int cnt = (int) r;

		FootTip old_pos = Hexapod.targetFootTip[id];	// current FootTip position
		FootTip new_pos(targetPos.x, targetPos.y, targetPos.z);
		FootTip step_pos = (new_pos - old_pos) / r;

		ulTaskNotifyTake( pdTRUE, portMAX_DELAY ); // Take from moving task

		// Interpolation round
		for(int i = 0; i < cnt; i++){
			Hexapod.targetFootTip[id] = Hexapod.targetFootTip[id] + step_pos;

			if(Hexapod.improvePitch || Hexapod.improveRoll || Hexapod.improveYaw){
				// balance mode
				Hexapod.footTip[id] = Hexapod.footTip[id] + step_pos;
			}
			else {
				// fixed mode
				Hexapod.footTip[id] = Hexapod.targetFootTip[id];
			}
			xil_printf("L%d{%d %d %d}\r\n", id, (int) roundf(Hexapod.footTip[id].x),
					(int) roundf(Hexapod.footTip[id].y),
					(int) roundf(Hexapod.footTip[id].z));

			ulTaskNotifyTake( pdTRUE, portMAX_DELAY ); // Take from moving task
		}

		// The final round
		step_pos = new_pos - Hexapod.targetFootTip[id];
		Hexapod.targetFootTip[id] = new_pos;

		if(Hexapod.improvePitch || Hexapod.improveRoll || Hexapod.improveYaw){
			// balance mode
			Hexapod.footTip[id] = Hexapod.footTip[id] + step_pos;
		}
		else {
			// fixed mode
			Hexapod.footTip[id] = Hexapod.targetFootTip[id];
		}
		xil_printf("L%d{%d %d %d}\r\n", id, (int) roundf(Hexapod.footTip[id].x),
							(int) roundf(Hexapod.footTip[id].y),
							(int) roundf(Hexapod.footTip[id].z));

		if(footDown){
			TickType_t dt = pdMS_TO_TICKS( Hexapod.dt * 1000 );
			vTaskDelay( dt );
			footDown = false;
			xTaskNotifyGive(xWalkingTask);
			xil_printf("L%d Down\r\n", id);
		}
	}
}

#endif /* SRC_TASKFUNCTION_H_ */
