/*
 * task.h
 *
 *  Created on: Apr 13, 2017
 *      Author: Phanomphon Yotchon
 */

#if GEN_TEST_APP == 0

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
		if(Hexapod.improvePitch || Hexapod.improveRoll || Hexapod.improveYaw){
			xil_printf("B");
			// balance mode
			Hexapod.balance();
		}
		xil_printf("M");
//		Hexapod.moving();
		vTaskDelay( dt );
	}
}

static void hexapodWalkingTask( void *pvParameters ){
//	for(;;);
	TickType_t stepTime;

	TaskHandle_t xLegGait[6];

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
					(void *) i, tskIDLE_PRIORITY, &xLegGait[i] );
	}

	for( ;; ){
		stepTime = pdMS_TO_TICKS( Hexapod.stepTime * 1000 );
		xil_printf("Gait\r\n");

		for(int i = 0; i < 6; i++){
			Trajectory3d tp = {(float)i, 14.0, 5.0, 0.5};
			xQueueSend(xTrajQueue[i], (void *)&tp, 0UL);
//			vTaskDelay( stepTime );
		}
		vTaskDelay( stepTime );
	}
}

static void hexapodLegGaitTask( void *pvParameters ){
	TickType_t dt;
	int id = (uint32_t) pvParameters;
	xil_printf("{%d %d %d}\r\n", (int) roundf(Hexapod.footTip[id].x),
								 (int) roundf(Hexapod.footTip[id].y),
								 (int) roundf(Hexapod.footTip[id].z));
	for(;;){
		Trajectory3d targetPos;
		/* Wait without a timeout for data. */
		xQueueReceive(xTrajQueue[id], (void *) &targetPos, portMAX_DELAY );
		dt = pdMS_TO_TICKS( Hexapod.dt * 1000 );

		float r = roundf(targetPos.duration * Hexapod.stepTime / Hexapod.dt) - 1;
		int cnt = (int) r;

		FootTip old_pos = Hexapod.targetFootTip[id];	// current FootTip position
		FootTip new_pos(targetPos.x, targetPos.y, targetPos.z);
		FootTip step_pos = (new_pos - old_pos) / r;

		// Interpolation round
		for(int i = 0; i < cnt; i++){
			xil_printf("L%d{%d %d %d}\r\n", (uint32_t) pvParameters,
					(int) roundf(step_pos.x),
					(int) roundf(step_pos.y),
					(int) roundf(step_pos.z));
			Hexapod.targetFootTip[id] = Hexapod.targetFootTip[id] + step_pos;

			if(Hexapod.improvePitch || Hexapod.improveRoll || Hexapod.improveYaw){
				// balance mode
				Hexapod.footTip[id] = Hexapod.footTip[id] + step_pos;
			}
			else {
				// fixed mode
				Hexapod.footTip[id] = Hexapod.targetFootTip[id];
			}
			xil_printf("{%d %d %d}\r\n", (int) roundf(Hexapod.footTip[id].x),
					(int) roundf(Hexapod.footTip[id].y),
					(int) roundf(Hexapod.footTip[id].z));
			vTaskDelay( dt );
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
		xil_printf("{%d %d %d}\r\n", (int) roundf(Hexapod.footTip[id].x),
							(int) roundf(Hexapod.footTip[id].y),
							(int) roundf(Hexapod.footTip[id].z));
		vTaskDelay( dt );
	}
}

#endif // GEN_TEST_APP == 0
