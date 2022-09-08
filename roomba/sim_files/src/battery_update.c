#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <time.h>
#include <semaphore.h>
#include <fcntl.h>
#include <sys/types.h>
#include <math.h>

#include "battery_update.h"
#include "sim_functions.h"

/* Initialize battery update thread */
int init_battery_update(){

    int status;

	pthread_attr_t aBatteryThreadAttr;
	struct itimerspec timerSpecStruct;
	timer_t	timerVar;
	struct	sigevent timerEvent;

	/* Initialize thread attributes structure */
	pthread_attr_init(&aBatteryThreadAttr);

	/* Initialize event to create thread */
	timerEvent.sigev_notify = SIGEV_THREAD;
    timerEvent.sigev_notify_function = (void*)tBatteryThreadFunc;
	timerEvent.sigev_notify_attributes = &aBatteryThreadAttr;

	/* Create timer */
  	if ((status = timer_create(CLOCK_REALTIME, &timerEvent, &timerVar))) {
  		fprintf(stderr, "Error creating timer: %d\n", status);
   		return 0;
    }

    /* Set up timer structure with time parameters */
	timerSpecStruct.it_value.tv_sec = sim_step_battery;		// 1 sec
	timerSpecStruct.it_value.tv_nsec = 0;
	timerSpecStruct.it_interval.tv_sec = sim_step_battery;
	timerSpecStruct.it_interval.tv_nsec = 0;

	// setting starting values
    initialize_battery();

	/* Change timer parameters and run */
	timer_settime(timerVar, 0, &timerSpecStruct, NULL);

    return EXIT_SUCCESS;
}

/* Battery state update thread function */
void * tBatteryThreadFunc(void *cookie) {

	int status;
    /* Scheduling policy: FIFO or RR */
	int policy = SCHED_FIFO;
	/* Structure of other thread parameters */
	struct sched_param param;

	/* Read modify and set new thread priority */
	pthread_getschedparam(pthread_self(), &policy, &param);
	param.sched_priority = sched_get_priority_max(policy) - 2;
	pthread_setschedparam( pthread_self(), policy, &param);

	if((status = calculate_battery())){
    	fprintf(stderr, "Error calculating battery state: %d\n", status);
  		return 0;
  	}		

    return EXIT_SUCCESS;
}
