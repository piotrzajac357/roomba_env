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

#include "dist_sensors_update.h"
#include "sim_functions.h"

/* initialize distance sensors update thread */
int init_dist_sensors_update(){

    int status;

	pthread_attr_t aDistSensorsThreadAttr;
	struct itimerspec timerSpecStruct;
	timer_t	timerVar;
	struct	sigevent timerEvent;

	pthread_attr_init(&aDistSensorsThreadAttr);

	timerEvent.sigev_notify = SIGEV_THREAD;
    timerEvent.sigev_notify_function = (void*)tDistSensorsThreadFunc;
	timerEvent.sigev_notify_attributes = &aDistSensorsThreadAttr;

  	if ((status = timer_create(CLOCK_REALTIME, &timerEvent, &timerVar))) {
  		fprintf(stderr, "Error creating timer : %d\n", status);
   		return 0;
    }

	timerSpecStruct.it_value.tv_sec = 0;
	timerSpecStruct.it_value.tv_nsec = sim_step_dist_sensors * 1000000000;
	timerSpecStruct.it_interval.tv_sec = 0;
	timerSpecStruct.it_interval.tv_nsec = sim_step_dist_sensors * 1000000000;

	// setting starting values of distance sensors
    initialize_dist_sensors();

	timer_settime(timerVar, 0, &timerSpecStruct, NULL);

    return EXIT_SUCCESS;
}

/* Distance sensors update thread */
void * tDistSensorsThreadFunc(void *cookie) {

	int status;
	int policy = SCHED_FIFO;
	struct sched_param param;

	pthread_getschedparam(pthread_self(), &policy, &param);
	param.sched_priority = sched_get_priority_max(policy) - 1;
	pthread_setschedparam( pthread_self(), policy, &param);

	if((status = calculate_sensors())){
    	fprintf(stderr, "Error calculating dist sensors: %d\n", status);
  		return 0;
  	}

    return EXIT_SUCCESS;
}
