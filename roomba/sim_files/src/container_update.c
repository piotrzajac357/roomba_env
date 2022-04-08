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

#include "container_update.h"
#include "sim_functions.h"

/* Initialize container update thread */
int init_container_update(){

    int status;

	pthread_attr_t aContainerThreadAttr;
	struct itimerspec timerSpecStruct;
	timer_t	timerVar;
	struct	sigevent timerEvent;

	pthread_attr_init(&aContainerThreadAttr);

	timerEvent.sigev_notify = SIGEV_THREAD;
    timerEvent.sigev_notify_function = (void*)tContainerThreadFunc;
	timerEvent.sigev_notify_attributes = &aContainerThreadAttr;

    /* Create timer */
  	if ((status = timer_create(CLOCK_REALTIME, &timerEvent, &timerVar))) {
  		fprintf(stderr, "Error creating timer : %d\n", status);
   		return 0;
    }

    /* Set up timer structure with time parameters */
	timerSpecStruct.it_value.tv_sec = sim_step_container;		// 1sec
	timerSpecStruct.it_value.tv_nsec = 0;
	timerSpecStruct.it_interval.tv_sec = sim_step_container;
	timerSpecStruct.it_interval.tv_nsec = 0;

	// setting starting state of container
    initialize_container();

    /* Change timer parameters and run */
	timer_settime(timerVar, 0, &timerSpecStruct, NULL);

    return EXIT_SUCCESS;
}

/* Container state update thread function */
void * tContainerThreadFunc(void *cookie) {

	int status;
	int policy = SCHED_FIFO;
	struct sched_param param;

    pthread_getschedparam(pthread_self(), &policy, &param);
	param.sched_priority = sched_get_priority_min(policy) + 4;
	pthread_setschedparam( pthread_self(), policy, &param);

	if((status = calculate_container())){
    	fprintf(stderr, "Error calculating container state: %d\n", status);
  		return 0;
  	}		

    return EXIT_SUCCESS;
}
