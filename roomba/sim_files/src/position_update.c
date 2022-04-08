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

#include "position_update.h"
#include "sim_functions.h"

/* initialization of position update thread */
int init_position_update(){

    int status;
	pthread_attr_t aPositionThreadAttr;
	struct itimerspec timerSpecStruct;
	timer_t	timerVar;
	struct	sigevent timerEvent;

	pthread_attr_init(&aPositionThreadAttr);

	timerEvent.sigev_notify = SIGEV_THREAD;
    timerEvent.sigev_notify_function = (void*)tPositionThreadFunc;
	timerEvent.sigev_notify_attributes = &aPositionThreadAttr;

  	if ((status = timer_create(CLOCK_REALTIME, &timerEvent, &timerVar))) {
  		fprintf(stderr, "Error creating timer : %d\n", status);
  		return 0;
    }

	timerSpecStruct.it_value.tv_sec = 0;
	timerSpecStruct.it_value.tv_nsec = sim_step_position * 1000000000;
	timerSpecStruct.it_interval.tv_sec = 0;
	timerSpecStruct.it_interval.tv_nsec = sim_step_position * 1000000000;

	// setting starting position values
	initialize_position();

	timer_settime(timerVar, 0, &timerSpecStruct, NULL);

    return EXIT_SUCCESS;
}

/* position update thread function */
void * tPositionThreadFunc(void *cookie) {

	int status;
	int policy = SCHED_FIFO;
	struct sched_param param;

    pthread_getschedparam(pthread_self(), &policy, &param);
	param.sched_priority = sched_get_priority_max(policy) - 4;
	pthread_setschedparam( pthread_self(), policy, &param);

	if((status = calculate_position())){
    	fprintf(stderr, "Error calculating position: %d\n", status);
  		return 0;
  	}	
	
    return 0;
}
