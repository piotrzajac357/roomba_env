#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#include <semaphore.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <pthread.h>
#include <signal.h>
#include <errno.h>
#include <time.h>

#include "../include/task_to_movement.h"
#include "../include/control_functions.h"

/* initialize converting task to movement type and supervising movement thread */
int init_task_to_movement() {

    int status;

	pthread_attr_t aT2MThreadAttr;
	struct itimerspec timerSpecStruct;
	timer_t	timerVar;
	struct	sigevent timerEvent;
	pthread_attr_init(&aT2MThreadAttr);

    timerEvent.sigev_notify = SIGEV_THREAD;
    timerEvent.sigev_notify_function = (void*)tT2MThreadFunc;
	timerEvent.sigev_notify_attributes = &aT2MThreadAttr;

    if ((status = timer_create(CLOCK_REALTIME, &timerEvent, &timerVar))) {
        fprintf(stderr, "Error creating timer : %d\n", status);
    return 0;
    }

    timerSpecStruct.it_value.tv_sec = 0;
	timerSpecStruct.it_value.tv_nsec = 50000000;
	timerSpecStruct.it_interval.tv_sec = 0;
	// gigaimportant - interval must be lower than 
	// sim->control data transfer
	timerSpecStruct.it_interval.tv_nsec = 1000000;

    init_movement_type();

    timer_settime(timerVar, 0, &timerSpecStruct, NULL);

    return EXIT_SUCCESS;
}

/* converting task to movement type and supervising movement thread function */
void * tT2MThreadFunc(void *cookie) {

	int status;
	int policy = SCHED_FIFO;
	struct sched_param param;

    pthread_getschedparam(pthread_self(), &policy, &param);
    param.sched_priority = sched_get_priority_max(policy) - 3;
	pthread_setschedparam(pthread_self(), policy, &param);
    
    if((status = calculate_movement_type())){
    	fprintf(stderr, "Error calculating movement type: %d\n", status);
  		return 0;
  	}
    
    return EXIT_SUCCESS;
}
