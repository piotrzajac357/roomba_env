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

#include "quality_indexes.h"
#include "sim_functions.h"

/* initialize quality indexes calculating thread */
int init_quality_indexes_update(){
    
    int status;

    pthread_attr_t aQualityThreadAttr;
    struct itimerspec timerSpecStruct;
    timer_t timerVar;
    struct sigevent timerEvent;

    pthread_attr_init(&aQualityThreadAttr);

    timerEvent.sigev_notify = SIGEV_THREAD;
    timerEvent.sigev_notify_function = (void*)tQualityThreadFunc;
    timerEvent.sigev_notify_attributes = &aQualityThreadAttr;

  	if ((status = timer_create(CLOCK_REALTIME, &timerEvent, &timerVar))) {
  		fprintf(stderr, "Error creating timer : %d\n", status);
   		return 0;
    }

	timerSpecStruct.it_value.tv_sec = 2;
	timerSpecStruct.it_value.tv_nsec = 0;
	timerSpecStruct.it_interval.tv_sec = 0;
	timerSpecStruct.it_interval.tv_nsec = sim_step_position * 1000000000;

    initialize_quality_indexes();

    timer_settime(timerVar, 0, &timerSpecStruct, NULL);

    return EXIT_SUCCESS;
}

void * tQualityThreadFunc(void *cookie) {

    int status;
    int policy = SCHED_FIFO;
    struct sched_param param;

    pthread_getschedparam(pthread_self(), &policy, &param);
    param.sched_priority = sched_get_priority_min(policy) - 3;
    pthread_setschedparam(pthread_self(), policy, &param);

    // in microseconds
    status = calculate_qis(sim_step_position);
    
    return EXIT_SUCCESS;
}