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

#include "control_update.h"
#include "control_functions.h"

/* initialize control (motors power and suction power) update thread */
int init_control_update() {

    int status;

	pthread_attr_t aControlUpdateThreadAttr;
	struct itimerspec timerSpecStruct;
	timer_t	timerVar;
	struct	sigevent timerEvent;
	pthread_attr_init(&aControlUpdateThreadAttr);

    timerEvent.sigev_notify = SIGEV_THREAD;
    timerEvent.sigev_notify_function = (void*)tControlUpdateThreadFunc;
	timerEvent.sigev_notify_attributes = &aControlUpdateThreadAttr;

    if ((status = timer_create(CLOCK_REALTIME, &timerEvent, &timerVar))) {
        fprintf(stderr, "Error creating timer : %d\n", status);
    return 0;
    }

    timerSpecStruct.it_value.tv_sec = 0;
	timerSpecStruct.it_value.tv_nsec = 5000000;
	timerSpecStruct.it_interval.tv_sec = 0;
	timerSpecStruct.it_interval.tv_nsec = 1000000;

	init_control();

    timer_settime(timerVar, 0, &timerSpecStruct, NULL);

    return EXIT_SUCCESS;
}

/* control calculating based on movement type thread function */
void * tControlUpdateThreadFunc(void *cookie) {

	int status;
	int policy = SCHED_FIFO;
	struct sched_param param;

    pthread_getschedparam(pthread_self(), &policy, &param);
    param.sched_priority = sched_get_priority_max(policy) - 3;
	pthread_setschedparam(pthread_self(), policy, &param);

	if((status = calculate_control())){
    	fprintf(stderr, "Error calculating control: %d\n", status);
  		return 0;
  	}
    return EXIT_SUCCESS;
}
