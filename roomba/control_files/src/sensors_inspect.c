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

#include "sensors_inspect.h"
#include "control_functions.h"

/* initialize sensors inspecting thread */
int init_sensors_inspect() {

    int status;

    pthread_attr_t aSensorsThreadAttr;
	struct itimerspec timerSpecStruct;
	timer_t	timerVar;
	struct	sigevent timerEvent;
	pthread_attr_init(&aSensorsThreadAttr);

    timerEvent.sigev_notify = SIGEV_THREAD;
    timerEvent.sigev_notify_function = (void*)tSensorsThreadFunc;
	timerEvent.sigev_notify_attributes = &aSensorsThreadAttr;

    if ((status = timer_create(CLOCK_REALTIME, &timerEvent, &timerVar))) {
        fprintf(stderr, "Error creating timer : %d\n", status);
    return 0;
    }

    timerSpecStruct.it_value.tv_sec = 0;
	timerSpecStruct.it_value.tv_nsec = 50000000;
	timerSpecStruct.it_interval.tv_sec = 0;
	timerSpecStruct.it_interval.tv_nsec = 5000000;

    timer_settime(timerVar, 0, &timerSpecStruct, NULL);

    return EXIT_SUCCESS;
}

/* sensors inspecting thread function */
void * tSensorsThreadFunc(void *cookie) {

	int status;
	int policy = SCHED_FIFO;
	struct sched_param param;

    pthread_getschedparam(pthread_self(), &policy, &param);
    param.sched_priority = sched_get_priority_max(policy);
	pthread_setschedparam(pthread_self(), policy, &param);

	if((status = inspect_sensors())){
    	fprintf(stderr, "Error inspecting sensor: %d\n", status);
  		return 0;
  	}
    return EXIT_SUCCESS;
}
