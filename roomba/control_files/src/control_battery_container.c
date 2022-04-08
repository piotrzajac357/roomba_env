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

#include "control_battery_container.h"
#include "control_functions.h"

/* initialize battery and container state checking thread */
int init_battery_container_inspect() {

    int status;

    pthread_attr_t aBattContThreadAttr;
	struct itimerspec timerSpecStruct;
	timer_t	timerVar;
	struct	sigevent timerEvent;
	pthread_attr_init(&aBattContThreadAttr);

    timerEvent.sigev_notify = SIGEV_THREAD;
    timerEvent.sigev_notify_function = (void*)tBattContThreadFunc;
	timerEvent.sigev_notify_attributes = &aBattContThreadAttr;

    if ((status = timer_create(CLOCK_REALTIME, &timerEvent, &timerVar))) {
        fprintf(stderr, "Error creating timer : %d\n", status);
    return 0;
    }

    timerSpecStruct.it_value.tv_sec = 0;
	timerSpecStruct.it_value.tv_nsec = 500000000;
	timerSpecStruct.it_interval.tv_sec = 0;
	timerSpecStruct.it_interval.tv_nsec = 500000000;

    init_battery_container();

    timer_settime(timerVar, 0, &timerSpecStruct, NULL);

    return EXIT_SUCCESS;
}

/* inspect battery and container state thread function */
void *tBattContThreadFunc(void *cookie) {

    int status;
    int policy = SCHED_FIFO;
    struct sched_param param;

    pthread_getschedparam(pthread_self(), &policy, &param);
    param.sched_priority = sched_get_priority_max(policy) - 8;
	pthread_setschedparam(pthread_self(), policy, &param);

	if((status = inspect_battery_and_container())){
    	fprintf(stderr, "Error inspecting battery and container: %d\n", status);
  		return 0;
  	}

    return EXIT_SUCCESS;
}
