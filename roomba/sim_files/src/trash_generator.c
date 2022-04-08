#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#include <semaphore.h>
#include <fcntl.h>
#include <pthread.h>
#include <errno.h>
#include <time.h>

#include "trash_generator.h"
#include "sim_functions.h"

int thread_counter = 0;

/* Initialization of trash generating thread */
int init_trash_generator() {

    int status;
	pthread_attr_t aTrashThreadAttr;
	struct itimerspec timerSpecStruct;
	timer_t	timerVar;
	struct	sigevent timerEvent;
    pthread_attr_init(&aTrashThreadAttr);

	timerEvent.sigev_notify = SIGEV_THREAD;
    timerEvent.sigev_notify_function = (void*)tTrashThreadFunc;
	timerEvent.sigev_notify_attributes = &aTrashThreadAttr;

  	if ((status = timer_create(CLOCK_REALTIME, &timerEvent, &timerVar))) {
  		fprintf(stderr, "Error creating timer : %d\n", status);
   		return 0;
    }

    // produce new trash area every 20sec
    timerSpecStruct.it_value.tv_sec = 5;
	timerSpecStruct.it_value.tv_nsec = 0;
	timerSpecStruct.it_interval.tv_sec = 20;
	timerSpecStruct.it_interval.tv_nsec = 0;

    // read trash coords from file
    if((status = initialize_trash())) {
        fprintf(stderr, "Error reading trash coords from file: %d\n", status);
    }

    timer_settime(timerVar, 0, &timerSpecStruct, NULL);

    return EXIT_SUCCESS;
}

/* Trash generator thread function */
void * tTrashThreadFunc(void *cookie) {

    int status;
    int policy = SCHED_FIFO;
    struct sched_param param;

    pthread_getschedparam(pthread_self(), &policy, &param);
    param.sched_priority = sched_get_priority_min(policy) + 1;
	pthread_setschedparam(pthread_self(), policy, &param);

    if((status = generate_trashes(&thread_counter))){
    	fprintf(stderr, "Error generating trashes: %d\n", status);
  	    return 0;
  	}	

    thread_counter++;
    return EXIT_SUCCESS;
}
