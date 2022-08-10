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
#include <math.h>

#include "../include/rg_algorithm.h"
#include "../include/control_functions.h"
#include "../include/control.h"

/* initialize selecting new direction thread */
int init_rg_algorithm() {

    int status;
    pthread_t RgThread;
    pthread_attr_t aRgThreadAttr;
    pthread_attr_init(&aRgThreadAttr);
    pthread_attr_setschedpolicy(&aRgThreadAttr, SCHED_FIFO);

    pthread_attr_t aRgMapThreadAttr;
    struct itimerspec timerSpecStruct;
    timer_t timerVar;
    struct sigevent timerEvent;
    pthread_attr_init(&aRgMapThreadAttr);

    timerEvent.sigev_notify = SIGEV_THREAD;
    timerEvent.sigev_notify_function = (void*)tRgMapThreadFunction;
    timerEvent.sigev_notify_attributes = &aRgMapThreadAttr;

    if ((status = timer_create(CLOCK_REALTIME, &timerEvent, &timerVar))) {
        fprintf(stderr, "Error creating timer : %d\n", status);
    return 0;
    }

    timerSpecStruct.it_value.tv_sec = 0;
	timerSpecStruct.it_value.tv_nsec = 5000000;
	timerSpecStruct.it_interval.tv_sec = 0;
	timerSpecStruct.it_interval.tv_nsec = 50000000;

    // read plan from file, direction grades are based on plan
    
    sleep(3);

    status = init_rg();

    // time_t tt;
    // int seed = time(&tt);    // random

    // initialize pseudorandom numbers generator
    int seed = 1;
    srand(seed);                // pseudorandom

    if ((status = pthread_create(&RgThread, &aRgThreadAttr, tRgThreadFunc, NULL))) {
        fprintf(stderr, "Cannot create thread.\n");
    return 0;
	}

    timer_settime(timerVar, 0, &timerSpecStruct, NULL);

    return EXIT_SUCCESS;
}

/* new direction setting thread function */
void *tRgThreadFunc(void *cookie) {

    int status;
    int policy = SCHED_FIFO;
    struct sched_param param;
    pthread_getschedparam(pthread_self(), &policy, &param);
	param.sched_priority = sched_get_priority_max(policy) - 10;
	pthread_setschedparam(pthread_self(), policy, &param);

    for(;;) {
        /* wait for spool ("new task must be set" signal) */
        sem_wait(&spool_calc_next_step_rg);
        
        // stop the robot while new task is calculated
        current_task = 0;

        // call a function to select new direction
        if((status = select_new_direction())){
    	fprintf(stderr, "Error selecting new direction: %d\n", status);
  		    return 0;
  	    }

        // get current orientation
        sem_wait(&position_orientationSemaphore);
        double tmp_orientation = orientation;
        sem_post(&position_orientationSemaphore);

        // choose rotating direction based on current and target orientation
        if ((   (target_direction > tmp_orientation) && (fabs(target_direction - tmp_orientation) > 180.0)) ||
            (   (target_direction < tmp_orientation) && (fabs(target_direction - tmp_orientation) < 180.0))) {
            current_task = 2;
        } else {
            current_task = 3;
        }
        spool_next_step_rg_calculated = 1;
    }
    return 0;
}

void* tRgMapThreadFunction(void* cookie) {

    int status; 
    int policy = SCHED_FIFO;
    struct sched_param param;

    pthread_getschedparam(pthread_self(), &policy, &param);
    param.sched_priority = sched_get_priority_min(policy) + 6;
	pthread_setschedparam(pthread_self(), policy, &param);
 
    if ((status = update_rg_map())) {
        fprintf(stderr, "Error updating ba map");
        return 0;
    }

    return EXIT_SUCCESS;
}