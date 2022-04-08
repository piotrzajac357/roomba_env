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

#include "../include/set_new_task.h"
#include "../include/control_functions.h"
#include "../include/control.h"
#include "../include/load_plan.h"

/* initialize selecting new direction thread */
int init_set_new_task() {

    int status;
    pthread_t TaskThread;
    pthread_attr_t aTaskThreadAttr;
    pthread_attr_init(&aTaskThreadAttr);
    pthread_attr_setschedpolicy(&aTaskThreadAttr, SCHED_FIFO);

    // read plan from file, direction grades are based on plan
    init_plan();

    init_new_task();

    // time_t tt;
    // int seed = time(&tt);    // random

    // initialize pseudorandom numbers generator
    int seed = 1;
    srand(seed);                // pseudorandom

    if ((status = pthread_create(&TaskThread, &aTaskThreadAttr, tTaskThreadFunc, NULL))) {
        fprintf(stderr, "Cannot create thread.\n");
    return 0;
	}

    return EXIT_SUCCESS;
}

/* new direction setting thread function */
void *tTaskThreadFunc(void *cookie) {

    int status;
    int policy = SCHED_FIFO;
    struct sched_param param;
    pthread_getschedparam(pthread_self(), &policy, &param);
	param.sched_priority = sched_get_priority_max(policy) - 10;
	pthread_setschedparam(pthread_self(), policy, &param);

    for(;;) {
        /* wait for spool ("new task must be set" signal) */
        sem_wait(&spool_set_new_task);
        
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
    }
    return 0;
}