#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <sys/types.h>
#include <signal.h>
#include <fcntl.h>
#include <time.h>
#include <errno.h>

#include "../include/ba_algorithm.h"
#include "../include/control_functions.h"

/* initialize ba algorithm thread */
int init_ba_algorithm() {

    int status;

    pthread_t BaThread;
    pthread_attr_t aBaThreadAttr;
    pthread_attr_init(&aBaThreadAttr);
    pthread_attr_setschedpolicy(&aBaThreadAttr, SCHED_FIFO);

    // initilize algorithm variables
    status = init_ba();

    // create new stc thread
    if ((status = pthread_create(&BaThread, &aBaThreadAttr, tBaThreadFunc, NULL))) {
        fprintf(stderr, "Cannot create ba thread.\n");
    return 0;
	}

    return EXIT_SUCCESS;
}

/* BA algorithm thread function */
void *tBaThreadFunc(void *cookie) {

    int status; 
    int policy = SCHED_FIFO;
    struct sched_param param;

    pthread_getschedparam(pthread_self(), &policy, &param);
    param.sched_priority = sched_get_priority_max(policy)-4;
	pthread_setschedparam(pthread_self(), policy, &param);


    // infinite loop calculating next step with ba algorithm
    for(;;) {
        // wait for notification that next step must be calculated
        // sem_wait(&spool_calc_next_step_stc);
        // calculate next step, movement controller will
        // know what to do with result
        // if (algorithm_finished == 0) {
        //     status = calc_next_step(); 
        // } else {
        //     next_step = 0;
        //     movement_type = 0;
        // }

        // wait for semaphore: task (moving/rotating) finished
        // moving is basically moving until obstacle
        // TODO update some data
        // data can be big matrix with some sort of mapping
        // TODO check some nbh to determine task
        // TODO first function calculates type of task: 
        // 1: move boustrophodenically
        // 2: critical point, find a new point (call a function) and move to that point
        // 3: continue moving to next starting point
        // TODO second function calculates 

    }

    return EXIT_SUCCESS;
}

