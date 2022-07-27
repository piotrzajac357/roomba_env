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


    pthread_attr_t aBaMapThreadAttr;
    struct itimerspec timerSpecStruct;
    timer_t timerVar;
    struct sigevent timerEvent;
    pthread_attr_init(&aBaMapThreadAttr);

    timerEvent.sigev_notify = SIGEV_THREAD;
    timerEvent.sigev_notify_function = (void*)tBaMapThreadFunction;
    timerEvent.sigev_notify_attributes = &aBaMapThreadAttr;

    if ((status = timer_create(CLOCK_REALTIME, &timerEvent, &timerVar))) {
        fprintf(stderr, "Error creating timer : %d\n", status);
    return 0;
    }

    timerSpecStruct.it_value.tv_sec = 0;
	timerSpecStruct.it_value.tv_nsec = 5000000;
	timerSpecStruct.it_interval.tv_sec = 0;
	timerSpecStruct.it_interval.tv_nsec = 10000000;

    // initilize algorithm variables
    sleep(1);
    status = init_ba();

    // create new stc thread
    if ((status = pthread_create(&BaThread, &aBaThreadAttr, tBaThreadFunc, NULL))) {
        fprintf(stderr, "Cannot create ba thread.\n");
    return 0;
	}

    // start timer creating thread with BA map update
    timer_settime(timerVar, 0, &timerSpecStruct, NULL);

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
        sem_wait(&spool_calc_next_step_ba);
        // calculate next step, movement controller will
        // know what to do with result
        // if (algorithm_finished == 0) {
        //     status = calc_next_step(); 
        // } else {
        //     next_step = 0;
        //     movement_type = 0;
        // }
        if (algorithm_finished == 0) {
            status = calc_next_task();

        } else {
            current_task_ba = 0;
            movement_type = 0;
        }

        spool_next_step_ba_calculated = 1;
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

void* tBaMapThreadFunction(void *cookie) {

    int status;
    int policy = SCHED_FIFO;
    struct sched_param param;

    pthread_getschedparam(pthread_self(), &policy, &param);
    param.sched_priority = sched_get_priority_max(policy) - 6;
	pthread_setschedparam(pthread_self(), policy, &param);
 
    if ((status = update_ba_map())) {
        fprintf(stderr, "Error updating ba map");
        return 0;
    }

    
    return EXIT_SUCCESS;
}