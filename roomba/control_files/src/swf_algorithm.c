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

#include "../include/swf_algorithm.h"
#include "../include/control_functions.h"

/* initialize smooth wall following algorithm thread */
int init_swf_algorithm(){

    int status;

    pthread_t SwfThread;
    pthread_attr_t aSwfThreadAttr;
    pthread_attr_init(&aSwfThreadAttr);
    pthread_attr_setschedpolicy(&aSwfThreadAttr,SCHED_FIFO);

    pthread_attr_t aSwfMapThreadAttr;
    struct itimerspec timerSpecStruct;
    timer_t timerVar;
    struct sigevent timerEvent;
    pthread_attr_init(&aSwfMapThreadAttr);

    timerEvent.sigev_notify = SIGEV_THREAD;
    timerEvent.sigev_notify_function = (void*)tSwfMapThreadFunction;
    timerEvent.sigev_notify_attributes = &aSwfMapThreadAttr;

    if ((status = timer_create(CLOCK_REALTIME, &timerEvent, &timerVar))) {
        fprintf(stderr, "Error creating timer : %d\n", status);
    return 0;
    }

    timerSpecStruct.it_value.tv_sec = 0;
	timerSpecStruct.it_value.tv_nsec = 5000000;
	timerSpecStruct.it_interval.tv_sec = 0;
	timerSpecStruct.it_interval.tv_nsec = 10000000;

    // inititalize algorithm variables
    sleep(3);   
    status = init_swf();
    
    // create new swf thread
    if ((status = pthread_create(&SwfThread, &aSwfThreadAttr, tSwfThreadFunc, NULL))) {
        fprintf(stderr, "Cannot create stc thread.\n");
        return 0;
    }

    timer_settime(timerVar, 0, &timerSpecStruct, NULL);

    return EXIT_SUCCESS;
}


/* Swf algorithm thread function */
void *tSwfThreadFunc(void *cookie) {
    
    int status;
    int policy = SCHED_FIFO;
    struct sched_param param;

    pthread_getschedparam(pthread_self(), &policy, &param);
    param.sched_priority = sched_get_priority_min(policy) + 5;
    pthread_setschedparam(pthread_self(), policy, &param);

    // infinite loop waiting for spool
    for(;;) {
        sem_wait(&spool_calc_next_step_swf);

        if (algorithm_finished == 0){
            status = next_step_swf();
        } else {
            movement_type = 0;
        }
    }

}

void* tSwfMapThreadFunction(void *cookie) {

    int status;
    int policy = SCHED_FIFO;
    struct sched_param param;

    pthread_getschedparam(pthread_self(), &policy, &param);
    param.sched_priority = sched_get_priority_max(policy) - 6;
	pthread_setschedparam(pthread_self(), policy, &param);
 
    status = virtual_sensors();
    if (is_updatable){
        if ((status = update_swf_map())) {
            fprintf(stderr, "Error updating swf map");
            return 0;
        }
    }
    
    return EXIT_SUCCESS;
}