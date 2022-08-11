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

    // inititalize algorithm variables
    
    status = init_swf();
    sleep(3);
    
    // create new swf thread
    if ((status = pthread_create(&SwfThread, &aSwfThreadAttr, tSwfThreadFunc, NULL))) {
        fprintf(stderr, "Cannot create stc thread.\n");
        return 0;
    }

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