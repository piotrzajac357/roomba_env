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

#include "../include/stc_algorithm.h"
#include "../include/control_functions.h"

/* initialize stc algorithm thread */
int init_stc_algorithm(){

    int status;

    pthread_t StcThread;
    pthread_attr_t aStcThreadAttr;
    pthread_attr_init(&aStcThreadAttr);
    pthread_attr_setschedpolicy(&aStcThreadAttr, SCHED_FIFO);

    // initialize algoithm variables
    status = init_stc();

    // create new stc thread
    if ((status = pthread_create(&StcThread, &aStcThreadAttr, tStcThreadFunc, NULL))) {
        fprintf(stderr, "Cannot create stc thread.\n");
    return 0;
	}

    return EXIT_SUCCESS;

}

/* STC algorithm thread function */
void *tStcThreadFunc(void *cookie) {

    int status;
    int policy = SCHED_FIFO;
    struct sched_param param;

    pthread_getschedparam(pthread_self(), &policy, &param);
    param.sched_priority = sched_get_priority_max(policy)-2;
	pthread_setschedparam(pthread_self(), policy, &param);

    // infinite loop calculating next step with stc algorithm
    for(;;) {
        // wait for notification that next step must be calculated
        sem_wait(&spool_calc_next_step_stc);
        // calculate next step, movement controller will
        // know what to do with result
        if (algorithm_finished == 0) {
            sem_wait(&superv_calc_stc_Semaphore);
            status = calc_next_step();
            sem_post(&superv_calc_stc_Semaphore);
        } else {
            next_step = 0;
            movement_type = 0;
        }

    }

    return EXIT_SUCCESS;
}
