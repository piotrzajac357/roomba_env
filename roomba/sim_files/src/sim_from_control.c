#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <semaphore.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <pthread.h>
#include <errno.h>
#include <time.h>

#include "../include/sim_from_control.h"
#include "../include/sim_functions.h"
#include "../include/sim.h"

/* Initialization of thread reading data from control->simulation shared memory */
int init_sim_from_control() {

    int status;
    pthread_t SFCThread;
    pthread_attr_t aSFCThreadAttr;
    pthread_attr_init(&aSFCThreadAttr);
    pthread_attr_setschedpolicy(&aSFCThreadAttr, SCHED_FIFO);

    if ((status = pthread_create( &SFCThread, &aSFCThreadAttr, tSFCThreadFunc, NULL))) {
        fprintf(stderr, "Cannot create thread.\n");
    return 0;
	}

    initialize_control_values();
    
    return EXIT_SUCCESS;
}

/* Control -> Simulation reading thread function */
void *tSFCThreadFunc(void *cookie) {

    int policy = SCHED_FIFO;
    struct sched_param param;

    pthread_getschedparam(pthread_self(), &policy, &param);
	param.sched_priority = sched_get_priority_max(policy) - 7;
	pthread_setschedparam(pthread_self(), policy, &param);

    // Wait for "data is ready" signal and write to tmp variables
    for(;;) {
    sem_wait(spool_sem_c2s);
    sem_wait(mutex_sem_c2s);
    double tmp_left_motor_power = c2s_shm_ptr->left_motor_power;
    double tmp_right_motor_power = c2s_shm_ptr->right_motor_power;
    double tmp_suction_power = c2s_shm_ptr->suction_power;
    double tmp_algorithm_finished = c2s_shm_ptr->algorithm_finished;
    sem_post(mutex_sem_c2s);

    // Write data to process global memory
    sem_wait(&controlSemaphore);
    left_motor_power = tmp_left_motor_power;
    right_motor_power = tmp_right_motor_power;
    suction_power = tmp_suction_power;
    algorithm_finished = tmp_algorithm_finished;
    sem_post(&controlSemaphore);
    }

    return 0;
}