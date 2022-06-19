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

#include "../include/control_to_sim.h"
#include "../include/control_functions.h"
#include "../include/control.h"

/* initialize control -> sim data sending thread */
int init_control_to_sim(){

    int status;

	pthread_attr_t aC2SThreadAttr;
    struct itimerspec timerSpecStruct;
	timer_t	timerVar;
	struct	sigevent timerEvent;
	pthread_attr_init(&aC2SThreadAttr);

	timerEvent.sigev_notify = SIGEV_THREAD;
    timerEvent.sigev_notify_function = (void*)tC2SThreadFunc;
	timerEvent.sigev_notify_attributes = &aC2SThreadAttr;

  	if ((status = timer_create(CLOCK_REALTIME, &timerEvent, &timerVar))) {
  		fprintf(stderr, "Error creating timer : %d\n", status);
   		return 0;
    }

    timerSpecStruct.it_value.tv_sec = 1;            // wait a sec before sending 
	timerSpecStruct.it_value.tv_nsec = 0;
	timerSpecStruct.it_interval.tv_sec = 0;
	timerSpecStruct.it_interval.tv_nsec = 1000000;

    timer_settime(timerVar, 0, &timerSpecStruct, NULL);

    return EXIT_SUCCESS;
}

/* control -> sim data sending thread function */
void * tC2SThreadFunc(void *cookie) {

	int status;
	int policy = SCHED_FIFO;
	struct sched_param param;

    pthread_getschedparam(pthread_self(), &policy, &param);
	param.sched_priority = sched_get_priority_max(policy) - 1;
	pthread_setschedparam(pthread_self(), policy, &param);

    // read all neccessary data from process and write to tmp variables
    sem_wait(&controlSemaphore);
    double left_motor_power_tmp = left_motor_power;
    double right_motor_power_tmp = right_motor_power;
    double suction_power_tmp = suction_power;
    sem_post(&controlSemaphore);

    /* write to IPC shared memory */
    sem_wait(mutex_sem_c2s);
    c2s_shm_ptr->left_motor_power = left_motor_power_tmp;
    c2s_shm_ptr->right_motor_power = right_motor_power_tmp;
    c2s_shm_ptr->suction_power = suction_power_tmp;
    sem_post(mutex_sem_c2s);

    // notify about new data
    sem_post(spool_sem_c2s);

    return EXIT_SUCCESS;
}