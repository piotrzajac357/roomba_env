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

#include "../include/sim_to_control.h"
#include "../include/sim_functions.h"
#include "../include/sim.h"

/* Initialization of simulation -> control data sending thread */
int init_sim_to_control(){

    int status;

	pthread_attr_t aS2CThreadAttr;
	struct itimerspec timerSpecStruct;
	timer_t	timerVar;
	struct	sigevent timerEvent;

	pthread_attr_init(&aS2CThreadAttr);

	timerEvent.sigev_notify = SIGEV_THREAD;
    timerEvent.sigev_notify_function = (void*)tS2CThreadFunc;
	timerEvent.sigev_notify_attributes = &aS2CThreadAttr;

  	if ((status = timer_create(CLOCK_REALTIME, &timerEvent, &timerVar))) {
  		fprintf(stderr, "Error creating timer : %d\n", status);
   		return 0;
    }

	timerSpecStruct.it_value.tv_sec = 0;
	timerSpecStruct.it_value.tv_nsec = 1000000;
	timerSpecStruct.it_interval.tv_sec = 0;
	timerSpecStruct.it_interval.tv_nsec = 500000;

	timer_settime(timerVar, 0, &timerSpecStruct, NULL);

    return EXIT_SUCCESS;
}

/* Simulation -> Control sending data thread */
void * tS2CThreadFunc(void *cookie) {

	int status;
	int policy = SCHED_FIFO;
	struct sched_param param;

	pthread_getschedparam(pthread_self(), &policy, &param);
	param.sched_priority = sched_get_priority_max(policy) - 6;
	pthread_setschedparam(pthread_self(), policy, &param);

	// read all neccessary data from process and store in tmp variables
    sem_wait(&dist_sensorsSemaphore);
    double tmp_front_sensor = front_sensor;
    double tmp_back_sensor = back_sensor;
    double tmp_left_sensor = left_sensor;
    double tmp_right_sensor = right_sensor;
	sem_post(&dist_sensorsSemaphore);

	sem_wait(&batterySemaphore);
    double tmp_battery_level = battery_level;
	sem_post(&batterySemaphore);

	sem_wait(&containerSemaphore);
    double tmp_container_level = container_level;
	sem_post(&containerSemaphore);

	sem_wait(&position_orientationSemaphore);
    double tmp_position_x = position_x;
    double tmp_position_y = position_y;
    double tmp_orientation = previous_orientation;
	sem_post(&position_orientationSemaphore);

    /* write to IPC shared memory from tmp variables */
    sem_wait(mutex_sem_s2c);
    s2c_shm_ptr->battery_level = tmp_battery_level;
	s2c_shm_ptr->container_level = tmp_container_level;
	s2c_shm_ptr->front_sensor = tmp_front_sensor;
	s2c_shm_ptr->back_sensor = tmp_back_sensor;
	s2c_shm_ptr->left_sensor = tmp_left_sensor;
	s2c_shm_ptr->right_sensor = tmp_right_sensor;
	s2c_shm_ptr->position_x = tmp_position_x;
	s2c_shm_ptr->position_y = tmp_position_y;
	s2c_shm_ptr->orientation = tmp_orientation;
    sem_post(mutex_sem_s2c);

    // notify about new data
    sem_post(spool_sem_s2c);
    
    return EXIT_SUCCESS;
}
