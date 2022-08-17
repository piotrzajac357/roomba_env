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

#include "../include/control_from_sim.h"
#include "../include/control_functions.h"
#include "../include/control.h"

/* initialize data from sim receiving thread */
int init_control_from_sim() {

    int status;
    pthread_t CFSThread;
    pthread_attr_t aCFSThreadAttr;
    pthread_attr_init(&aCFSThreadAttr);
    pthread_attr_setschedpolicy(&aCFSThreadAttr, SCHED_FIFO);

    if ((status = pthread_create(&CFSThread, &aCFSThreadAttr, tCFSThreadFunc, NULL))) {
        fprintf(stderr, "Cannot create thread.\n");
    return 0;
	}

    return EXIT_SUCCESS;
}

/* Sim -> control data receiving thread function */
void *tCFSThreadFunc(void *cookie) {

    int policy = SCHED_FIFO;
    struct sched_param param;
    
    pthread_getschedparam(pthread_self(), &policy, &param);
	param.sched_priority = sched_get_priority_max(policy) - 2;
    pthread_setschedparam(pthread_self(), policy, &param);

    // infinite loop waiting for data
    for(;;) {
        // notifications from sim
        sem_wait(spool_sem_s2c);

        // read data from shared memory and write to tmp variables
        sem_wait(mutex_sem_s2c);
        double front_sensor_tmp = s2c_shm_ptr->front_sensor;
        double back_sensor_tmp = s2c_shm_ptr->back_sensor;
        double left_sensor_tmp = s2c_shm_ptr->left_sensor;
        double right_sensor_tmp = s2c_shm_ptr->right_sensor;
        double battery_level_tmp = s2c_shm_ptr->battery_level;
        double position_x_tmp = s2c_shm_ptr->position_x;
        double position_y_tmp = s2c_shm_ptr->position_y;
        double orientation_tmp = s2c_shm_ptr->orientation;
        sem_post(mutex_sem_s2c);

        // write data from tmp variables to global process memory
        sem_wait(&batterySemaphore);
        battery_level = battery_level_tmp;
        sem_post(&batterySemaphore);

        sem_wait(&dist_sensorsSemaphore);
        front_sensor = front_sensor_tmp;
        back_sensor = back_sensor_tmp;
        left_sensor = left_sensor_tmp;
        right_sensor = right_sensor_tmp;
        sem_post(&dist_sensorsSemaphore);
        sem_wait(&position_orientationSemaphore);
        // position_x = round(1000*position_x_tmp)/1000;
        // position_y = round(1000*position_y_tmp)/1000;
        position_x = position_x_tmp;
        position_y = position_y_tmp;

        orientation = orientation_tmp;
        sem_post(&position_orientationSemaphore);
    }
    return 0;
}
