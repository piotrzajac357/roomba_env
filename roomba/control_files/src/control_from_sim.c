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

    // variables for determing whether position and new trashes coords changed since last time
    int previous_x_pix = -1;
    int previous_y_pix = -1;
    int new_trashes_coords[2] = {0,0};

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
        int trash_sensor_tmp = s2c_shm_ptr->trash_sensor;
        double battery_level_tmp = s2c_shm_ptr->battery_level;
        double container_level_tmp = s2c_shm_ptr->container_level;
        double position_x_tmp = s2c_shm_ptr->position_x;
        double position_y_tmp = s2c_shm_ptr->position_y;
        double orientation_tmp = s2c_shm_ptr->orientation;
        // int new_trashes_coords_tmp[2] = {s2c_shm_ptr->new_trashes_coords[0],s2c_shm_ptr->new_trashes_coords[1]};
        sem_post(mutex_sem_s2c);

        // write data from tmp variables to global process memory
        sem_wait(&batterySemaphore);
        battery_level = battery_level_tmp;
        sem_post(&batterySemaphore);
        sem_wait(&containerSemaphore);
        container_level = container_level_tmp;
        sem_post(&containerSemaphore);
        sem_wait(&dist_sensorsSemaphore);
        front_sensor = front_sensor_tmp;
        back_sensor = back_sensor_tmp;
        left_sensor = left_sensor_tmp;
        right_sensor = right_sensor_tmp;
        sem_post(&dist_sensorsSemaphore);
        sem_wait(&position_orientationSemaphore);
        position_x = position_x_tmp;
        position_y = position_y_tmp;
        orientation = orientation_tmp;
        sem_post(&position_orientationSemaphore);
        sem_wait(&trashSemaphore);
        trash_sensor = trash_sensor_tmp;
        sem_post(&trashSemaphore);
          
        // function updates plan (visited points)
        //int status = update_plan(previous_x_pix, previous_y_pix, round(10*position_x_tmp), round(10*position_y_tmp));
        
        // if new trashes coords are different since last, insert trashes into plan 
        /* sem_wait(&planSemaphore);
        if ((new_trashes_coords_tmp[0] != new_trashes_coords[0]) &&
            new_trashes_coords_tmp[1] != new_trashes_coords[1]){
                for (int i = -4; i <= 4; i++) {
                    for (int j = -4; j <= 4; j++){
                        plan[new_trashes_coords_tmp[0]+i][new_trashes_coords_tmp[1]+j] = '3';
                    }
                }
        }
        sem_post(&planSemaphore);
        */
       
        // update previous coords for next iteration
        previous_x_pix = round(10 * position_x_tmp);
        previous_y_pix = round(10 * position_y_tmp);
        // new_trashes_coords[0] = new_trashes_coords_tmp[0];
        // new_trashes_coords[1] = new_trashes_coords_tmp[1];
    }
    return 0;
}
