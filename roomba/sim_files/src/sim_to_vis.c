#include <stdlib.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#include <semaphore.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include <pthread.h>

#include "../include/sim_to_vis.h"
#include "../include/sim_functions.h"

/* UDP communication with visualisation process */

/* initialization of simulation -> visualisation data sending thread */
int init_sim_to_vis() {

        int status;
        pthread_attr_t aS2VThreadAttr;
        struct itimerspec timerSpecStruct;
        timer_t	timerVar;
        struct	sigevent timerEvent;

        pthread_attr_init(&aS2VThreadAttr);

        struct sockaddr_in socket_addr;
        
 	    timerEvent.sigev_notify = SIGEV_THREAD;
        timerEvent.sigev_notify_function = (void*)tS2VThreadFunc;
	    timerEvent.sigev_notify_attributes = &aS2VThreadAttr;

        if ((status = timer_create(CLOCK_REALTIME, &timerEvent, &timerVar))) {
  		    fprintf(stderr, "Error creating timer : %d\n", status);
   		return 0;
        }

        // visualization upgrade rate - 40/sec
      	timerSpecStruct.it_value.tv_sec = 0;
	    timerSpecStruct.it_value.tv_nsec = 25000000;
	    timerSpecStruct.it_interval.tv_sec = 0;
    	timerSpecStruct.it_interval.tv_nsec = 25000000;

        // create and configure socket
        my_socket = socket(PF_INET, SOCK_DGRAM, 0);
        if (my_socket == -1) {
            fprintf(stderr, "Cannot create socket\n");
            return 0;
        }

        memset(&socket_addr, 0, sizeof(socket_addr));
        socket_addr.sin_family = AF_INET;
        socket_addr.sin_port = htons(1100);
        socket_addr.sin_addr.s_addr = INADDR_ANY;


        timer_settime(timerVar, 0, &timerSpecStruct, NULL);

        return EXIT_SUCCESS;
}

/* Simulation -> Visualisation sending data thread function */
void * tS2VThreadFunc(void *cookie) {

    int status;
    int policy = SCHED_FIFO;
    struct sched_param param;
    char buffer[1024]; 
    struct sockaddr_in socket_addr;

    pthread_getschedparam(pthread_self(), &policy, &param);
    param.sched_priority = sched_get_priority_min(policy) + 1;
    pthread_setschedparam(pthread_self(), policy, &param);

    socket_addr.sin_family = AF_INET;
    socket_addr.sin_port = htons(1100);
    socket_addr.sin_addr.s_addr = INADDR_ANY;
    
    /* read from process global memory */
    sem_wait(&position_orientationSemaphore);
    double tmp_position_x = position_x;
    double tmp_position_y = position_y;
    double tmp_orientation = previous_orientation;
    sem_post(&position_orientationSemaphore);
    
    sem_wait(&batterySemaphore);
    double tmp_battery = battery_level;
    sem_post(&batterySemaphore);

    sem_wait(&controlSemaphore);
    double tmp_suction_power = suction_power;
    sem_post(&controlSemaphore);

    sem_wait(&qiSemaphore);
    double tmp_time_qi = time_QI;
    double tmp_path_qi = path_QI;
    double tmp_rotation_qi = rotation_QI;
    double tmp_coverage_qi = coverage_QI;
    double tmp_idle_time_qi = idle_time_QI;
    double tmp_battery_qi = battery_QI;
    sem_post(&qiSemaphore);

    // write data to buffer
    
    // buffer with trash coords 
    // snprintf(buffer, sizeof(buffer), "%f %f %f %f %f %d %d %f ", tmp_position_x, tmp_position_y, tmp_orientation, tmp_battery, tmp_container, trashes[0], trashes[1], tmp_suction_power);
    snprintf(buffer, sizeof(buffer), "%f %f %f %f %f %f %f %f %d %f  ", tmp_position_x, tmp_position_y, tmp_orientation, tmp_battery_qi, tmp_time_qi, tmp_path_qi, tmp_rotation_qi, tmp_coverage_qi, algorithm_finished, tmp_suction_power);

    // send buffer 
    sendto(my_socket, buffer, 100, MSG_CONFIRM, (const struct sockaddr *) &socket_addr, sizeof(socket_addr));

    // no answer from vis needed
    return EXIT_SUCCESS;
}
        