#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <math.h>
#include <semaphore.h>
#include <unistd.h>
#include <sys/types.h>
#include <signal.h>
#include <fcntl.h>
#include <time.h>
#include <errno.h>

#include "../include/stc_algorithm.h"

/* initialize stc algorithm thread */

int init_stc_algorithm(){

    int status;

    pthread_attr_t aStcThreadAttr;
    struct itimerspec timerSpecStruct;
    timer_t timerVar;
    struct sigevent timerEvent;
    pthread_attr_init(&aStcThreadAttr);

    timerEvent.sigev_notify = SIGEV_THREAD;
    timerEvent.sigev_notify_function = (void*)tStcThreadFunc;
	timerEvent.sigev_notify_attributes = &aStcThreadAttr;

    if ((status = timer_create(CLOCK_REALTIME, &timerEvent, &timerVar))) {
        fprintf(stderr, "Error creating timer : %d\n", status);
    return 0;
    }

    timerSpecStruct.it_value.tv_sec = 0;
	timerSpecStruct.it_value.tv_nsec = 50000000;
	timerSpecStruct.it_interval.tv_sec = 1;
	timerSpecStruct.it_interval.tv_nsec = 0;

    status = init_stc();
    timer_settime(timerVar, 0, &timerSpecStruct, NULL);

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

    status = update_position_orientation();
    status = check_nbh();

    return EXIT_SUCCESS;
}



int init_stc(void) {
    
    current_position_x = 250;
    current_position_y = 250;
    current_orientation = 0;
    return EXIT_SUCCESS;
}

int update_position_orientation(){
    current_position_x++;
    current_position_y++;
    return EXIT_SUCCESS;
}


int check_nbh(void) {

    sem_wait(&dist_sensorsSemaphore);
    double tmp_front_sensor = front_sensor;
    double tmp_back_sensor = back_sensor;
    double tmp_left_sensor = left_sensor;
    double tmp_right_sensor = right_sensor;
    sem_post(&dist_sensorsSemaphore);

    double f_s;
    double b_s;
    double r_s;
    double l_s;

    if (current_orientation == 0){
        f_s = tmp_front_sensor;
        b_s = tmp_back_sensor;
        l_s = tmp_left_sensor;
        r_s = tmp_right_sensor;
        }
    else if (current_orientation == 1) {
        f_s = tmp_right_sensor;
        b_s = tmp_left_sensor;
        l_s = tmp_front_sensor;
        r_s = tmp_back_sensor;
        }
    else if (current_orientation == 2) {
        f_s = tmp_back_sensor;
        b_s = tmp_front_sensor;
        l_s = tmp_right_sensor;
        r_s = tmp_left_sensor;
        }
    else if (current_orientation == 3) {
        f_s = tmp_left_sensor;
        b_s = tmp_right_sensor;
        l_s = tmp_back_sensor;
        r_s = tmp_front_sensor;
        }
 

    if (disc_plan[current_position_x][current_position_y-1] == 0){
        if (f_s <= 0.02){
            disc_plan[current_position_x][current_position_y-1] = 3;
        }
        else{
            disc_plan[current_position_x][current_position_y-1] = 1;
        }
    }
    if (disc_plan[current_position_x+1][current_position_y] == 0){
        if (l_s <= 0.02){
            disc_plan[current_position_x+1][current_position_y] = 3;
        }
        else{
            disc_plan[current_position_x+1][current_position_y] = 1;
        }
    }
    if (disc_plan[current_position_x][current_position_y+1] == 0){
        if (b_s <= 0.02){
            disc_plan[current_position_x][current_position_y+1] = 3;
        }
        else{
            disc_plan[current_position_x][current_position_y+1] = 1;
        }
    }
    if (disc_plan[current_position_x-1][current_position_y] == 0){
        if (r_s <= 0.02){
            disc_plan[current_position_x-1][current_position_y] = 3;
        }
        else{
            disc_plan[current_position_x-1][current_position_y] = 1;
        }
    }
    disc_plan[current_position_x][current_position_y] = 2;

    //printf("front sensor:%f\n",front_sensor);
    // printf("disc_plan[x][y]: %d\n",disc_plan[current_position_x][current_position_y]);
    // printf("disc_plan[x+1][y]: %d\n",disc_plan[current_position_x+1][current_position_y]);
    // printf("disc_plan[x][y+1]: %d\n",disc_plan[current_position_x][current_position_y+1]);
    // printf("disc_plan[x-1][y]: %d\n",disc_plan[current_position_x-1][current_position_y]);
    // printf("disc_plan[x][y-1]: %d\n",disc_plan[current_position_x][current_position_y-1]);
 
    for (int i=250;i<260;i++){
        for (int j=250;j<260;j++){
            printf("%d",disc_plan[j][i]);
        }
        printf("\n");
    }
    printf("\n\n");

    return EXIT_SUCCESS;
}


int select_target_cell(void){
    if (current_orientation == 0){
        if (disc_plan[current_position_x-1][current_position_y] == 1) {target_cell = 1;}
        else if (disc_plan[current_position_x][current_position_y-1] == 1) {target_cell = 2;}
        else if (disc_plan[current_position_x+1][current_position_y] == 1) {target_cell = 3;}
        else if (disc_plan[current_position_x-1][current_position_y] == 2) {target_cell = 1;}
        else if (disc_plan[current_position_x][current_position_y-1] == 2) {target_cell = 2;}
        else if (disc_plan[current_position_x+1][current_position_y] == 2) {target_cell = 3;}
        else {target_cell = 0;}
    }
    else if (current_orientation == 1){
        if (disc_plan[current_position_x][current_position_y-1] == 1) {target_cell = 2;}
        else if (disc_plan[current_position_x+1][current_position_y] == 1) {target_cell = 3;}
        else if (disc_plan[current_position_x][current_position_y+1] == 1) {target_cell = 0;}
        else if (disc_plan[current_position_x][current_position_y-1] == 2) {target_cell = 2;}
        else if (disc_plan[current_position_x+1][current_position_y] == 2) {target_cell = 3;}
        else if (disc_plan[current_position_x][current_position_y+1] == 2) {target_cell = 0;}
        else {target_cell = 1;}
    }
    else if (current_orientation == 2){
        if (disc_plan[current_position_x+1][current_position_y] == 1) {target_cell = 3;}
        else if (disc_plan[current_position_x][current_position_y+1] == 1) {target_cell = 0;}
        else if (disc_plan[current_position_x-1][current_position_y] == 1) {target_cell = 1;}
        else if (disc_plan[current_position_x+1][current_position_y] == 2) {target_cell = 3;}
        else if (disc_plan[current_position_x][current_position_y+1] == 2) {target_cell = 0;}
        else if (disc_plan[current_position_x-1][current_position_y] == 2) {target_cell = 1;}
        else {target_cell = 2;}
    }
    else if (current_orientation == 3){
        if (disc_plan[current_position_x][current_position_y+1] == 1) {target_cell = 0;}
        else if (disc_plan[current_position_x-1][current_position_y] == 1) {target_cell = 1;}
        else if (disc_plan[current_position_x][current_position_y-1] == 1) {target_cell = 2;}
        else if (disc_plan[current_position_x][current_position_y+1] == 2) {target_cell = 0;}
        else if (disc_plan[current_position_x-1][current_position_y] == 2) {target_cell = 1;}
        else if (disc_plan[current_position_x][current_position_y-1] == 2) {target_cell = 2;}
        else {target_cell = 3;}
    }

    return EXIT_SUCCESS;
}


int calc_next_step(){
    if (current_quarter == 0){
        if (current_orientation == 0){
            if (target_cell == 1) { next_step = 2;}
            else { next_step = 3;}
        }
        else if (current_orientation == 3){
            if (target_cell == 1) { next_step = 3;}
            else { next_step = 1;}
        }
    }
    else if (current_quarter == 1){
        if (current_orientation == 1){
            if (target_cell == 2) { next_step = 2;}
            else { next_step = 3;}
        }
        else if (current_orientation == 0){
            if (target_cell == 2) { next_step = 3;}
            else { next_step = 1;}
        }
    }
    else if (current_quarter == 2){
        if (current_orientation == 2){
            if (target_cell == 3) { next_step = 2;}
            else { next_step = 3;}
        }
        else if (current_orientation == 1){
            if (target_cell == 3) { next_step = 3;}
            else { next_step = 1;}
        }
    }
    else if (current_quarter == 3){
        if (current_orientation == 3){
            if (target_cell == 0) { next_step = 2;}
            else { next_step = 3;}
        }
        else if (current_orientation == 2){
            if (target_cell == 0) { next_step = 3;}
            else { next_step = 1;}
        }
    }

    return EXIT_SUCCESS;
}
