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
#include "../include/control_functions.h"

/* initialize stc algorithm thread */

int init_stc_algorithm(){

    int status;

    pthread_t StcThread;
    pthread_attr_t aStcThreadAttr;
    pthread_attr_init(&aStcThreadAttr);
    pthread_attr_setschedpolicy(&aStcThreadAttr, SCHED_FIFO);

    status = init_stc();
    FILE *fptr2;
    fptr2 = fopen("../../roomba/log/decisions.txt","w");
    fclose(fptr2);
    FILE *fptr3;
    fptr3 = fopen("../../roomba/log/quarters.txt","w");
    fclose(fptr3);

    //status = calc_next_step();

    if ((status = pthread_create(&StcThread, &aStcThreadAttr, tStcThreadFunc, NULL))) {
        fprintf(stderr, "Cannot create thread.\n");
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

    usleep(1000000);
    starting_x = position_x;
    starting_y = position_y;
    parent_cells[(int)(round(position_x*2))][(int)(round(position_y*2))] = 1;
    //status = init_stc();
    printf("%f %f",starting_x,starting_y);
    status = update_position_orientation();
    status = check_nbh();
    status = select_target_cell();
    //target_cell = 3;

    status = calc_next_step();
    // for(;;) {
    // sem_wait(&spool_calc_next_step_stc);
    // printf("decreased semaphore\n");
    // fflush(stdout);
    // sem_wait(&calc_next_step_stcSemaphore);
    // status = calc_next_step();
    // sem_post(&calc_next_step_stcSemaphore);
    // }


    return EXIT_SUCCESS;
}



int init_stc(void) {
    
    // current_position_x = 250;
    // current_position_y = 250;
    current_orientation = 0;
    current_quarter = 0;
    target_cell = 1;
    next_step = 0;
    //parent_cells[(int)(round(position_x*2))][(int)(round(position_y*2))] = 1;
    return EXIT_SUCCESS;
}

int update_position_orientation(){
    //current_position_x++;
    //current_position_y++;
    return EXIT_SUCCESS;
}


int check_nbh(void) {

    double i = 0;
    double j = 0;
    double f, b, r, l = 0;
    switch (current_quarter){
        case 0: { i = 0;     j = 0;     f = 0.025; l = 0.025;}
        case 1: { i = 0;     j = 0.25;  b = 0.025; l = 0.025;}
        case 2: { i = -0.25; j = 0.25;  b = 0.025; r = 0.025;}
        case 3: { i = -0.25; j = 0;     f = 0.025; r = 0.025;}
    }

    sem_wait(&dist_sensorsSemaphore);
    double tmp_front_sensor = front_sensor;
    double tmp_back_sensor = back_sensor;
    double tmp_left_sensor = left_sensor;
    double tmp_right_sensor = right_sensor;
    sem_post(&dist_sensorsSemaphore);

    current_position_x = (int)(round((position_x+i)*2));
    current_position_y = (int)(round((position_y+j)*2));

    //printf("\nsensors: %f %f %f %f\n", tmp_front_sensor, tmp_left_sensor,tmp_back_sensor,tmp_right_sensor);

    double f_s;
    double b_s;
    double r_s;
    double l_s;

    if (parent_cells[current_position_x][current_position_y] == 0){
        parent_cells[current_position_x][current_position_y] = target_cell + 1;
    }

    FILE *fptr;
    char c;
    fptr = fopen("../../roomba/log/map_parent.txt","w");
    for(int i = 0; i < 40; i++) {
        for(int j = 0; j < 40; j++){
            c = parent_cells[j][40-i] +'0';
            fputc(c,fptr);
        }
        fprintf(fptr,"\r\n");
    }
    fclose(fptr);


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
 

    // if (disc_plan[current_position_x][current_position_y-1] == 0){
    //     if (f_s <= 0.1){disc_plan[current_position_x][current_position_y-1] = 3;}
    //     else{disc_plan[current_position_x][current_position_y-1] = 1;}
    // }
    // if (disc_plan[current_position_x+1][current_position_y] == 0){
    //     if (l_s <= 0.1){disc_plan[current_position_x+1][current_position_y] = 3;}
    //     else{disc_plan[current_position_x+1][current_position_y] = 1;}
    // }
    // if (disc_plan[current_position_x][current_position_y+1] == 0){
    //     if (b_s <= 0.1){disc_plan[current_position_x][current_position_y+1] = 3;}
    //     else{disc_plan[current_position_x][current_position_y+1] = 1;}
    // }
    // if (disc_plan[current_position_x-1][current_position_y] == 0){
    //     if (r_s <= 0.1){disc_plan[current_position_x-1][current_position_y] = 3;}
    //     else{disc_plan[current_position_x-1][current_position_y] = 1;}
    // }
    // disc_plan[current_position_x][current_position_y] = 2;



    if (disc_plan2[(int)(round((position_x+i)*2))][(int)(round((position_y+j)*2)) - 1] <= 1){
        if (f_s <= (0.05+f)){disc_plan2[(int)(round((position_x+i)*2))][(int)(round((position_y+j)*2)) - 1] = 3;}
        else {disc_plan2[(int)(round((position_x+i)*2))][(int)(round((position_y+j)*2)) - 1] = 1;}
    }
    if (disc_plan2[(int)(round((position_x+i)*2)+1)][(int)(round((position_y+j)*2))] <= 1){
        if (l_s <= (0.05+l)){disc_plan2[(int)(round((position_x+i)*2)+1)][(int)(round((position_y+j)*2))] = 3;}
        else {disc_plan2[(int)(round((position_x+i)*2)+1)][(int)(round((position_y+j)*2))] = 1;}
    }
    if (disc_plan2[(int)(round((position_x+i)*2))][(int)(round((position_y+j)*2)) + 1] <= 1){
        if (b_s <= (0.05+b)){disc_plan2[(int)(round((position_x+i)*2))][(int)(round((position_y+j)*2)) + 1] = 3;}
        else {disc_plan2[(int)(round((position_x+i)*2))][(int)(round((position_y+j)*2)) + 1] = 1;}
    }
    if (disc_plan2[(int)(round((position_x+i)*2)-1)][(int)(round((position_y+j)*2))] <= 1){
        if (r_s <= (0.05+r)){disc_plan2[(int)(round((position_x+i)*2)-1)][(int)(round((position_y+j)*2))] = 3;}
        else {disc_plan2[(int)(round((position_x+i)*2)-1)][(int)(round((position_y+j)*2))] = 1;}
    }
    disc_plan2[(int)(round((position_x+i)*2))][(int)(round((position_y+j)*2))] = 2;
    //parent_cells[(int)(round(position_x*2))][(int)(round(position_y*2))] = 1;
    

    FILE *fptr1;
    char cc;
    fptr1 = fopen("../../roomba/log/map.txt","w");
    for(int i = 0; i < 40; i++) {
        for(int j = 0; j < 40; j++){
            cc = disc_plan2[j][40-i] +'0';
            fputc(cc,fptr1);
        }
        fprintf(fptr1,"\r\n");
    }
    fclose(fptr1);



    //printf("front sensor:%f\n",front_sensor);
    // printf("disc_plan[x][y]: %d\n",disc_plan[current_position_x][current_position_y]);
    // printf("disc_plan[x+1][y]: %d\n",disc_plan[current_position_x+1][current_position_y]);
    // printf("disc_plan[x][y+1]: %d\n",disc_plan[current_position_x][current_position_y+1]);
    // printf("disc_plan[x-1][y]: %d\n",disc_plan[current_position_x-1][current_position_y]);
    // printf("disc_plan[x][y-1]: %d\n",disc_plan[current_position_x][current_position_y-1]);
 
    // for (int i=247;i<253;i++){
    //     for (int j=247;j<253;j++){
    //         printf("%d",disc_plan[j][i]);
    //     }
    //     printf("\n");
    // }
    // printf("\n\n");

    // FILE *fptr;
    // char c;
    // fptr = fopen("../../roomba/log/map.txt","w");
    // for(int i = 0; i < 500; i++) {
    //     for(int j = 0; j < 500; j++){
    //         c = disc_plan[j][500-i] +'0';
    //         fputc(c,fptr);
    //     }
    //     fprintf(fptr,"\r\n");
    // }
    // fclose(fptr);


    return EXIT_SUCCESS;
}


int select_target_cell(void){
    // if (current_orientation == 0){
    //     if (disc_plan[current_position_x-1][current_position_y] == 1) {target_cell = 1;}
    //     else if (disc_plan[current_position_x][current_position_y-1] == 1) {target_cell = 2;}
    //     else if (disc_plan[current_position_x+1][current_position_y] == 1) {target_cell = 3;}
    //     else if (disc_plan[current_position_x-1][current_position_y] == 2) {target_cell = 1;}
    //     else if (disc_plan[current_position_x][current_position_y-1] == 2) {target_cell = 2;}
    //     else if (disc_plan[current_position_x+1][current_position_y] == 2) {target_cell = 3;}
    //     else {target_cell = 0;}
    // }
    // else if (current_orientation == 1){
    //     if (disc_plan[current_position_x][current_position_y-1] == 1) {target_cell = 2;}
    //     else if (disc_plan[current_position_x+1][current_position_y] == 1) {target_cell = 3;}
    //     else if (disc_plan[current_position_x][current_position_y+1] == 1) {target_cell = 0;}
    //     else if (disc_plan[current_position_x][current_position_y-1] == 2) {target_cell = 2;}
    //     else if (disc_plan[current_position_x+1][current_position_y] == 2) {target_cell = 3;}
    //     else if (disc_plan[current_position_x][current_position_y+1] == 2) {target_cell = 0;}
    //     else {target_cell = 1;}
    // }
    // else if (current_orientation == 2){
    //     if (disc_plan[current_position_x+1][current_position_y] == 1) {target_cell = 3;}
    //     else if (disc_plan[current_position_x][current_position_y+1] == 1) {target_cell = 0;}
    //     else if (disc_plan[current_position_x-1][current_position_y] == 1) {target_cell = 1;}
    //     else if (disc_plan[current_position_x+1][current_position_y] == 2) {target_cell = 3;}
    //     else if (disc_plan[current_position_x][current_position_y+1] == 2) {target_cell = 0;}
    //     else if (disc_plan[current_position_x-1][current_position_y] == 2) {target_cell = 1;}
    //     else {target_cell = 2;}
    // }
    // else if (current_orientation == 3){
    //     if (disc_plan[current_position_x][current_position_y+1] == 1) {target_cell = 0;}
    //     else if (disc_plan[current_position_x-1][current_position_y] == 1) {target_cell = 1;}
    //     else if (disc_plan[current_position_x][current_position_y-1] == 1) {target_cell = 2;}
    //     else if (disc_plan[current_position_x][current_position_y+1] == 2) {target_cell = 0;}
    //     else if (disc_plan[current_position_x-1][current_position_y] == 2) {target_cell = 1;}
    //     else if (disc_plan[current_position_x][current_position_y-1] == 2) {target_cell = 2;}
    //     else {target_cell = 3;}
    // }

    // current_position_x = (int)(round(position_x*2));
    // current_position_y = (int)(round(position_y*2));
    if (current_orientation == 0){
        if (disc_plan2[current_position_x-1][current_position_y] == 1) {target_cell = 1;}
        else if (disc_plan2[current_position_x][current_position_y-1] == 1) {target_cell = 2;}
        else if (disc_plan2[current_position_x+1][current_position_y] == 1) {target_cell = 3;}
        else {target_cell = parent_to_target(parent_cells[current_position_x][current_position_y]);}
        // else if (disc_plan2[current_position_x-1][current_position_y] == 2) {target_cell = 1;}
        // else if (disc_plan2[current_position_x][current_position_y-1] == 2) {target_cell = 2;}
        // else if (disc_plan2[current_position_x+1][current_position_y] == 2) {target_cell = 3;}
        //else {target_cell = 0;}
    }
    else if (current_orientation == 1){
        if (disc_plan2[current_position_x][current_position_y-1] == 1) {target_cell = 2;}
        else if (disc_plan2[current_position_x+1][current_position_y] == 1) {target_cell = 3;}
        else if (disc_plan2[current_position_x][current_position_y+1] == 1) {target_cell = 0;}
        else {target_cell = parent_to_target(parent_cells[current_position_x][current_position_y]);}
        // else if (disc_plan2[current_position_x][current_position_y-1] == 2) {target_cell = 2;}
        // else if (disc_plan2[current_position_x+1][current_position_y] == 2) {target_cell = 3;}
        // else if (disc_plan2[current_position_x][current_position_y+1] == 2) {target_cell = 0;}
        // else {target_cell = 1;}
    }
    else if (current_orientation == 2){
        if (disc_plan2[current_position_x+1][current_position_y] == 1) {target_cell = 3;}
        else if (disc_plan2[current_position_x][current_position_y+1] == 1) {target_cell = 0;}
        else if (disc_plan2[current_position_x-1][current_position_y] == 1) {target_cell = 1;}
        else {target_cell = parent_to_target(parent_cells[current_position_x][current_position_y]);}
        // else if (disc_plan2[current_position_x+1][current_position_y] == 2) {target_cell = 3;}
        // else if (disc_plan2[current_position_x][current_position_y+1] == 2) {target_cell = 0;}
        // else if (disc_plan2[current_position_x-1][current_position_y] == 2) {target_cell = 1;}
        // else {target_cell = 2;}
    }
    else if (current_orientation == 3){
        if (disc_plan2[current_position_x][current_position_y+1] == 1) {target_cell = 0;}
        else if (disc_plan2[current_position_x-1][current_position_y] == 1) {target_cell = 1;}
        else if (disc_plan2[current_position_x][current_position_y-1] == 1) {target_cell = 2;}
        else {target_cell = parent_to_target(parent_cells[current_position_x][current_position_y]);}
        // else if (disc_plan2[current_position_x][current_position_y+1] == 2) {target_cell = 0;}
        // else if (disc_plan2[current_position_x-1][current_position_y] == 2) {target_cell = 1;}
        // else if (disc_plan2[current_position_x][current_position_y-1] == 2) {target_cell = 2;}
        // else {target_cell = 3;}
    }


    //printf("\n");
    //printf("target_cell: %d  ", target_cell);
    FILE *fptr2;
    char ccc;
    fptr2 = fopen("../../roomba/log/decisions.txt","a");
    fprintf(fptr2,"pos_x: %f pos_y: %f orientation: %f\r\n", position_x, position_y, orientation);
    fprintf(fptr2,"up: %d left: %d down: %d right: %d, parent %d, target: %d\r\n",
                disc_plan2[current_position_x][current_position_y+1],
                disc_plan2[current_position_x-1][current_position_y],
                disc_plan2[current_position_x][current_position_y-1],
                disc_plan2[current_position_x+1][current_position_y],
                parent_cells[current_position_x][current_position_y], target_cell);
    fprintf(fptr2,"front: %.5f left: %.5f back: %.5f right: %.5f\r\n",front_sensor,left_sensor,back_sensor,right_sensor); 
    fprintf(fptr2,"\r\n");
    fclose(fptr2);
    return EXIT_SUCCESS;
}


int calc_next_step(){

    int status;
    // printf("decreased semaphore\n");
    // fflush(stdout);
    status = update_quarter_and_cell();

    tmp_orientation_stc_step = orientation;
    tmp_pos_x_stc_step = position_x;
    tmp_pos_y_stc_step = position_y;
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
    if (next_step == 1){
        // if (tmp_orientation_stc_step >= 270.0){
        //     tmp_orientation_stc_step -= 270.0;
        // }
        // else {tmp_orientation_stc_step += 90.0;}
        switch (current_orientation){
            case 0:
                tmp_orientation_stc_step = 0.0;
                break;
            case 1:
                tmp_orientation_stc_step = 90.0;
                break;
            case 2:
                tmp_orientation_stc_step = 180.0;
                break;
            case 3:
                tmp_orientation_stc_step = 270.0;
                break;
        }
        // int a = tmp_orientation_stc_step + 90;
        // tmp_orientation_stc_step = a % 360;
    }
    else if (next_step == 2) {
        // TODO zrobic cos z rownymi skretami
        switch (current_orientation){
            case 0:
                tmp_orientation_stc_step = 180.0;
                break;
            case 1:
                tmp_orientation_stc_step = 270.0;
                break;
            case 2:
                tmp_orientation_stc_step = 0.0;
                break;
            case 3:
                tmp_orientation_stc_step = 90.0;
                break;
        }

        // if (tmp_orientation_stc_step <= 90.0){
        //     tmp_orientation_stc_step += 270.0;
        // }
        //  else {tmp_orientation_stc_step -= 90.0;}
        //printf("xd tmp_orient: %f  desired: %f\n", orientation,tmp_orientation_stc_step);
        // int a = tmp_orientation_stc_step + 270;
        // tmp_orientation_stc_step = a % 360;
        //printf("xd tmp_orient: %f  desired: %f\n", orientation,tmp_orientation_stc_step);

    }
    
    // printf("current quarter: %d\n", current_quarter);
    // printf("target cell: %d\n", target_cell);
    // printf("current orientation: %d\n",current_orientation);
    // printf("next step: %d   \n", next_step);
    // printf("algorithm:%d\n",algorithm_select);
    // printf("%f\n\n",tmp_orientation_stc_step);
    return EXIT_SUCCESS;
}

int update_quarter_and_cell() {
    
    int status;

    //printf("%d -> ",current_quarter);
    if (next_step == 3) {
        switch (current_quarter)
        {
            case 0:
                if (current_orientation == 0){
                    current_quarter += 1;
                }
                else if (current_orientation == 3) {
                    current_quarter = 3;
                    current_position_x -= 1;
                    status = check_nbh();
                    status = select_target_cell(); 
                }
                break;
            case 1:
                if (current_orientation == 1){
                    current_quarter += 1;
                }
                else if (current_orientation == 0) {
                    current_quarter = 0;
                    current_position_y -= 1;
                    status = check_nbh();
                    status = select_target_cell();
                }
                break;
            case 2:
                if (current_orientation == 2) {
                    current_quarter += 1;
                }
                else if (current_orientation == 1) {
                    current_quarter = 1;
                    current_position_x += 1;
                    status = check_nbh();
                    status = select_target_cell();
                }
                break;
            case 3:
                if (current_orientation == 3) {
                    current_quarter = 0;
                } 
                else if (current_orientation == 2) {
                    current_quarter = 2;
                    current_position_y += 1;
                    status = check_nbh();
                    status = select_target_cell();
                }
                break;
        }

    }
    FILE *fptr3;
    char q;
    fptr3 = fopen("../../roomba/log/quarters.txt","a");
    fprintf(fptr3,"pos_x: %.2f pos_y: %.2f orientation: %.2f next step: %d quarter: %d\r\n", position_x, position_y, orientation, next_step, current_quarter);
    fclose(fptr3);



    return EXIT_SUCCESS;
}

int parent_to_target(int parent_cell) {
    if (parent_cell == 1) { return 2;}
    else if (parent_cell == 2) {return 3;}
    else if (parent_cell == 3) {return 0;}
    else if (parent_cell == 4) {return 1;}
    else { return 0;}
}