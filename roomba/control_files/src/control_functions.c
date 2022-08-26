#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <math.h>
#include <stdint.h>
#include <inttypes.h>
#include <semaphore.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <fcntl.h>
#include <string.h>
#include <bits/pthreadtypes.h>

#include "../include/astar/astar.uint8_t.h"
#include "../include/astar/grid.uint8_t.h"
#include "../include/astar/path_main.h"

#include "../include/control_functions.h"

/* function initializing semaphores */
int initialize_semaphores(void) {
	if(	sem_init(&batterySemaphore, 0, 1) 				||
		sem_init(&position_orientationSemaphore, 0, 1)	||
		sem_init(&dist_sensorsSemaphore, 0, 1) 			||
		sem_init(&controlSemaphore, 0, 1) 				||
		sem_init(&taskSemaphore, 0, 1)					||
		sem_init(&movement_typeSemaphore, 0, 1)			||
		sem_init(&target_directionSemaphore, 0, 1)		||
		sem_init(&spool_calc_next_step_stc, 0, 0)		||
		sem_init(&spool_calc_next_step_ba, 0, 0)		||
		sem_init(&spool_calc_next_step_rg, 0, 0)		||
		sem_init(&spool_calc_next_step_swf, 0, 0)		||
		sem_init(&virtual_sensorsSemaphore, 0, 1)		||
		sem_init(&superv_calc_swf_Semaphore, 0, 1)		||
		sem_init(&swf_planSemaphore, 0, 1)				||
		sem_init(&superv_calc_rg_Semaphore, 0, 1)		||
		sem_init(&superv_calc_stc_Semaphore, 0, 1)		||
		sem_init(&superv_calc_ba_Semaphore, 0, 1))
		{
			return EXIT_FAILURE;
		}
	return EXIT_SUCCESS;
}

/* function setting starting control to 0 */
int init_control(void) {
	sem_wait(&controlSemaphore);
	left_motor_power = 0.0;
	right_motor_power = 0.0;
	suction_power = 0.0;
	sem_post(&controlSemaphore);
	return EXIT_SUCCESS;
}

/* function setting starting task to 1 (drive forward) */
int init_rg(void) {
	current_task = 1;
	spool_next_step_rg_calculated = 1;
	return EXIT_SUCCESS;
}

/* function initializing starting movement type */
int init_movement_type(void) {
	sem_wait(&movement_typeSemaphore);
	movement_type = 0;
	sem_post(&movement_typeSemaphore);
	return EXIT_SUCCESS;
}

/* function calculating control based on movement type, trash sensor, container full flag and battery level */
int calculate_control(void){

	/*	function calculates motor power and suction power based on current movement type
	0 - stop, no driving, we calculating or sth
	1 - drive forward
	2 - rotating clockwise
	3 - rotating counterclockwise
	4 - drive backward
	*/
	// read input values

	// set control based on movement type 
	sem_wait(&controlSemaphore);
	int tmp_movement_type = movement_type;
	switch (tmp_movement_type)
	{
	case 0:
		// just standing
		left_motor_power = 0.0;
		right_motor_power = 0.0;
		break;
	case 1:
		// driving forward
		left_motor_power = 1.0;
		right_motor_power = 1.0;
		break;
	case 2:
		// rotating clockwise with suction 0.2
		left_motor_power = 1.0;
		right_motor_power = -1.0;
		break;
	case 3:
		// rotating counter clockwise with suction 0.2
		left_motor_power = -1.0;
		right_motor_power = 1.0;
		break;
	case 4:
		// driving backward 
		left_motor_power = -1.0;
		right_motor_power = -1.0;
		break;
	default:
		// default controls 0.0 for safety
		left_motor_power = 0.0;
		right_motor_power = 0.0;
		break;
	}
	suction_power = calculate_suction(algorithm_select, movement_type);
	sem_post(&controlSemaphore);
	return EXIT_SUCCESS;
}

/* function calculating suction power based on algorithm variables */
double calculate_suction(int alg_select, int tmp_movement_type){
	double tmp_suction;
	switch (alg_select){
		case 0:
			sem_wait(&position_orientationSemaphore);
			double tmp_pos_x = position_x;
			double tmp_pos_y = position_y;
			sem_post(&position_orientationSemaphore);
			if (rg_disc_map[(int)round(4*tmp_pos_x)][(int)round(4*tmp_pos_y)] != 2){
				tmp_suction = (tmp_movement_type == 1 || tmp_movement_type == 4) ? 1.0 : 0.5;
			} else {
				tmp_suction = 0.0;
			}
			break;
		case 1:
				tmp_suction = (tmp_movement_type == 1 || tmp_movement_type == 4) ? 1.0 : 0.5;
			break;
		case 2:
			if (movement_mode == 0){
				tmp_suction = (tmp_movement_type == 1 || tmp_movement_type == 4) ? 1.0 : 0.5;
			} else {
				tmp_suction = 0.0;
			}
			break;
		case 3:
			if (movement_mode == 0){
				tmp_suction = (tmp_movement_type == 1 || tmp_movement_type == 4) ? 1.0 : 0.5;
			}
			else {
				tmp_suction = 0.0;
			}
			break;
	}
	return tmp_suction;
}

/* function choosing the best new direction from random candidates  */
int select_new_direction(void) {
	// select new direction to drive
	// candidates are random, prefer directions with less visited points

	// read input values
	sem_wait(&position_orientationSemaphore);
	double tmp_orientation = orientation;
	double tmp_x = position_x;
	double tmp_y = position_y;
	sem_post(&position_orientationSemaphore);

	/* a few candidates without randomizing */
	/* double candidate_directions[6] = {(tmp_orientation+90.0)*ST_TO_RAD, (tmp_orientation-90.0)*ST_TO_RAD,
										(tmp_orientation+135.0)*ST_TO_RAD, (tmp_orientation-135.0)*ST_TO_RAD,
										(tmp_orientation+45.0)*ST_TO_RAD, (tmp_orientation-45.0)*ST_TO_RAD}; */
	
	/* random candidates */
	double candidate_directions[18] = 	{(rand()%359)*ST_TO_RAD, (rand()%359)*ST_TO_RAD, (rand()%359)*ST_TO_RAD,
										 (rand()%359)*ST_TO_RAD, (rand()%359)*ST_TO_RAD, (rand()%359)*ST_TO_RAD,
										 (rand()%359)*ST_TO_RAD, (rand()%359)*ST_TO_RAD, (rand()%359)*ST_TO_RAD,
										 (rand()%359)*ST_TO_RAD, (rand()%359)*ST_TO_RAD, (rand()%359)*ST_TO_RAD,
										 (rand()%359)*ST_TO_RAD, (rand()%359)*ST_TO_RAD, (rand()%359)*ST_TO_RAD,
										 (rand()%359)*ST_TO_RAD, (rand()%359)*ST_TO_RAD, (rand()%359)*ST_TO_RAD};
	
	// candidate grades to determine better ones
	double candidate_grades[18] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	// double candidate_grades[9] = {0,0,0,0,0,0,0,0,0};
	int best_candidate_index = 0;

	// calculate grade for each candidate
	for (int i = 0; i < 18; i++) {
		// flag to skip loop
		int flag = 0;
		// check the closest pixel for every candidate every 10cm in given direction
		// 10cm is most natural since 1 pixel represents 10x10cm square

		for (double j = 0.25; j < 10.0; j = j + 0.125){
			if(!flag){
				// determining the closest pixel
				int x_pix = round(4 * (tmp_x + j * cos(candidate_directions[i])));
				int y_pix = round(4 * (tmp_y + j * sin(candidate_directions[i])));
				if ((x_pix * y_pix <= 0) || x_pix > 78 || y_pix > 78) {
					break;
				}

				int x_pix_left =	(int)round(4 * ((tmp_x + j * cos(candidate_directions[i]) - 0.125)));
				int x_pix_right = 	(int)round(4 * ((tmp_x + j * cos(candidate_directions[i]) + 0.125)));
				int y_pix_up = 		(int)round(4 * ((tmp_y + j * sin(candidate_directions[i]) + 0.125)));
				int y_pix_down = 	(int)round(4 * ((tmp_y + j * sin(candidate_directions[i]) - 0.125)));

				// get that pixels value
				int pix_val = rg_disc_map[x_pix][y_pix];
				if 		(pix_val == 3) { flag = 1; }							// 3 - obstacle
				else if (pix_val == 1) { candidate_grades[i] += 1; }			// 1 - floor (not visited)
				else if (pix_val == 2) { candidate_grades[i] += 0.1; }			// 2 - floor (visited)
				else if (pix_val == 0) { candidate_grades[i] += 0.5; }			// 0 - not recognized
				else { continue; }

				// check also 4 neighbour pixels, because robot will visit them following that path
				int pix_val_left  = rg_disc_map[x_pix_left][y_pix];
				if 		(pix_val_left == 3) { flag = 1; }							// 3 - obstacle
				else if (pix_val_left == 1) { candidate_grades[i] += 1; }			// 1 - floor (not visited)
				else if (pix_val_left == 2) { candidate_grades[i] += 0.1; }			// 2 - floor (visited)
				else if (pix_val_left == 0) { candidate_grades[i] += 0.5; }			// 0 - not recognized
				else { continue; }

				int pix_val_right = rg_disc_map[x_pix_right][y_pix];
				if 		(pix_val_right == 3) { flag = 1; }							// 3 - obstacle
				else if (pix_val_right == 1) { candidate_grades[i] += 1; }			// 1 - floor (not visited)
				else if (pix_val_right == 2) { candidate_grades[i] += 0.1; }		// 2 - floor (visited)
				else if (pix_val_right == 0) { candidate_grades[i] += 0.5; }		// 0 - not recognized
				else { continue; }

				int pix_val_up    = rg_disc_map[x_pix][y_pix_up];
				if 		(pix_val_up == 3) { flag = 1; }								// 3 - obstacle
				else if (pix_val_up == 1) { candidate_grades[i] += 1; }				// 1 - floor (not visited)
				else if (pix_val_up == 2) { candidate_grades[i] += 0.1; }			// 2 - floor (visited)
				else if (pix_val_up == 0) { candidate_grades[i] += 0.5; }			// 0 - not recognized
				else { continue; }

				int pix_val_down  = rg_disc_map[x_pix][y_pix_down];
				if 		(pix_val_down == 3) { flag = 1; }							// 3 - obstacle
				else if (pix_val_down == 1) { candidate_grades[i] += 1; }			// 1 - floor (not visited)
				else if (pix_val_down == 2) { candidate_grades[i] += 0.1; }			// 2 - floor (visited)
				else if (pix_val_down == 0) { candidate_grades[i] += 0.5; }			// 0 - not recognized
				else { continue; }
			}
		}

		// change the best candidate if current is better
		if (candidate_grades[i] > candidate_grades[best_candidate_index]){
			best_candidate_index = i;
		}
	}			

	// calculate target direction in degrees and write to output
	sem_wait(&target_directionSemaphore);
	target_direction = (candidate_directions[best_candidate_index]) * 57.295779513;
	if (target_direction > 360) { target_direction -= 360; }
	else if (target_direction < 0) { target_direction += 360; }
	sem_post(&target_directionSemaphore);

	//printf("target_direction_selected: %.2f		grade: %f\n", target_direction, candidate_grades[best_candidate_index]);

	return EXIT_SUCCESS;
}

/* function updating rg map */
int update_rg_map(void) {

	int status;

	sem_wait(&position_orientationSemaphore);
	double tmp_orientation = orientation;
	double tmp_position_x = position_x;
	double tmp_position_y = position_y;
	sem_post(&position_orientationSemaphore);

	sem_wait(&dist_sensorsSemaphore);
	double tmp_front_sensor = front_sensor;
	double tmp_back_sensor  = back_sensor;
	double tmp_left_sensor  = left_sensor;
	double tmp_right_sensor = right_sensor;
	sem_post(&dist_sensorsSemaphore);

	int this_pixel_x 	   = (int)round(4*(tmp_position_x));
	int this_pixel_y 	   = (int)round(4*(tmp_position_y));

	int left_sensor_pix_x  = (int)round(4*(tmp_position_x - 0.25 * (sin(ST_TO_RAD * tmp_orientation))));  
	int left_sensor_pix_y  = (int)round(4*(tmp_position_y + 0.25 * (cos(ST_TO_RAD * tmp_orientation))));
	int right_sensor_pix_x = (int)round(4*(tmp_position_x + 0.25 * (sin(ST_TO_RAD * tmp_orientation))));
	int right_sensor_pix_y = (int)round(4*(tmp_position_y - 0.25 * (cos(ST_TO_RAD * tmp_orientation))));
	int front_sensor_pix_x = (int)round(4*(tmp_position_x + 0.25 * (cos(ST_TO_RAD * tmp_orientation))));
	int front_sensor_pix_y = (int)round(4*(tmp_position_y + 0.25 * (sin(ST_TO_RAD * tmp_orientation))));
	
	
	// printf("raw left: %f %f\n",tmp_position_x - 0.25 * (sin(ST_TO_RAD * tmp_orientation)),tmp_position_y + 0.25 * (cos(ST_TO_RAD * tmp_orientation)));
	// printf("this: %d %d\n",this_pixel_x,this_pixel_y);
	// printf("f:%d %d    l:%d %d     r:%d %d\n",front_sensor_pix_x,front_sensor_pix_y,left_sensor_pix_x,left_sensor_pix_y,right_sensor_pix_x,right_sensor_pix_y);

		if (rg_disc_map[this_pixel_x][this_pixel_y] != 2) {
			rg_disc_map[this_pixel_x][this_pixel_y] = 2; }
		
		if (rg_disc_map[left_sensor_pix_x][left_sensor_pix_y] < 2){
			if (tmp_left_sensor < 0.025) {
				rg_disc_map[left_sensor_pix_x][left_sensor_pix_y] = 3;
			}
			else {
				ba_disc_map[left_sensor_pix_x][left_sensor_pix_y] = 1;
			}
		}
		if (rg_disc_map[right_sensor_pix_x][right_sensor_pix_y] < 2){
			if (tmp_right_sensor < 0.025) {
				rg_disc_map[right_sensor_pix_x][right_sensor_pix_y] = 3;
			}
			else {

				rg_disc_map[right_sensor_pix_x][right_sensor_pix_y] = 1;
			}
		}
		if (rg_disc_map[front_sensor_pix_x][front_sensor_pix_y] < 2){
			if (tmp_front_sensor < 0.025) {
				rg_disc_map[front_sensor_pix_x][front_sensor_pix_y] = 3;
			}
			else {
				rg_disc_map[front_sensor_pix_x][front_sensor_pix_y] = 1;
			}
		}


	/* write a map for debugging */
    // FILE *fptr;
    // char c;
    // fptr = fopen("../../roomba/log/map_rg.txt","w");
    // for(int i = 0; i < 80; i++) {
    //     for(int j = 0; j < 80; j++){
    //         c = rg_disc_map[j][80-i] +'0';
    //         fputc(c,fptr);
    //     }
    //     fprintf(fptr,"\r\n");
    // }
    // fclose(fptr);

	return EXIT_SUCCESS;
}

/* function supervising movement within current task */
int calculate_movement_type(void) {

	// read input values
	sem_wait(&position_orientationSemaphore);
	double tmp_orientation = orientation;
	double tmp_pos_x = position_x;
	double tmp_pos_y = position_y;
	sem_post(&position_orientationSemaphore);
	sem_wait(&dist_sensorsSemaphore);
	double tmp_front_sensor = front_sensor;
	double tmp_back_sensor = back_sensor;
	double tmp_left_sensor = left_sensor;
	double tmp_right_sensor = right_sensor;
	sem_post(&dist_sensorsSemaphore);
	sem_wait(&taskSemaphore);
	double tmp_current_task = current_task;
	sem_post(&taskSemaphore);
	sem_wait(&target_directionSemaphore);
	double target_direction_tmp = target_direction;
	sem_post(&target_directionSemaphore);

	if (algorithm_select == 0){
		if (spool_next_step_rg_calculated == 1){
			if (sem_trywait(&superv_calc_rg_Semaphore) != 0){
				return EXIT_SUCCESS;
			}
			// algorith_select = 0 is random bouncing
			switch (current_task)
			{
				case 0:
					// just standing
					movement_type = 0;
					break;
				case 1:
					// driving forward
					if (tmp_front_sensor < 0.0025 || tmp_left_sensor < 0.005 || tmp_right_sensor < 0.005){
						// if close to wall, notify about need to select new direction
						spool_next_step_rg_calculated = 0;
						sem_post(&spool_calc_next_step_rg);
					}
					else {
						movement_type = 1;
					}
					break;
				case 2:
					// rotating clockwise until reaching target_direction, then driving forward
					if (fabs(tmp_orientation - target_direction_tmp) < 0.5) { 
						current_task = 1;
						movement_type = 1; 
					} else {
						movement_type = 2;
					}
					break;
				case 3: 
					// rotating counter clokcwise until reaching target direction, then driving forward
					if (fabs(tmp_orientation - target_direction_tmp) < 0.5) {
						current_task = 1;
						movement_type = 1;
					} else {
						movement_type = 3;
					}
					break;
				default: 
					break;
			}
			sem_post(&superv_calc_rg_Semaphore);
		}
	}
	
	else if (algorithm_select == 1){
		// algorithm_select = 1 is STC algorithm

		// check flag "is task calculated yet"
		if (spool_next_step_calculated == 1) {
			if (sem_trywait(&superv_calc_stc_Semaphore) != 0) {
				return EXIT_SUCCESS;
			}
			switch(next_step)
			{
				case 0:
					// just standing
					movement_type = 0;
					break;
				case 1:
					// rotate counter clockwise until reaching target direction
					if (fabs(tmp_orientation_stc_step - fmod(tmp_orientation,360.0)) > 0.1) {
						movement_type = 3;
					} else {
						// target direction reached, set movement to 0
						movement_type = 0;
						// update current orientation
						current_orientation = (current_orientation + 1) % 4;
						// reset "next_step calculated" flag
						spool_next_step_calculated = 0;
						// notify STC thread
						sem_post(&spool_calc_next_step_stc);
					}
					break;
				case 2:
					// rotate clockwise until reaching target direction
					if (fabs(tmp_orientation_stc_step - fmod(tmp_orientation,360.0)) > 0.1) {
						movement_type = 2;
					} else {
						// target direction reached, set movement to 0
						movement_type = 0;
						// update current orientation
						current_orientation = (current_orientation + 3) % 4;
						// reset "next_step calculated" flag
						spool_next_step_calculated = 0;
						// notify STC thread
						sem_post(&spool_calc_next_step_stc);
					}
					break;
				case 3:
					// move forward until moving by a quarter distance
					// printf("\n%f %f", tmp_pos_x, tmp_pos_y);
					// printf("\n%f %f", fabs(tmp_pos_x_stc_step - tmp_pos_x),fabs(tmp_pos_y_stc_step - tmp_pos_y));
					if (fabs(tmp_pos_x_stc_step - tmp_pos_x) >= 0.24999999 ||
						fabs(tmp_pos_y_stc_step - tmp_pos_y) >= 0.24999999) {
						// distance reached, set movement to 0
						movement_type = 0;
						// reset "next step calculated" flag
						spool_next_step_calculated = 0;
						// notify STC thread
						sem_post(&spool_calc_next_step_stc);
						//status = calc_next_step();
						}
					else {
						// keep moving while distance not reached
						movement_type = 1;
					}
					break;
			 	default:
					// default movement is 0 (no movement) for safety 
					movement_type = 0;
					break;
			}
		sem_post(&superv_calc_stc_Semaphore);			
		}
	}

	else if (algorithm_select == 2){
		if (spool_next_step_ba_calculated == 1){
			if(sem_trywait(&superv_calc_ba_Semaphore) != 0){
				return EXIT_SUCCESS;
			}
			switch(current_task_ba){
				case 0:
					// just standing
					movement_type = 0;
					break;
				case 1:
					// rotate clockwise to orientation
					if (fabs(target_orientation_ba - fmod(tmp_orientation,360.0)) > 0.05) {
						movement_type = 2;
						//printf("%f\n",fabs(target_orientation_ba - fmod(tmp_orientation,360.0)));
						}
					else {
						movement_type = 0;
						// set flag so there is no movement until BA thread calculates
						spool_next_step_ba_calculated = 0;
						// notify BA thread
						sem_post(&spool_calc_next_step_ba);
					}
					break;
				case 2: 
					// rotate clockwise to orientation
					if (fabs(target_orientation_ba - fmod(tmp_orientation,360.0)) > 0.05) {
						movement_type = 3;
						//printf("%f\n",fabs(target_orientation_ba - fmod(tmp_orientation,360.0)));
					}
					else {
						//printf("reached orientation\n");
						movement_type = 0;
						// set flag so there is no movement until BA thread calculates
						spool_next_step_ba_calculated = 0;
						// notify BA thread
						sem_post(&spool_calc_next_step_ba);
					}
					break;
				case 3:
					if (ba_disc_map[(int)round(4*(tmp_pos_x+0.25*(cos(ST_TO_RAD*tmp_orientation))))]
							   	   [(int)round(4*(tmp_pos_y+0.25*(sin(ST_TO_RAD*tmp_orientation))))] == 2 ||
					   (ba_disc_map[(int)round(4*(tmp_pos_x+0.25*(cos(ST_TO_RAD*tmp_orientation))))]
								   [(int)round(4*(tmp_pos_y+0.25*(sin(ST_TO_RAD*tmp_orientation))))] == 3 &&
							   	   front_sensor <= 0.005)){//} ||
								   //left_sensor <= 0.0005    ||
								   //right_sensor <= 0.0005){
						movement_type = 0;
						spool_next_step_ba_calculated = 0;
						sem_post(&spool_calc_next_step_ba);	
					}
					else {
						movement_type = 1;
					}
					break;
				case 4:
					// printf("%f\n",sqrt(pow(target_position_x_ba - tmp_pos_x,2) + 
					// 			       pow(target_position_y_ba - tmp_pos_y,2)));
					dist = sqrt(pow(target_position_x_ba - tmp_pos_x,2) + 
								pow(target_position_y_ba - tmp_pos_y,2));
					if (movement_mode == 0){
						if (dist >= 0.005 && prev_dist >= dist) {
							prev_dist = dist;
							movement_type = 1;
						}
						else {
							prev_dist = 10000;
							movement_type = 0;
							spool_next_step_ba_calculated = 0;
							sem_post(&spool_calc_next_step_ba);
						}
					} else {
						if (dist >= 0.005 && prev_dist >= dist && front_sensor > 0.0025) {
							prev_dist = dist;
							movement_type = 1;
						} else {
							prev_dist = 10000;
							movement_type = 0;
							spool_next_step_ba_calculated = 0;
							sem_post(&spool_calc_next_step_ba);
						}
					}
					break;

				default:
				movement_type = 0;
				break;
					
			}
		sem_post(&superv_calc_ba_Semaphore); 			
		}
	}
	
	else if (algorithm_select == 3){
		if (spool_next_step_swf_calculated == 1){
			if (sem_trywait(&superv_calc_swf_Semaphore) != 0){
				return EXIT_SUCCESS;
			};
			movement_type = swf_mov_superv(tmp_pos_x,tmp_pos_y,tmp_orientation, tmp_front_sensor,
									   	   tmp_back_sensor,tmp_left_sensor,tmp_right_sensor);
			sem_post(&superv_calc_swf_Semaphore);
		}
	}
	
	return EXIT_SUCCESS;
}

/* function initializing STC algorithm variables and calculating first step */
int init_stc(void) {
    
	int status;
	spool_next_step_calculated = 0;
	current_position_x = 0;
	current_position_y = 0;
    current_orientation = 0;
	tmp_orientation_stc_step = 0;
	tmp_pos_x_stc_step = 0;
	tmp_pos_y_stc_step = 0;
    current_quarter = 0;
    target_cell = 1;
    next_step = 0;
	algorithm_finished = 0;

	/* clear log files for debugging */
	FILE *fptr2;
    fptr2 = fopen("../../roomba/log/decisions.txt","w");
    fclose(fptr2);
    FILE *fptr3;
    fptr3 = fopen("../../roomba/log/quarters.txt","w");
    fclose(fptr3);
	
	// sleep some time so simulator will set up
	sleep(3);
	
	sem_wait(&position_orientationSemaphore);
	starting_cell_x = (int)(round((position_x+0.125)*2));
	starting_cell_y = (int)(round((position_y-0.125)*2));
	sem_post(&position_orientationSemaphore);

	parent_cells[starting_cell_x][starting_cell_y] = 3;

	// check neighbourhood
    status = check_nbh();
	// select first target cell
    status = select_target_cell();
	// calculate next step
    status = calc_next_step();

    return EXIT_SUCCESS;
}

/* function initializing BA algorithm variables */
int init_ba(void) {
	current_task_ba = 0;
	movement_mode = 0;
	vertical_horizontal = 0;
	path_index = 0;
	dist = 10000;
	prev_dist = 10000;

	FILE *fptr12;
    fptr12 = fopen("../../roomba/log/decisions_ba.txt","w");
    fclose(fptr12);

	//int status = run_test_1();

	current_task_ba = 3;
	spool_next_step_ba_calculated = 1;
	return EXIT_SUCCESS;
}

/* function initializing sfw algorithm variables */
int init_swf(void) {
	current_swf_step = 1;
	spool_next_step_swf_calculated = 1;
	new_wall_parameter = 0;
	dist_swf = 10000;
	prev_dist_swf = 10000;
	is_at_corner = 0;
	virt_sensor_front = 1.0;
	virt_sensor_right = 1.0;
	virt_sensor_left = 1.0;
	new_loop = 0;
	is_updatable = 0;
	movement_mode_swf = 0;
	is_path_calculated = 0;
	init_plan();
	return EXIT_SUCCESS;
}

/* function checking distance sensors and updating disc_plan if obstacles detected */
int check_nbh(void) {

    double i = 0, j = 0;
    double f, b, r, l = 0;

	// write sensors data to tmp variables 
    sem_wait(&dist_sensorsSemaphore);
    double tmp_front_sensor = front_sensor;
    double tmp_back_sensor = back_sensor;
    double tmp_left_sensor = left_sensor;
    double tmp_right_sensor = right_sensor;
    sem_post(&dist_sensorsSemaphore);

	// write odometry data to tmp variables
	sem_wait(&position_orientationSemaphore);
	double tmp_position_x = position_x;
	double tmp_position_y = position_y;
	// double tmp_position_x = round(100*(position_x))/100;
	// double tmp_position_y = round(100*(position_y))/100;


	sem_post(&position_orientationSemaphore);

    switch (current_quarter){
		// i, j, f, b, r, l are for correct obstacles detection
		// needed space depends on current quarter and orientation
        case 0:  i = 0.125; j = -0.125;f = 0.025; l = 0.025; break;
        case 1:  i = 0.125; j = 0.125; b = 0.025; l = 0.025; break;
        case 2:  i = -0.125;j = 0.125; b = 0.025; r = 0.025; break;
        case 3:  i = -0.125;j = -0.125;f = 0.025; r = 0.025; break;
    }

	// transform cartesian position to appropriate int value
    current_position_x = (int)(round((tmp_position_x+i)*2));
    current_position_y = (int)(round((tmp_position_y+j)*2));
	// printf("\ncurrent quarter: %d\n",current_quarter);
	// printf("i: %f j: %f\n",i,j);
	// printf("raw_pos_x: %f raw_pos_y: %f\n", tmp_position_x,tmp_position_y);
	// printf("curr_x: %f  curr_y: %f\n",tmp_position_x+i,tmp_position_y+j);
	// printf("current_x: %d  current_y: %d\n", current_position_x, current_position_y);


	// absolute sensors values in terms of disc_map
    double f_s, b_s, r_s, l_s;

	// update parent cells structure
    if (parent_cells[current_position_x][current_position_y] == 0){
        parent_cells[current_position_x][current_position_y] = target_cell + 1;
    }

	// transformation of relative (depending on robot orientation) sensors values
	// to absolute in terms of disc_map
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
 

	// update disc_plan based on sensors

	// if lower cell is not set or not visited
    if (disc_plan2[(int)(round((tmp_position_x+i)*2))][(int)(round((tmp_position_y+j)*2)) - 1] <= 1){
		// if corresponding sensor detects obstacle on lower cell, set that cell status to 3 - obstacle
        if (f_s <= (0.05+f)){disc_plan2[(int)(round((tmp_position_x+i)*2))][(int)(round((tmp_position_y+j)*2)) - 1] = 3;}
		// if there is no obstacle, set that cell status to 1 - not visited
        else {disc_plan2[(int)(round((tmp_position_x+i)*2))][(int)(round((tmp_position_y+j)*2)) - 1] = 1;}
    }
	// if right cell is not set or not visited
    if (disc_plan2[(int)(round((tmp_position_x+i)*2)+1)][(int)(round((tmp_position_y+j)*2))] <= 1){
		// if corresponding sensor detects obstacle on right cell, set that cell status to 3 - obstacle
        if (l_s <= (0.05+l)){disc_plan2[(int)(round((tmp_position_x+i)*2)+1)][(int)(round((tmp_position_y+j)*2))] = 3;}
		// if there is no obstacle, set that cell status to 1 - not visited		
        else {disc_plan2[(int)(round((tmp_position_x+i)*2)+1)][(int)(round((tmp_position_y+j)*2))] = 1;}
    }
	// if upper cell is not set or not visited
    if (disc_plan2[(int)(round((tmp_position_x+i)*2))][(int)(round((tmp_position_y+j)*2)) + 1] <= 1){
		// if corresponding sensor detects obstacle on upper cell, set that cell status to 3 - obstacle		
        if (b_s <= (0.05+b)){disc_plan2[(int)(round((tmp_position_x+i)*2))][(int)(round((tmp_position_y+j)*2)) + 1] = 3;}
		// if there is no obstacle, set that cell status to 1 - not visited		
        else {disc_plan2[(int)(round((tmp_position_x+i)*2))][(int)(round((tmp_position_y+j)*2)) + 1] = 1;}
    }
	// if left cell is not set or not visited
    if (disc_plan2[(int)(round((tmp_position_x+i)*2)-1)][(int)(round((tmp_position_y+j)*2))] <= 1){
		// if corresponding sensor detects obstacle on left cell, set that cell status to 3 - obstacle			
        if (r_s <= (0.05+r)){disc_plan2[(int)(round((tmp_position_x+i)*2)-1)][(int)(round((tmp_position_y+j)*2))] = 3;}
		// if there is no obstacle, set that cell status to 1 - not visited		
        else {disc_plan2[(int)(round((tmp_position_x+i)*2)-1)][(int)(round((tmp_position_y+j)*2))] = 1;}
    }

	// set current cell status to 2 - visited
    disc_plan2[(int)(round((tmp_position_x+i)*2))][(int)(round((tmp_position_y+j)*2))] = 2;
    
	// // update parent cells structure
	// if (parent_cells[current_position_x][current_position_y] == 0){
    //     parent_cells[current_position_x][current_position_y] = target_cell + 1;
    // }

	/* write map for debugging */
    // FILE *fptr1;
    // char cc;
    // fptr1 = fopen("../../roomba/log/map.txt","w");
    // for(int i = 0; i < 40; i++) {
    //     for(int j = 0; j < 40; j++){
    //         cc = disc_plan2[j][40-i] +'0';
    //         fputc(cc,fptr1);
    //     }
    //     fprintf(fptr1,"\r\n");
    // }
    // fclose(fptr1);

	// /* write parent map for debugging */
    // FILE *fptr;
    // char c;
    // fptr = fopen("../../roomba/log/map_parent.txt","w");
    // for(int i = 0; i < 40; i++) {
    //     for(int j = 0; j < 40; j++){
    //         c = parent_cells[j][40-i] +'0';
    //         fputc(c,fptr);
    //     }
    //     fprintf(fptr,"\r\n");
    // }
    // fclose(fptr);

    return EXIT_SUCCESS;
}

/* function selecting target cell based on disc_plan */
int select_target_cell(void){
// this function is only called after reaching first quarter of a cell from another cell

	// if orientation is down aka quarter 0:
    if (current_orientation == 0){
		// check counter clockwise for nearest not visited cell
        if (disc_plan2[current_position_x-1][current_position_y] == 1) {target_cell = 1;}
        else if (disc_plan2[current_position_x][current_position_y-1] == 1) {target_cell = 2;}
        else if (disc_plan2[current_position_x+1][current_position_y] == 1) {target_cell = 3;}
		// if there is no not visited cell, then set parent cell as a target
        else {target_cell = parent_to_target(parent_cells[current_position_x][current_position_y]);}
    }
	// if orientation is right aka quarter 1:
    else if (current_orientation == 1){
        if (disc_plan2[current_position_x][current_position_y-1] == 1) {target_cell = 2;}
        else if (disc_plan2[current_position_x+1][current_position_y] == 1) {target_cell = 3;}
        else if (disc_plan2[current_position_x][current_position_y+1] == 1) {target_cell = 0;}
        else {target_cell = parent_to_target(parent_cells[current_position_x][current_position_y]);}
    }
	// if orientation is up aka quarter 2:
    else if (current_orientation == 2){
        if (disc_plan2[current_position_x+1][current_position_y] == 1) {target_cell = 3;}
        else if (disc_plan2[current_position_x][current_position_y+1] == 1) {target_cell = 0;}
        else if (disc_plan2[current_position_x-1][current_position_y] == 1) {target_cell = 1;}
        else {target_cell = parent_to_target(parent_cells[current_position_x][current_position_y]);}
    }
	// if orientation is left aka quarter 3:
    else if (current_orientation == 3){
        if (disc_plan2[current_position_x][current_position_y+1] == 1) {target_cell = 0;}
        else if (disc_plan2[current_position_x-1][current_position_y] == 1) {target_cell = 1;}
        else if (disc_plan2[current_position_x][current_position_y-1] == 1) {target_cell = 2;}
        else {target_cell = parent_to_target(parent_cells[current_position_x][current_position_y]);}
    }

	/* write decisions and circumstances to file for debugging */
    // FILE *fptr2;
    // char ccc;
    // fptr2 = fopen("../../roomba/log/decisions.txt","a");
    // fprintf(fptr2,"pos_x: %f pos_y: %f orientation: %f\r\n", position_x, position_y, orientation);
    // fprintf(fptr2,"up: %d left: %d down: %d right: %d, parent %d, target: %d\r\n",
    //             disc_plan2[current_position_x][current_position_y+1],
    //             disc_plan2[current_position_x-1][current_position_y],
    //             disc_plan2[current_position_x][current_position_y-1],
    //             disc_plan2[current_position_x+1][current_position_y],
    //             parent_cells[current_position_x][current_position_y], target_cell);
    // fprintf(fptr2,"front: %.5f left: %.5f back: %.5f right: %.5f\r\n",front_sensor,left_sensor,back_sensor,right_sensor); 
    // fprintf(fptr2,"\r\n");
    // fclose(fptr2);

    return EXIT_SUCCESS;
}

/* function calculating next step */
int calc_next_step(void){

    int status;
	
	// update quarter and cell (check nbh and select target cell in there if needed)
    status = update_quarter_and_cell();
	sem_wait(&position_orientationSemaphore);
    tmp_orientation_stc_step = orientation;
    tmp_pos_x_stc_step = (round(position_x*8))/8;
    tmp_pos_y_stc_step = (round(position_y*8))/8;
	sem_post(&position_orientationSemaphore);

	// depending on quarter, orientation and target cell, set next step
	// to rotating or moving forward
	// next step is compliant to stc algorithm principles

	/* end algorithm condition */
	if (current_position_x == starting_cell_x &&
	    current_position_y == starting_cell_y &&
		current_quarter    == 3){
			algorithm_finished = 1;
			return EXIT_SUCCESS;
		}

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
	// set target orientation for rotating counter clockwise
    if (next_step == 1){
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
    }
	// set target orientation for rotating clockwise
    else if (next_step == 2) {
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
    }
    
    // printf("current quarter: %d\n", current_quarter);
    // printf("target cell: %d\n", target_cell);
    // printf("current orientation: %d\n",current_orientation);
    // printf("next step: %d   \n", next_step);
    // printf("algorithm:%d\n",algorithm_select);
    // printf("%f\n\n",tmp_orientation_stc_step);

	// set flag "calculated next step"
	spool_next_step_calculated = 1;
    return EXIT_SUCCESS;
}

/* function updating quarter and cell if needed after movement */
int update_quarter_and_cell(void) {
    
    int status;

	// if the movement was rotating - do nothing
	// if the movement was driving - update quarter and cell
    if (next_step == 3) {
		// set parent cell of this cell so the algorithm can go back later
		if (parent_cells[current_position_x][current_position_y] == 0){
        	parent_cells[current_position_x][current_position_y] = target_cell + 1;
    	}
        switch (current_quarter)
        {
            case 0:
				// moving within cell - update quarter
                if (current_orientation == 0){
                    current_quarter += 1;
                }
				// moving between cells - update quarter, cell and select new target cell
                else if (current_orientation == 3) {
                    current_quarter = 3;
                    current_position_x -= 1;
                    status = check_nbh();
                    status = select_target_cell(); 
                }
                break;
            case 1:
				// moving within cell - update quarter
                if (current_orientation == 1){
                    current_quarter += 1;
                }
				// moving between cells - update quarter, cell and select new target cell
                else if (current_orientation == 0) {
                    current_quarter = 0;
                    current_position_y -= 1;
                    status = check_nbh();
                    status = select_target_cell();
                }
                break;
            case 2:
				// moving within cell - update quarter
                if (current_orientation == 2) {
                    current_quarter += 1;
                }
				// moving between cells - update quarter, cell and select new target cell				
                else if (current_orientation == 1) {
                    current_quarter = 1;
                    current_position_x += 1;
                    status = check_nbh();
                    status = select_target_cell();
                }
                break;
            case 3:
				// moving within cell - update quarter
                if (current_orientation == 3) {
                    current_quarter = 0;
                }
				// moving between cells - update quarter, cell and select new target cell				
                else if (current_orientation == 2) {
                    current_quarter = 2;
                    current_position_y += 1;
                    status = check_nbh();
                    status = select_target_cell();
                }
                break;
        }
    }
	 /*for debugging */
    // FILE *fptr3;
    // char q;
    // fptr3 = fopen("../../roomba/log/quarters.txt","a");
    // fprintf(fptr3,"pos_x: %.2f pos_y: %.2f orientation: %.2f next step: %d quarter: %d\r\n", position_x, position_y, orientation, next_step, current_quarter);
    // fclose(fptr3);

    return EXIT_SUCCESS;
}

/* function determining target cell based on given parent_cell value */
int parent_to_target(int parent_cell) {
    if (parent_cell == 1) { return 2;}
    else if (parent_cell == 2) {return 3;}
    else if (parent_cell == 3) {return 0;}
    else if (parent_cell == 4) {return 1;}
    else { return 0;}
}

/* function calculating what to do in BA* algorithm */
int calc_next_task(void){
	
	// get most recent state of map
	update_ba_map();

	sem_wait(&position_orientationSemaphore);
	double tmp_position_x = position_x;
	double tmp_position_y = position_y;
	double tmp_orientation = orientation;
	sem_post(&position_orientationSemaphore);
	
	// get current task and vertical/horizontal orientation
	int tmp_current_task_ba = current_task_ba;
	vertical_horizontal = ((int)round(tmp_orientation/90)+1) % 2;
	//printf("%d", tmp_current_task_ba);
	switch (tmp_current_task_ba){
		case 0:
			// just standing
			break;
		case 1 ... 2:
			// finished rotating
			if (movement_mode == 1){ 
				// if navigating to BT point, set target point from path and drive
				target_position_x_ba = bt_target_x;
				target_position_y_ba = bt_target_y;
				current_task_ba = 4;
				spool_next_step_ba_calculated = 1;
				break;
			}
			else {
				// if simple B. movement
				if (vertical_horizontal == 1){
					// if horizontal then move by left/right by 25cm (cleaning size)
					target_position_x_ba = tmp_position_x + 0.25*cos((round(tmp_orientation) * ST_TO_RAD));
					target_position_y_ba = tmp_position_y + 0.25*sin((round(tmp_orientation) * ST_TO_RAD));
					current_task_ba = 4;
					spool_next_step_ba_calculated = 1;
				}
				else {
					// if its B. movement, position vertical and finished rotating, then move forward 
					current_task_ba = 3;
					spool_next_step_ba_calculated = 1;
				}
				break;
			}
		case 3 ... 4:
			// if finished moving forward (reaching wall, point or visited point)
			if (movement_mode == 0) {
				// B. movement
				// check nbh and choose apropriate new direction
				if (ba_disc_map[(int)round(4*(tmp_position_x))][(int)round(4*(tmp_position_y))+1] == 1) {
					target_orientation_ba = 90.0;
					//printf("90\n");
					current_task_ba = rotation_direction(target_orientation_ba,tmp_orientation);
					spool_next_step_ba_calculated = 1;		
					break;		
				}
				else if (ba_disc_map[(int)round(4*(tmp_position_x))][(int)round(4*(tmp_position_y))-1] == 1) {
					target_orientation_ba = 270.0;
					//printf("270\n");
					current_task_ba = rotation_direction(target_orientation_ba,tmp_orientation);
					spool_next_step_ba_calculated = 1;
					break;
				}
				else if (ba_disc_map[(int)round(4*(tmp_position_x))+1][(int)round(4*(tmp_position_y))] == 1) {
					target_orientation_ba = 0.0;
					//printf("0\n");
					current_task_ba = rotation_direction(target_orientation_ba,tmp_orientation);
					spool_next_step_ba_calculated = 1;
					break;
				}
				else if (ba_disc_map[(int)round(4*(tmp_position_x))-1][(int)round(4*(tmp_position_y))] == 1) {
					target_orientation_ba = 180.0;
					//printf("180\n");
					current_task_ba = rotation_direction(target_orientation_ba,tmp_orientation);
					spool_next_step_ba_calculated = 1;
					break;
				}
				else {
					// no valid point, then find path to new bt point

					// FILE *fptr14;
					// char c;
					// fptr14 = fopen("../../roomba/log/decisions_ba.txt","a");
					// fprintf(fptr14,"pos_x: %f pos_y: %f orientation: %f\r\n", position_x, position_y, orientation);
					// fprintf(fptr14,"up: %d down: %d left: %d right: %d, target: %f\r\n moving to next area\r\n\r\n",
					// 			ba_disc_map[(int)round(4*(tmp_position_x))][(int)round(4*(tmp_position_y))+1],
					// 			ba_disc_map[(int)round(4*(tmp_position_x))][(int)round(4*(tmp_position_y))-1],
					// 			ba_disc_map[(int)round(4*(tmp_position_x))-1][(int)round(4*(tmp_position_y))],
					// 			ba_disc_map[(int)round(4*(tmp_position_x))+1][(int)round(4*(tmp_position_y))],
					// 			target_orientation_ba);
					// fprintf(fptr14,"\r\n");
					// fclose(fptr14);

					// set movement mode to navigating to next area
					movement_mode = 1;
					// get most recent data
					update_ba_map();
					// create list of bt points
					int status = create_bt_list();
					int bt_point;
					// select best bt point
					bt_point = select_bt_point();

					// if there were no bt points, algorithm has finished
					if (bt_point < 0) {
						algorithm_finished = 1;
						return EXIT_SUCCESS;
					}

					// starting point for A*
					int begin_x = (int)round(4*(tmp_position_x));
					int begin_y = (int)round(4*(tmp_position_y));

					// prepare structures for A* pathfinder
					grid_uint8_t grid;
					init_a_grid(&grid);
					coordinate_t begin = {begin_x,begin_y};
					coordinate_t end = {bt_list[bt_point][0],bt_list[bt_point][1]};
					//print_begin_end(&begin, &end);

					// find path 
					path_t path_raw = astar_uint8_get_path(&grid,end,begin, is_point_traversable);

					// index for path following
					path_index = 0;
					if(path_empty(&path_raw)) {
						algorithm_finished = 1;
						printf("No path found!\nFinishing work...\n");
					}
					// else {
					// 	printf("Path before smoothing:\n");
					// 	path_for_each_r(&path_raw, coordinate_print);
					// 	printf("\n");
					// }
					//printf("Path after smoothing:\n");
					//path_t path2 = path;

					// smoth the path using smoothing funciton
					path = smooth_path(&path_raw);

					path_destroy(&path_raw);
					//path_for_each_r(&path, coordinate_print);
					//printf("\n");
					//printf("size: %zu capacity: %zu\n",path.size,path.capacity);
					//fflush(stdout);
					grid_uint8_destroy(&grid);

					// get first point' coordinates and orientation to rotate to
					double dest_x = ((double)(path_get(&path,path_index)).row)/4;
					double dest_y = ((double)(path_get(&path,path_index)).col)/4;
					target_orientation_ba = calculate_target_angle(tmp_position_x,tmp_position_y,dest_x,dest_y);

					bt_target_x = dest_x;
					bt_target_y = dest_y;

					current_task_ba = rotation_direction(target_orientation_ba,tmp_orientation);
					spool_next_step_ba_calculated = 1;
					break;
				}


			// FILE *fptr13;
			// char cccc;
			// fptr13 = fopen("../../roomba/log/decisions_ba.txt","a");
			// fprintf(fptr13,"pos_x: %f pos_y: %f orientation: %f\r\n", position_x, position_y, orientation);
			// fprintf(fptr13,"up: %d down: %d left: %d right: %d, target: %f\r\n",
			// 			ba_disc_map[(int)round(4*(tmp_position_x))][(int)round(4*(tmp_position_y))+1],
			// 			ba_disc_map[(int)round(4*(tmp_position_x))][(int)round(4*(tmp_position_y))-1],
			// 			ba_disc_map[(int)round(4*(tmp_position_x))-1][(int)round(4*(tmp_position_y))],
			// 			ba_disc_map[(int)round(4*(tmp_position_x))-1][(int)round(4*(tmp_position_y))],
			// 			target_orientation_ba);
			// fprintf(fptr13,"front: %.5f left: %.5f back: %.5f right: %.5f\r\n",front_sensor,left_sensor,back_sensor,right_sensor); 
			// fprintf(fptr13,"\r\n");
			// fclose(fptr13);
		
		}
		else {
			// if navigating to next area
			// increment point index
			path_index++;
			if (path_index == path_size(&path)){
				// if that was last point of path, select new apropriate direction and 
				// start B. movement
				if (ba_disc_map[(int)round(4*(tmp_position_x))][(int)round(4*(tmp_position_y))+1] == 1) {
					target_orientation_ba = 90.0;
					//printf("90\n");
				}
				else if (ba_disc_map[(int)round(4*(tmp_position_x))][(int)round(4*(tmp_position_y))-1] == 1) {
					target_orientation_ba = 270.0;
					//printf("270\n");
				}
				else if (ba_disc_map[(int)round(4*(tmp_position_x))+1][(int)round(4*(tmp_position_y))] == 1) {
					target_orientation_ba = 0.0;
					//printf("0\n");
				}
				else if (ba_disc_map[(int)round(4*(tmp_position_x))-1][(int)round(4*(tmp_position_y))] == 1) {
					target_orientation_ba = 180.0;
					//printf("180\n");
				}
				//printf("begin B. movement\n");
				movement_mode = 0;
				current_task_ba = rotation_direction(target_orientation_ba,tmp_orientation);
				spool_next_step_ba_calculated = 1;
				path_destroy(&path);
				break; 
			}
			else {
				// if points was not the last one, get coords of next one and orientation
				bt_target_x = ((double)(path_get(&path,path_index).row))/4;
				bt_target_y = ((double)(path_get(&path,path_index).col))/4;
				//printf("target_x_px: %d target_y_px: %d\n", (int)path_get(&path,path_index).row,(int)path_get(&path,path_index).col);
				target_orientation_ba = calculate_target_angle(tmp_position_x,tmp_position_y,bt_target_x,bt_target_y);
				current_task_ba = rotation_direction(target_orientation_ba,tmp_orientation);
				spool_next_step_ba_calculated = 1;
				break;
			}
		}
		}
	return EXIT_SUCCESS;
}

/* function updating BA* map */
int update_ba_map(void) {

	int status;

	sem_wait(&position_orientationSemaphore);
	double tmp_orientation = orientation;
	double tmp_position_x = position_x;
	double tmp_position_y = position_y;
	sem_post(&position_orientationSemaphore);

	sem_wait(&dist_sensorsSemaphore);
	double tmp_front_sensor = front_sensor;
	double tmp_back_sensor  = back_sensor;
	double tmp_left_sensor  = left_sensor;
	double tmp_right_sensor = right_sensor;
	sem_post(&dist_sensorsSemaphore);

	// convert coordinates to grid indexes
	int this_pixel_x 	   = (int)round(4*(tmp_position_x));
	int this_pixel_y 	   = (int)round(4*(tmp_position_y));

	int left_sensor_pix_x  = (int)round(4*(tmp_position_x - 0.25 * (sin(ST_TO_RAD * tmp_orientation))));  
	int left_sensor_pix_y  = (int)round(4*(tmp_position_y + 0.25 * (cos(ST_TO_RAD * tmp_orientation))));
	int right_sensor_pix_x = (int)round(4*(tmp_position_x + 0.25 * (sin(ST_TO_RAD * tmp_orientation))));
	int right_sensor_pix_y = (int)round(4*(tmp_position_y - 0.25 * (cos(ST_TO_RAD * tmp_orientation))));
	int front_sensor_pix_x = (int)round(4*(tmp_position_x + 0.25 * (cos(ST_TO_RAD * tmp_orientation))));
	int front_sensor_pix_y = (int)round(4*(tmp_position_y + 0.25 * (sin(ST_TO_RAD * tmp_orientation))));
	
	
	// printf("raw left: %f %f\n",tmp_position_x - 0.25 * (sin(ST_TO_RAD * tmp_orientation)),tmp_position_y + 0.25 * (cos(ST_TO_RAD * tmp_orientation)));
	// printf("this: %d %d\n",this_pixel_x,this_pixel_y);
	// printf("f:%d %d    l:%d %d     r:%d %d\n",front_sensor_pix_x,front_sensor_pix_y,left_sensor_pix_x,left_sensor_pix_y,right_sensor_pix_x,right_sensor_pix_y);

	// update only on modulo 90deg to avoid bugs
	if (((int) round(tmp_orientation) % 90) == 0) {

		// set visited 
		if (ba_disc_map[this_pixel_x][this_pixel_y] != 2) {
			ba_disc_map[this_pixel_x][this_pixel_y] = 2; }
		
		// if obstacle set obstacle, if not set visitable
		if (ba_disc_map[left_sensor_pix_x][left_sensor_pix_y] < 2){
			if (tmp_left_sensor < 0.025) {
				ba_disc_map[left_sensor_pix_x][left_sensor_pix_y] = 3;
			}
			else {
				ba_disc_map[left_sensor_pix_x][left_sensor_pix_y] = 1;
			}
		}
		if (ba_disc_map[right_sensor_pix_x][right_sensor_pix_y] < 2){
			if (tmp_right_sensor < 0.025) {
				ba_disc_map[right_sensor_pix_x][right_sensor_pix_y] = 3;
			}
			else {

				ba_disc_map[right_sensor_pix_x][right_sensor_pix_y] = 1;
			}
		}
		if (ba_disc_map[front_sensor_pix_x][front_sensor_pix_y] < 2){
			if (tmp_front_sensor < 0.025) {
				ba_disc_map[front_sensor_pix_x][front_sensor_pix_y] = 3;
			}
			else {
				ba_disc_map[front_sensor_pix_x][front_sensor_pix_y] = 1;
			}
		}
	}


	/* write a map for debugging */
    // FILE *fptr;
    // char c;
    // fptr = fopen("../../roomba/log/map_ba.txt","w");
    // for(int i = 0; i < 80; i++) {
    //     for(int j = 0; j < 80; j++){
    //         c = ba_disc_map[j][80-i] +'0';
    //         fputc(c,fptr);
    //     }
    //     fprintf(fptr,"\r\n");
    // }
    // fclose(fptr);

	return EXIT_SUCCESS;
}

/* function creating bt list for ba* */
int create_bt_list(void) {

	memset(bt_list,0,sizeof(bt_list));
	bt_list[0][0] = 0;
	bt_list[0][1] = 0;
	int k = 0;

	for (int i = 1; i < 79; i++){
		for (int j = 1; j < 79; j++){
			if (ba_disc_map[i][j] == 2) {

				int s1 = ba_disc_map[i+1][j];
				int s2 = ba_disc_map[i+1][j+1];
				int s3 = ba_disc_map[i][j+1];
				int s4 = ba_disc_map[i-1][j+1];
				int s5 = ba_disc_map[i-1][j];
				int s6 = ba_disc_map[i-1][j-1];
				int s7 = ba_disc_map[i][j-1];
				int s8 = ba_disc_map[i+1][j-1];

				int goal = 0;
				goal += (s1 == 1 && (s2 % 3 == 0)) ? 1 : 0;
				goal += (s1 == 1 && (s8 % 3 == 0)) ? 1 : 0;
				goal += (s5 == 1 && (s6 % 3 == 0)) ? 1 : 0;
				goal += (s5 == 1 && (s4 % 3 == 0)) ? 1 : 0;
				goal += (s7 == 1 && (s6 % 3 == 0)) ? 1 : 0;
				goal += (s7 == 1 && (s8 % 3 == 0)) ? 1 : 0;

				if (goal >= 1) {
					bt_list[k][0] = i;
					bt_list[k][1] = j;
					k++;
				}
			}

		}
	}
	// for (int z = 0; bt_list[z][0] != 0; z++){
	// 	printf("\n{%d,%d}\n",bt_list[z][0],bt_list[z][1]);
	// }

	return EXIT_SUCCESS;
}

/* function selecting the best bt point for ba* */
int select_bt_point(void) {
	// just distance for now

	sem_wait(&position_orientationSemaphore);
	double tmp_position_x = position_x;
	double tmp_position_y = position_y;
	double tmp_orientation = orientation;
	sem_post(&position_orientationSemaphore);

	int best_point = 0;
	double best_value = 10000;
	int bt_point_x;
	int bt_point_y;
	double i_value;

	if (bt_list[0][0] == 0 && bt_list[0][1] == 0) { return -1; }

	for (int i = 0; i < 100; i++) {
		if (bt_list[i][0] == 0 && bt_list[i][1] == 0) {break;}
		bt_point_x = bt_list[i][0];
		bt_point_y = bt_list[i][1];	
		i_value = sqrt(pow(((double)bt_point_x)/4 - tmp_position_x,2) +
				 	   pow(((double)bt_point_y)/4 - tmp_position_y,2));
		if (i_value < best_value) {
			best_value = i_value;
			best_point = i;
		}
	}
	//printf("selected point: x = %d, y = %d\n",bt_list[best_point][0],bt_list[best_point][1]);

	return best_point;
}

/* input for path finding A* algorithm preparation (BA*) */
void init_a_grid(grid_uint8_t* grid){

	uint8_t values[80*80];
	for (int i = 0; i < 80; i++){
		for (int j = 0; j < 80; j++){
			values[80*i+j] = ba_disc_map[i][j];
		}
	}
	grid_uint8_init(grid,80,80,&values[0]);
	//grid_uint8_for_each(grid,grid_uint8_print);
}

/* input for path finding A* algorithm preparation (SWF) */
void init_a_grid_swf(grid_uint8_t* grid){

	uint8_t values[80*80];
	for (int i = 0; i < 80; i++){
		for (int j = 0; j < 80; j++){
			values[80*i+j] = swf_bt_map[i][j];
		}
	}
	grid_uint8_init(grid,80,80,&values[0]);
	//grid_uint8_for_each(grid,grid_uint8_print);
}

/* is point traversable? for pathfinding */
int is_point_traversable(const uint8_t* val) {
    return *val == 2 ? 1 : 0;
}

/* function calculating angle between two points in coordinates system */
double calculate_target_angle(double start_x, double start_y, double end_x, double end_y) {
	
	double diff_x = end_x - start_x;
	double diff_y = end_y - start_y;
	double angle = fmod((atan2(diff_y, diff_x) * 180.0 / M_PI) + 360.0,360.0);

	return round(100*angle)/100;
}

/* calculate cw or ccw direction of rotation so that robot will rotate <=180deg */
int rotation_direction(double target_angle, double current_angle){

        if ((   (target_angle > current_angle) && (fabs(target_angle - current_angle) > 180.0)) ||
            (   (target_angle < current_angle) && (fabs(target_angle - current_angle) < 180.0))) {
        	return 1;
        } else {
			return 2;
        }
}

/* function smoothing path for BA* */
path_t smooth_path(path_t* path) {

	path_t smoothed_path;
	path_init(&smoothed_path, 10);

	int is_path_traversable = 1;
	int last_checked_point = 0;
	int sequence_first_point = 0;
	int checked_point = 0;

	if (path_size(path) == 1) { 
		path_push_back(&smoothed_path,path_get(path,0));
		return smoothed_path;
	}


	//printf("size of path: %zu", path_size(path));
	while(last_checked_point != path_size(path)-1){
		//printf("last checked: %d   path size: %zu\n",last_checked_point,path_size(path));
		//printf("sequence_first_point: %d", sequence_first_point);
		// for(int i = sequence_first_point + 1; i < path_size(path); i++) {
		for(int i = path_size(path) - 1; i > sequence_first_point; i--) {
			double first_x 		= ((double)(path_get(path,sequence_first_point)).row)/4;
			double first_y 		= ((double)(path_get(path,sequence_first_point)).col)/4;
			double checked_x 	= ((double)(path_get(path,i)).row)/4;
			double checked_y 	= ((double)(path_get(path,i)).col)/4;

			// get angle between these 2 points
			double angle 	= calculate_target_angle(first_x,first_y,checked_x,checked_y);
			double distance = sqrt((pow(first_x-checked_x,2)+pow(first_y-checked_y,2)));
			is_path_traversable = 1;

			for (double j = 0.0; j <= distance; j = j + 0.1){
				// determining the closest pixel
				int x_pix = (int)round(4 * (first_x + j * cos(angle * ST_TO_RAD)));
				int y_pix = (int)round(4 * (first_y + j * sin(angle * ST_TO_RAD)));

				int x_pix_left =	(int)round(4 * ((first_x + j * cos(angle * ST_TO_RAD) - 0.1)));
				int x_pix_right = 	(int)round(4 * ((first_x + j * cos(angle * ST_TO_RAD) + 0.1)));
				int y_pix_up = 		(int)round(4 * ((first_y + j * sin(angle * ST_TO_RAD) + 0.1)));
				int y_pix_down = 	(int)round(4 * ((first_y + j * sin(angle * ST_TO_RAD) - 0.1)));

				int pix_value 		= ba_disc_map[x_pix][y_pix];
				int pix_value_left  = ba_disc_map[x_pix_left][y_pix];
				int pix_value_right = ba_disc_map[x_pix_right][y_pix];
				int pix_value_up    = ba_disc_map[x_pix][y_pix_up];
				int pix_value_down  = ba_disc_map[x_pix][y_pix_down];


				if ((pix_value == 2 && pix_value_left == 2 && pix_value_right == 2 && 
									  pix_value_up == 2	  && pix_value_down == 2) ||
									  x_pix_left == path_begin(path)->col ||
									  x_pix_right == path_begin(path)->col ||
									  y_pix_up == path_begin(path)->row ||
									  y_pix_down == path_begin(path)->row){
					continue; }
				else {
					is_path_traversable = 0;
					//printf("due to px %d, %d\n", x_pix,y_pix);
					break;
				}
			}
			//printf("\n");
			//printf("from point %d to point %d  ", sequence_first_point, i);
			//printf("is traversable path: %d\n", is_path_traversable);
			
			last_checked_point = i;


			// end to start
			if (is_path_traversable == 0) {
				if (i == sequence_first_point+1){
					path_push_back(&smoothed_path,path_get(path,i));
					sequence_first_point = i;
				}
				else {
					continue;
				}
			}
			else {
				path_push_back(&smoothed_path,path_get(path, i));
				sequence_first_point = i;
			}

			// start to end
			// if (is_path_traversable == 1) {
			// 	continue;
			// }
			// else {
			// 	path_push_back(&smoothed_path,path_get(path, i));
			// 	sequence_first_point = i-1;
			// }
		}
	}
	//path_push_back(&smoothed_path,path_get(path,last_checked_point));
	//path_push_back(&smoothed_path,*path_end(path));
	//path_push_back(&smoothed_path,*path_end(path));
	return smoothed_path;
}

/* function calculating next step based on current state for swf algorithm */
int next_step_swf(void){
	sem_wait(&position_orientationSemaphore);
	double tmp_pos_x = position_x;
	double tmp_pos_y = position_y;
	double tmp_orientation = orientation;
	sem_post(&position_orientationSemaphore);

	int tmp_current_swf_step = current_swf_step;

	if (movement_mode_swf == 1){
		if (is_path_calculated == 0){
			update_bt_swf_map();
			//printf(" ... map updated\n");
			int status = create_bt_list_swf();
			//printf("... bt list created\n");
			int bt_point;
			bt_point = select_bt_point_swf();
			//printf("... bt point selected\n");
			if (bt_point < 0) {
				algorithm_finished = 1;
				printf("\nalgorithm finished \n");
				return EXIT_SUCCESS;
			}

			int begin_x = (int)round(4*(tmp_pos_x));
			int begin_y = (int)round(4*(tmp_pos_y));

			grid_uint8_t grid;
			init_a_grid_swf(&grid);
			coordinate_t begin = {begin_x,begin_y};
			coordinate_t end = {bt_list_swf[bt_point][0],bt_list_swf[bt_point][1]};
			//print_begin_end(&begin, &end);
			path_t path_raw_swf = astar_uint8_get_path(&grid,end,begin, is_point_traversable);

			path_index_swf = 0;
			if(path_empty(&path_raw_swf)) {
				//printf("%d %d\n", swf_bt_map[begin_x][begin_y],swf_bt_map[bt_list_swf[bt_point][0]][bt_list_swf[bt_point][1]]);
				algorithm_finished = 1;
				//printf("No path found!\nFinishing work...\n");
			}
			//else {
				//printf("Path before smoothing:\n");
				//path_for_each_r(&path_raw_swf, coordinate_print);
				//printf("\n");
			//}
			//printf("Path after smoothing:\n");
			//path_t path2 = path;

			path_swf = smooth_path_swf(&path_raw_swf);

			path_destroy(&path_raw_swf);
			// path_for_each_r(&path_swf, coordinate_print);
			// printf("\n");
			// printf("size: %zu capacity: %zu\n",path_swf.size,path_swf.capacity);
			// fflush(stdout);
			grid_uint8_destroy(&grid);

			double dest_x = ((double)(path_get(&path_swf,path_index_swf)).row)/4;
			double dest_y = ((double)(path_get(&path_swf,path_index_swf)).col)/4;
			target_orientation_swf = calculate_target_angle(tmp_pos_x,tmp_pos_y,dest_x,dest_y);

			bt_target_swf_x = dest_x;
			bt_target_swf_y = dest_y;

			int a = rotation_direction(target_orientation_swf,tmp_orientation);
			if (a == 1){
				current_swf_step = 5;
			}
			else{
				current_swf_step = 4;
			}
			is_path_calculated = 1;
			spool_next_step_swf_calculated = 1;
		}
		else {
			
			switch (tmp_current_swf_step){
				case 4 ... 5:
					target_position_x_swf = bt_target_swf_x;
					target_position_y_swf = bt_target_swf_y;
					current_swf_step = 3;
					spool_next_step_swf_calculated = 1;
					break;
				case 3:
					path_index_swf++;
					if (path_index_swf == path_size(&path_swf)){
						// start new movement somehow
						//printf("begin new swf movement\n");

						if (swf_bt_map[(int)round(4*(tmp_pos_x))][(int)round(4*(tmp_pos_y))+1] == 1) {
							target_orientation_swf = 90.0;
							//printf("90\n");
						}
						else if (swf_bt_map[(int)round(4*(tmp_pos_x))][(int)round(4*(tmp_pos_y))-1] == 1) {
							target_orientation_swf = 270.0;
							//printf("270\n");
						}
						else if (swf_bt_map[(int)round(4*(tmp_pos_x))+1][(int)round(4*(tmp_pos_y))] == 1) {
							target_orientation_swf = 0.0;
							//printf("0\n");
						}
						else if (swf_bt_map[(int)round(4*(tmp_pos_x))-1][(int)round(4*(tmp_pos_y))] == 1) {
							target_orientation_swf = 180.0;
							//printf("180\n");
						}

						movement_mode_swf = 0;
						is_path_calculated = 0;
						new_loop = 1;
						target_position_x_swf = tmp_pos_x + 0.125 * cos(target_orientation_swf * ST_TO_RAD);
						target_position_y_swf = tmp_pos_y + 0.125 * sin(target_orientation_swf * ST_TO_RAD);
						int a = rotation_direction(target_orientation_swf,tmp_orientation);
						if (a == 1){
							current_swf_step = 5;
						}
						else{
							current_swf_step = 4;
						}
						spool_next_step_swf_calculated = 1;
						path_destroy(&path_swf);
						break; 
					}
				else {
					bt_target_swf_x = ((double)(path_get(&path_swf,path_index_swf).row))/4;
					bt_target_swf_y = ((double)(path_get(&path_swf,path_index_swf).col))/4;
					//printf("target_x_px: %d target_y_px: %d\n", (int)path_get(&path,path_index).row,(int)path_get(&path,path_index).col);
					target_orientation_swf = calculate_target_angle(tmp_pos_x,tmp_pos_y,bt_target_swf_x,bt_target_swf_y);
					int a = rotation_direction(target_orientation_swf,tmp_orientation);
					if (a == 1){
							current_swf_step = 5;
						}
						else{
							current_swf_step = 4;
						}
					spool_next_step_swf_calculated = 1;
					break;
				}					
			}

		}
		return EXIT_SUCCESS;		
	}
	

	switch(tmp_current_swf_step){
		case 0: 
			current_swf_step = 0;
			break;
		case 1:
			is_updatable = 1;
			target_orientation_swf = fmod(tmp_orientation + 90.0,360.0);
			current_swf_step = 4;
			break;
		case 2:
			if (new_wall_parameter == 0) {
				target_position_x_swf = tmp_pos_x + (0.08 + smaller_right*0.08) * cos(tmp_orientation * ST_TO_RAD);
				target_position_y_swf = tmp_pos_y + (0.08 + smaller_right*0.08) * sin(tmp_orientation * ST_TO_RAD);
				//printf("x: %.4f y: %.4f target x: %.4f target y: %.4f", tmp_pos_x,tmp_pos_y,target_position_x_swf,target_position_y_swf);
				current_swf_step = 3;
				//current_swf_step = 5;
				target_orientation_swf = 90.0 * round(fmod(tmp_orientation + 360.0 - 90.0,360.0)/90);
				is_at_corner = 1;
				new_loop_plan();
			}
			else if (new_wall_parameter == 1) {
				current_swf_step = 4;
				target_orientation_swf = 90.0 * round(fmod(tmp_orientation + 360.0 + 90.0,360.0)/90);
			}
			else {
				current_swf_step = 5;
				target_orientation_swf = 90.0 * round(fmod(tmp_orientation + 360.0 - 90.0,360.0)/90);
			}
			
			break;
		case 3:
 			if (is_at_corner == 1){
				current_swf_step = 5;
			}
			else {
				current_swf_step = 2;
			}
			break;

		case 4:
			if (new_loop == 1){
				new_loop = 0;
				current_swf_step = 3;
			}
			else {
				target_position_x_swf = tmp_pos_x + 0.1 * cos(tmp_orientation * ST_TO_RAD);
				target_position_y_swf = tmp_pos_y + 0.1 * sin(tmp_orientation * ST_TO_RAD);
				current_swf_step = 3;
				new_loop_plan();
			}
			break;
		case 5:
			if (new_loop == 1){
				new_loop = 0;
				current_swf_step = 3;
			}
			else if (is_at_corner == 1){
				target_position_x_swf = tmp_pos_x + (0.25 + 0.1*smaller_right)* cos(tmp_orientation * ST_TO_RAD);
				target_position_y_swf = tmp_pos_y + (0.25 + 0.1*smaller_right)* sin(tmp_orientation * ST_TO_RAD);
				//printf("%f %f \n", target_position_x_swf, target_position_y_swf);
				current_swf_step = 3;
				new_loop_plan();
				is_at_corner = 0;
			}
			else{
				current_swf_step = 2;
			}
			break;

	}
	//printf("step: %d\n",current_swf_step);
	spool_next_step_swf_calculated = 1;
	return EXIT_SUCCESS;
}

/* function supervising swf movement */
int swf_mov_superv(double tmp_pos_x, double tmp_pos_y, double tmp_orientation,
                   double tmp_front_sensor, double tmp_back_sensor,
                   double tmp_left_sensor, double tmp_right_sensor) {
	int tmp_movement_type = 0;

	sem_wait(&virtual_sensorsSemaphore);
	smaller_front = (tmp_front_sensor <= virt_sensor_front) ? 1 : 0;
	smaller_right = (tmp_right_sensor <= virt_sensor_right) ? 1 : 0;
	smaller_left =  (tmp_left_sensor  <= virt_sensor_left)  ? 1 : 0;

	tmp_front_sensor = fmin(tmp_front_sensor,virt_sensor_front);
	tmp_right_sensor = fmin(tmp_right_sensor,virt_sensor_right);
	tmp_left_sensor =  fmin(tmp_left_sensor,virt_sensor_left);
	sem_post(&virtual_sensorsSemaphore);	

	switch(current_swf_step) {
		case 0:
			tmp_movement_type = 0;
			break;
		case 1:	
			if (tmp_front_sensor < 0.005){
				tmp_movement_type = 0;
				spool_next_step_swf_calculated = 0;
				sem_post(&spool_calc_next_step_swf);
			}
			else {
				tmp_movement_type = 1;
			}
			break;
		case 2:
			if (tmp_right_sensor > 0.05) {
				//printf("end of wall, right sensor:  %.4f\n", tmp_right_sensor);
				//printf("real right: %.4f  virtual right: %.4f\n\n", right_sensor,virt_sensor_right);
				tmp_movement_type = 0;
				new_wall_parameter = 0;
				spool_next_step_swf_calculated = 0;
				sem_post(&spool_calc_next_step_swf);
			}
			else if (tmp_front_sensor < 0.0075) {
				if (tmp_front_sensor < 0.005 &&
					tmp_right_sensor < 0.005 &&
					tmp_left_sensor  < 0.005){
						tmp_movement_type = 0;
						spool_next_step_swf_calculated = 0;
						movement_mode_swf = 1;
						//printf("lets calculate new path\n");
						sem_post(&spool_calc_next_step_swf);
						return tmp_movement_type;
				}
				else {		
					if (tmp_left_sensor < 0.01)	{

						tmp_movement_type = 0;
						spool_next_step_swf_calculated = 0;
						movement_mode_swf = 1;
						//printf("lets calculate new path\n");
						sem_post(&spool_calc_next_step_swf);
						return tmp_movement_type;
					}
					else {	
						//printf("new wall, front sensor:  %.4f\n", tmp_front_sensor);
						//printf("real front: %.4f  virtual front: %.4f\n\n", front_sensor,virt_sensor_front);
						tmp_movement_type = 0;
						new_wall_parameter = 1;
						spool_next_step_swf_calculated = 0;
						sem_post(&spool_calc_next_step_swf);
					}
				}
			}
			else {
				tmp_movement_type = 1;
			}
			break;
		case 3:
			dist_swf = sqrt(pow(target_position_x_swf - tmp_pos_x,2) + 
							pow(target_position_y_swf - tmp_pos_y,2));
			//printf("dist_swf = %.4f   prev = %.4f\n", dist_swf, prev_dist_swf);
			if (dist_swf >= 0.001 && prev_dist_swf >= dist_swf) {
				prev_dist_swf = dist_swf;
				tmp_movement_type = 1;
			}
			else {
				prev_dist_swf = 10000;
				tmp_movement_type = 0;
				spool_next_step_swf_calculated = 0;
				sem_post(&spool_calc_next_step_swf);
			}
			break;
		case 4:
			if (fabs(target_orientation_swf - fmod(tmp_orientation,360.0)) > 0.1) {
				tmp_movement_type = 3;
			}
			else {
				tmp_movement_type = 0;
				spool_next_step_swf_calculated = 0;
				sem_post(&spool_calc_next_step_swf);
			}
			break;
		case 5:
			//if(movement_mode_swf == 1){printf("%.4f\n",fabs(target_orientation_swf - fmod(tmp_orientation,360.0)));}
			if (fabs(target_orientation_swf - fmod(tmp_orientation,360.0)) > 0.1) {
				tmp_movement_type = 2;
			}
			else {
				tmp_movement_type = 0;
				spool_next_step_swf_calculated = 0;
				sem_post(&spool_calc_next_step_swf);
			}
			break;
		default: 
		tmp_movement_type = 0;
		break;
	}
	return tmp_movement_type;
}

/* function updating swf discovery map */
int update_swf_map(void) {

	int status;

	sem_wait(&position_orientationSemaphore);
	double tmp_orientation = orientation;
	double tmp_position_x = position_x;
	double tmp_position_y = position_y;
	sem_post(&position_orientationSemaphore);

	sem_wait(&dist_sensorsSemaphore);
	double tmp_front_sensor = front_sensor;
	double tmp_back_sensor  = back_sensor;
	double tmp_left_sensor  = left_sensor;
	double tmp_right_sensor = right_sensor;
	sem_post(&dist_sensorsSemaphore);

	int this_pixel_x 	   = (int)round(8*(tmp_position_x));
	int this_pixel_y 	   = (int)round(8*(tmp_position_y));

	int left_sensor_pix_x  = (int)round(8*(tmp_position_x - 0.175 * (sin(ST_TO_RAD * tmp_orientation))));  
	int left_sensor_pix_y  = (int)round(8*(tmp_position_y + 0.175 * (cos(ST_TO_RAD * tmp_orientation))));
	int right_sensor_pix_x = (int)round(8*(tmp_position_x + 0.175 * (sin(ST_TO_RAD * tmp_orientation))));
	int right_sensor_pix_y = (int)round(8*(tmp_position_y - 0.175 * (cos(ST_TO_RAD * tmp_orientation))));
	int front_sensor_pix_x = (int)round(8*(tmp_position_x + 0.175 * (cos(ST_TO_RAD * tmp_orientation))));
	int front_sensor_pix_y = (int)round(8*(tmp_position_y + 0.175 * (sin(ST_TO_RAD * tmp_orientation))));
	
	// printf("raw left: %f %f\n",tmp_position_x - 0.25 * (sin(ST_TO_RAD * tmp_orientation)),tmp_position_y + 0.25 * (cos(ST_TO_RAD * tmp_orientation)));
	// printf("this: %d %d\n",this_pixel_x,this_pixel_y);
	// printf("f:%d %d    l:%d %d     r:%d %d\n",front_sensor_pix_x,front_sensor_pix_y,left_sensor_pix_x,left_sensor_pix_y,right_sensor_pix_x,right_sensor_pix_y);

	if (((int) round(tmp_orientation) % 90) == 0) {

		for (double i = -0.125; i <= 0.125; i += 0.025){
			for (double j = -0.125; j <= 0.125; j += 0.025){
				if (sqrt(pow(i,2) + pow(j,2)) <= 0.125) {
						if (swf_disc_map[(int)round(20*(tmp_position_x + i))][(int)round(20*(position_y + j))] < 2) {
							swf_disc_map[(int)round(20*(tmp_position_x + i))][(int)round(20*(position_y + j))] = 2; 
					}	
				}
			}
		}
	}


	/* write a map for debugging */
    // FILE *fptr;
    // char c;
    // fptr = fopen("../../roomba/log/map_swf.txt","w");
    // for(int i = 0; i < 160; i++) {
    //     for(int j = 0; j < 160; j++){
    //         c = swf_disc_map[j][160-i] +'0';
    //         fputc(c,fptr);
    //     }
    //     fprintf(fptr,"\r\n");
    // }
    // fclose(fptr);

	return EXIT_SUCCESS;
}

/* function calcucating virtual sensors (distance from visited point) indications */
int virtual_sensors(void) {

	sem_wait(&position_orientationSemaphore);
	double front_orientation = orientation;
	double x_coord = position_x;
	double y_coord = position_y;
	sem_post(&position_orientationSemaphore);

	double right_orientation = (front_orientation + 270.0) * ST_TO_RAD;
	double left_orientation  = (front_orientation +  90.0) * ST_TO_RAD;
	front_orientation = front_orientation * ST_TO_RAD;

	double front_x = x_coord + 0.05 * cos(front_orientation);
	double front_y = y_coord + 0.05 * sin(front_orientation);
	double right_x = x_coord + 0.05 * cos(right_orientation);
	double right_y = y_coord + 0.05 * sin(right_orientation);
	double left_x  = x_coord + 0.05 * cos(left_orientation);
	double left_y  = y_coord + 0.05 * sin(left_orientation);

	int this_pix_x = round(20 * x_coord);
	int this_pix_y = round(20 * y_coord);

	int f_flag = 0;
	int r_flag = 0;
	int l_flag = 0;

	double temp_front_sensor = 1.0;
	double temp_right_sensor = 1.0;
	double temp_left_sensor = 1.0;

	sem_wait(&swf_planSemaphore);
	for (double i = 0.0; i < 5.0; i = i + 0.05) {
		// front sensor
		if (!f_flag) {
			// determine closest pixel on background matrix
			int f_x_pix = round(20 * (front_x + i * cos(front_orientation)));
			int f_y_pix = round(20 * (front_y + i * sin(front_orientation)));
			if (swf_plan[f_x_pix][f_y_pix] == 0){// &&
				//this_pix_x != f_x_pix && this_pix_y != f_y_pix) {
				temp_front_sensor = i/10;
				//printf("%f\n",i);
				// printf("f_x=%.2f f_y=%.2f %d %d\n",front_x,front_y,f_x_pix,f_y_pix);
				f_flag = 1;
			}
		}
		if (!r_flag) {
			// determine closest pixel on background matrix
			int r_x_pix = round(20 * (right_x + i * cos(right_orientation)));
			int r_y_pix = round(20 * (right_y + i * sin(right_orientation)));
			if (swf_plan[r_x_pix][r_y_pix] == 0){// &&
				//this_pix_x != r_x_pix && this_pix_y != r_y_pix) {
				temp_right_sensor = i/10;
				//printf("%f\n",i);
				r_flag = 1;
			}
		}
		if (!l_flag) {
			// determine closest pixel on background matrix
			int l_x_pix = round(20 * (left_x + i * cos(left_orientation)));
			int l_y_pix = round(20 * (left_y + i * sin(left_orientation)));
			if (swf_plan[l_x_pix][l_y_pix] == 0){// &&
				//this_pix_x != l_x_pix && this_pix_y != l_y_pix) {
				temp_left_sensor = i/10;
				//printf("%f\n",i);
				l_flag = 1;
			}
		}
	}
	sem_post(&swf_planSemaphore);
	sem_wait(&virtual_sensorsSemaphore);
	virt_sensor_front = temp_front_sensor;
	virt_sensor_right = temp_right_sensor;
	virt_sensor_left = temp_left_sensor;
	sem_post(&virtual_sensorsSemaphore);

	// printf("c_x: %f,      c_y: %f    %f\n",position_x,position_y,orientation);
	// printf("v_front: %.4f v_right: %.4f\n\n", virt_sensor_front,virt_sensor_right);
	//printf("l: %.4f r: %.4f f: %.4f\n", virt_sensor_left,virt_sensor_right,virt_sensor_front);
	return EXIT_SUCCESS;
}

/* initialize plan with ones for swf */
void init_plan(){
    for(int i = 0; i < 400; i++) {
        for(int j = 0; j < 400; j++) {
            if (!(j == 400)) {
                swf_plan[j][400-i] = 1;
            }
        }
    }
    return;
}

/* update obstables/visited points grid for virtual sensors */
int new_loop_plan(void) {
	sem_wait(&swf_planSemaphore);
	for (int i = 0; i < 400; i++){
		for (int j = 0; j<  400; j++){
			if (swf_disc_map[i][j] > 1){
				swf_plan[i][j] = 0;
			}
		}
	}
	for (double i = -0.125; i <= 0.125; i += 0.025){
				for (double j = -0.125; j <= 0.125; j += 0.025){
					if (sqrt(pow(i,2) + pow(j,2)) <= 0.125) {
							if (swf_plan[(int)round(20*(position_x + i))][(int)round(20*(position_y + j))] == 0) {
								swf_plan[(int)round(20*(position_x + i))][(int)round(20*(position_y + j))] = 1; 
						}	
					}
				}
			}
		/* write a map for debugging */
		// FILE *fptr;
		// char c;
		// fptr = fopen("../../roomba/log/map_swf_new_loop.txt","w");
		// for(int i = 0; i < 160; i++) {
		//     for(int j = 0; j < 160; j++){
		//         c = swf_plan[j][160-i] +'0';
		//         fputc(c,fptr);
		//     }
		//     fprintf(fptr,"\r\n");
		// }
		// fclose(fptr);

	sem_post(&swf_planSemaphore);
	return EXIT_SUCCESS;
}

/* function creating bt list for SWF */
int create_bt_list_swf(void) {

	memset(bt_list_swf,0,sizeof(bt_list_swf));
	bt_list_swf[0][0] = 0;
	bt_list_swf[0][1] = 0;
	int k = 0;

	for (int i = 1; i < 80; i++){
		for (int j = 1; j < 80; j++){
			if (swf_bt_map[i][j] == 1){
				int b1 = swf_bt_map[i+1][j];
				int b2 = swf_bt_map[i][j+1];
				int b3 = swf_bt_map[i-1][j];
				int b4 = swf_bt_map[i][j-1];
				if (b1 > 1 && b2 > 1 && b3 > 1 && b4 > 4){
					swf_bt_map[i][j] = 2;
				}
			}

			if (swf_bt_map[i][j] == 2) {

				int s1 = swf_bt_map[i+1][j];
				int s2 = swf_bt_map[i+1][j+1];
				int s3 = swf_bt_map[i][j+1];
				int s4 = swf_bt_map[i-1][j+1];
				int s5 = swf_bt_map[i-1][j];
				int s6 = swf_bt_map[i-1][j-1];
				int s7 = swf_bt_map[i][j-1];
				int s8 = swf_bt_map[i+1][j-1];

				int goal = 0;
				goal += (s1 == 1 && (s2 != 2)) ? 1 : 0;
				goal += (s1 == 1 && (s8 != 2)) ? 1 : 0;
				goal += (s5 == 1 && (s6 != 2)) ? 1 : 0;
				goal += (s5 == 1 && (s4 != 2)) ? 1 : 0;
				goal += (s7 == 1 && (s6 != 2)) ? 1 : 0;
				goal += (s7 == 1 && (s8 != 2)) ? 1 : 0;
				goal += (s3 == 1 && (s2 != 2)) ? 1 : 0;
				goal += (s3 == 1 && (s4 != 2)) ? 1 : 0;

				// if (s1 > 1 && s3 > 1 && s5 > 1 && s7 > 1){
				// 	goal = 0;
				// }


				if ((goal >= 1) && !(i == (int)round(4*position_x) && j == (int)round(4*position_y))) {
					bt_list_swf[k][0] = i;
					bt_list_swf[k][1] = j;
					k++;
				}
			}

		}
	}
	for (int z = 0; bt_list_swf[z][0] != 0; z++){
		//printf("\n{%d,%d}\n",bt_list_swf[z][0],bt_list_swf[z][1]);
	}

	return EXIT_SUCCESS;
}

/* function updating map for bt list creation of SWF */
int update_bt_swf_map(void) {

	int status;

	sem_wait(&position_orientationSemaphore);
	double tmp_orientation = orientation;
	double tmp_position_x = position_x;

	double tmp_position_y = position_y;
	sem_post(&position_orientationSemaphore);

	sem_wait(&dist_sensorsSemaphore);
	double tmp_front_sensor = front_sensor;
	double tmp_back_sensor  = back_sensor;
	double tmp_left_sensor  = left_sensor;
	double tmp_right_sensor = right_sensor;
	sem_post(&dist_sensorsSemaphore);

	int this_pixel_x 	   = (int)round(4*(tmp_position_x));
	int this_pixel_y 	   = (int)round(4*(tmp_position_y));

	int this_pixel_x_1 	   = (int)round(4*(tmp_position_x+0.075));
	int this_pixel_y_1 	   = (int)round(4*(tmp_position_y+0.075));
	int this_pixel_x_2 	   = (int)round(4*(tmp_position_x+0.075));
	int this_pixel_y_2 	   = (int)round(4*(tmp_position_y-0.075));
	int this_pixel_x_3 	   = (int)round(4*(tmp_position_x-0.075));
	int this_pixel_y_3 	   = (int)round(4*(tmp_position_y+0.075));
	int this_pixel_x_4 	   = (int)round(4*(tmp_position_x-0.075));
	int this_pixel_y_4 	   = (int)round(4*(tmp_position_y-0.075));

	int left_sensor_pix_x  = (int)round(4*(tmp_position_x - 0.25 * (sin(ST_TO_RAD * tmp_orientation))));  
	int left_sensor_pix_y  = (int)round(4*(tmp_position_y + 0.25 * (cos(ST_TO_RAD * tmp_orientation))));
	int right_sensor_pix_x = (int)round(4*(tmp_position_x + 0.25 * (sin(ST_TO_RAD * tmp_orientation))));
	int right_sensor_pix_y = (int)round(4*(tmp_position_y - 0.25 * (cos(ST_TO_RAD * tmp_orientation))));
	int front_sensor_pix_x = (int)round(4*(tmp_position_x + 0.25 * (cos(ST_TO_RAD * tmp_orientation))));
	int front_sensor_pix_y = (int)round(4*(tmp_position_y + 0.25 * (sin(ST_TO_RAD * tmp_orientation))));
	
	
	// printf("raw left: %f %f\n",tmp_position_x - 0.25 * (sin(ST_TO_RAD * tmp_orientation)),tmp_position_y + 0.25 * (cos(ST_TO_RAD * tmp_orientation)));
	// printf("this: %d %d\n",this_pixel_x,this_pixel_y);
	// printf("f:%d %d    l:%d %d     r:%d %d\n",front_sensor_pix_x,front_sensor_pix_y,left_sensor_pix_x,left_sensor_pix_y,right_sensor_pix_x,right_sensor_pix_y);


	if (((int) round(tmp_orientation) % 90) == 0) {

		if (swf_bt_map[this_pixel_x][this_pixel_y] != 2) {
			swf_bt_map[this_pixel_x][this_pixel_y] = 2; }

		if (swf_bt_map[left_sensor_pix_x][left_sensor_pix_y] < 2){
			if (tmp_left_sensor < 0.025) {
				swf_bt_map[left_sensor_pix_x][left_sensor_pix_y] = 3;
			}
			else {
				swf_bt_map[left_sensor_pix_x][left_sensor_pix_y] = 1;
			}
		}
		if (swf_bt_map[right_sensor_pix_x][right_sensor_pix_y] < 2){
			if (tmp_right_sensor < 0.025) {
				swf_bt_map[right_sensor_pix_x][right_sensor_pix_y] = 3;
			}
			else {

				swf_bt_map[right_sensor_pix_x][right_sensor_pix_y] = 1;
			}
		}
		if (swf_bt_map[front_sensor_pix_x][front_sensor_pix_y] < 2){
			if (tmp_front_sensor < 0.025) {
				swf_bt_map[front_sensor_pix_x][front_sensor_pix_y] = 3;
			}
			else {
				swf_bt_map[front_sensor_pix_x][front_sensor_pix_y] = 1;
			}
		}



		if (swf_bt_map[this_pixel_x_1][this_pixel_y_1] < 2) {
			swf_bt_map[this_pixel_x_1][this_pixel_y_1] = 2; }

		if (swf_bt_map[this_pixel_x_2][this_pixel_y_2] < 2) {
			swf_bt_map[this_pixel_x_2][this_pixel_y_2] = 2; }

		if (swf_bt_map[this_pixel_x_3][this_pixel_y_3] < 2) {
			swf_bt_map[this_pixel_x_3][this_pixel_y_3] = 2; }	
	}


	/* write a map for debugging */
    // FILE *fptr;
    // char c;
    // fptr = fopen("../../roomba/log/map_swf_bt.txt","w");
    // for(int i = 0; i < 80; i++) {
    //     for(int j = 0; j < 80; j++){
    //         c = swf_bt_map[j][80-i] +'0';
    //         fputc(c,fptr);
    //     }
    //     fprintf(fptr,"\r\n");
    // }
    // fclose(fptr);

	return EXIT_SUCCESS;
}

/* select the best bt point - swf */
int select_bt_point_swf(void) {
	// just distance for now

	sem_wait(&position_orientationSemaphore);
	double tmp_position_x = position_x;
	double tmp_position_y = position_y;
	double tmp_orientation = orientation;
	sem_post(&position_orientationSemaphore);

	int best_point = 0;
	double best_value = 10000;
	int bt_point_x;
	int bt_point_y;
	double i_value;

	if (bt_list_swf[0][0] == 0 && bt_list_swf[0][1] == 0) { return -1; }

	for (int i = 0; i < 100; i++) {
		if (bt_list_swf[i][0] == 0 && bt_list_swf[i][1] == 0) {break;}
		bt_point_x = bt_list_swf[i][0];
		bt_point_y = bt_list_swf[i][1];	
		i_value = sqrt(pow(((double)bt_point_x)/4 - tmp_position_x,2) +
				 	   pow(((double)bt_point_y)/4 - tmp_position_y,2));
		if (i_value < best_value) {
			best_value = i_value;
			best_point = i;
		}
	}
	//printf("selected point: x = %d, y = %d\n",bt_list_swf[best_point][0],bt_list_swf[best_point][1]);

	return best_point;
}

/* smooth path found by pathfinder for swf */
path_t smooth_path_swf(path_t* path) {

	path_t smoothed_path;
	path_init(&smoothed_path, 10);

	int is_path_traversable = 1;
	int last_checked_point = 0;
	int sequence_first_point = 0;
	int checked_point = 0;

	if (path_size(path) == 1) { 
		path_push_back(&smoothed_path,path_get(path,0));
		return smoothed_path;
	}

	while(last_checked_point != path_size(path)-1){
		// for(int i = sequence_first_point + 1; i < path_size(path); i++) {
		for(int i = path_size(path) - 1; i > sequence_first_point; i--) {
			double first_x 		= ((double)(path_get(path,sequence_first_point)).row)/4;
			double first_y 		= ((double)(path_get(path,sequence_first_point)).col)/4;
			double checked_x 	= ((double)(path_get(path,i)).row)/4;
			double checked_y 	= ((double)(path_get(path,i)).col)/4;

			// get angle between these 2 points
			double angle 	= calculate_target_angle(first_x,first_y,checked_x,checked_y);
			double distance = sqrt((pow(first_x-checked_x,2)+pow(first_y-checked_y,2)));
			is_path_traversable = 1;

			for (double j = 0.0; j <= distance; j = j + 0.1){
				// determining the closest pixel
				int x_pix = (int)round(4 * (first_x + j * cos(angle * ST_TO_RAD)));
				int y_pix = (int)round(4 * (first_y + j * sin(angle * ST_TO_RAD)));

				int x_pix_left =	(int)round(4 * ((first_x + j * cos(angle * ST_TO_RAD) - 0.1)));
				int x_pix_right = 	(int)round(4 * ((first_x + j * cos(angle * ST_TO_RAD) + 0.1)));
				int y_pix_up = 		(int)round(4 * ((first_y + j * sin(angle * ST_TO_RAD) + 0.1)));
				int y_pix_down = 	(int)round(4 * ((first_y + j * sin(angle * ST_TO_RAD) - 0.1)));

				int pix_value 		= swf_bt_map[x_pix][y_pix];
				int pix_value_left  = swf_bt_map[x_pix_left][y_pix];
				int pix_value_right = swf_bt_map[x_pix_right][y_pix];
				int pix_value_up    = swf_bt_map[x_pix][y_pix_up];
				int pix_value_down  = swf_bt_map[x_pix][y_pix_down];


				if ((pix_value == 2 && pix_value_left == 2 && pix_value_right == 2 && 
									  pix_value_up == 2	  && pix_value_down == 2) ||
									  x_pix_left == path_begin(path)->col ||
									  x_pix_right == path_begin(path)->col ||
									  y_pix_up == path_begin(path)->row ||
									  y_pix_down == path_begin(path)->row){
					continue; }
				else {
					is_path_traversable = 0;
					// printf("due to px %d, %d\n", x_pix,y_pix);
					break;
				}
			}
			// printf("\n");
			// printf("from point %d to point %d  ", sequence_first_point, i);
			// printf("is traversable path: %d\n", is_path_traversable);
			
			last_checked_point = i;

			// end to start
			if (is_path_traversable == 0) {
				if (i == sequence_first_point+1){
					path_push_back(&smoothed_path,path_get(path,i));
					sequence_first_point = i;
				}
				else {
					continue;
				}
			}
			else {
				path_push_back(&smoothed_path,path_get(path, i));
				sequence_first_point = i;
			}
		}
	}
	return smoothed_path;
}