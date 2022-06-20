#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <math.h>
#include <semaphore.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <fcntl.h>

#include "../include/control_functions.h"
#include "../include/load_plan.h"
#include "../include/stc_algorithm.h"
//#include "../include/stc_algorithm.h"

/* function initializing semaphores */
int initialize_semaphores(void) {
	if(	sem_init(&batterySemaphore, 0, 1) 				||
		sem_init(&containerSemaphore, 0, 1) 			||
		sem_init(&position_orientationSemaphore, 0, 1)	||
		sem_init(&dist_sensorsSemaphore, 0, 1) 			||
		sem_init(&controlSemaphore, 0, 1) 				||
		sem_init(&taskSemaphore, 0, 1)					||
		sem_init(&spool_set_new_task, 0, 0)				||
		sem_init(&movement_typeSemaphore, 0, 1)			||
		sem_init(&planSemaphore, 0, 1)					||
		sem_init(&trashSemaphore, 0 ,1)					||
		sem_init(&target_directionSemaphore, 0, 1)		||
		sem_init(&spool_calc_next_step_stc, 0, 1)		||
		sem_init(&calc_next_step_stcSemaphore, 0, 1))
		{
			return EXIT_FAILURE;
		}
	return EXIT_SUCCESS;
}

/* function initializing container full flag to 0 */
int init_battery_container(void) {
	container_full = 0;
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
int init_new_task(void) {
	current_task = 1;
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
	sem_wait(&trashSemaphore);
	int trash_sensor_tmp = trash_sensor;
	sem_post(&trashSemaphore);
	sem_wait(&containerSemaphore);
	int container_full_tmp = container_full;
	sem_post(&containerSemaphore);

	// set control based on movement type 
	sem_wait(&controlSemaphore);
	switch (movement_type)
	{
	case 0:
		// just standing
		left_motor_power = 0.0;
		right_motor_power = 0.0;
		suction_power = 0.0;
		break;
	case 1:
		// driving forward
		if (trash_sensor == 1) {
			// if there are trashes below - slow down and increase suction
			left_motor_power = 0.2;
			right_motor_power = 0.2;
			if (!container_full_tmp) { suction_power = 1.0; }
			else { suction_power = 0.0; }
		} else {
			// if there are no trashes below - drive forward with 0.5 suction
			left_motor_power = 1.0;
			right_motor_power = 1.0;
			if (!container_full_tmp) { suction_power = 0.5; }
			else { suction_power = 0.0; }
		}
		break;
	case 2:
		// rotating clockwise with suction 0.2
		left_motor_power = 1.0;
		right_motor_power = -1.0;
			if (!container_full_tmp) { suction_power = 0.2; }
			else { suction_power = 0.0; }
		break;
	case 3:
		// rotating counter clockwise with suction 0.2
		left_motor_power = -1.0;
		right_motor_power = 1.0;
			if (!container_full_tmp) { suction_power = 0.2; }
			else { suction_power = 0.0; }
		break;
	case 4:
		// driving backward 
		if (trash_sensor == 1) {
			// if there are trashes below - slow down and increase suction
			left_motor_power = 0.1;
			right_motor_power = 0.1;
			if (!container_full_tmp) { suction_power = 1.0; }
			else { suction_power = 0.0; }
		} else {
			// if there are no trashes below - drive forward with 0.5 suction
			left_motor_power = 1;
			right_motor_power = 1;
			if (!container_full_tmp) { suction_power = 0.5; }
			else { suction_power = 0.0; }
		}
		break;
	default:
		// default controls 0.0 for safety
		left_motor_power = 0.0;
		right_motor_power = 0.0;
		suction_power = 0.0;
		break;
	}

	sem_post(&controlSemaphore);
	return EXIT_SUCCESS;
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
	sem_wait(&planSemaphore);

	// calculate grade for each candidate
	for (int i = 0; i < 18; i++) {
		// flag to skip loop
		int flag = 0;
		// check the closest pixel for every candidate every 10cm in given direction
		// 10cm is most natural since 1 pixel represents 10x10cm square

		for (double j = 0.0; j < 20.0; j = j + 0.1){
			if(!flag){
				// determining the closest pixel
				int x_pix = round(10 * (tmp_x + j * cos(candidate_directions[i])));
				int y_pix = round(10 * (tmp_y + j * sin(candidate_directions[i])));

				// get that pixels value
				char c = plan[y_pix-1][x_pix-1];
				if (c == '0') { flag = 1; }														// 0 - obstacle
				else if (c == '1') { candidate_grades[i] = candidate_grades[i] + 1; }			// 1 - floor (not visited)
				else if (c == '2') { candidate_grades[i] = candidate_grades[i] -0.5; }			// 2 - floor (visited)
				else if (c == '3') { candidate_grades[i] = candidate_grades[i] + 2; }			// 3 - trashes
				else { continue; }

				// check also 4 neighbour pixels, because robot will visit them following that path
				c = plan[y_pix-2][x_pix-1];
				if (c == '0') { flag = 1; }
				else if (c == '1') { candidate_grades[i] = candidate_grades[i] + 1; }
				else if (c == '2') { candidate_grades[i] = candidate_grades[i] - 0.5; }
				else if (c == '3') { candidate_grades[i] = candidate_grades[i] + 2; }
				else { continue; }

				c = plan[y_pix-1][x_pix-2];
				if (c == '0') { flag = 1; }
				else if (c == '1') { candidate_grades[i] = candidate_grades[i] + 1; }
				else if (c == '2') { candidate_grades[i] = candidate_grades[i] - 0.5; }
				else if (c == '3') { candidate_grades[i] = candidate_grades[i] + 2; }
				else { continue; }

				c = plan[y_pix][x_pix-1];
				if (c == '0') { flag = 1; }
				else if (c == '1') { candidate_grades[i] = candidate_grades[i] + 1; }
				else if (c == '2') { candidate_grades[i] = candidate_grades[i] - 0.5; }
				else if (c == '3') { candidate_grades[i] = candidate_grades[i] + 2; }
				else { continue; }

				c = plan[y_pix-1][x_pix];
				if (c == '0') { flag = 1; }
				else if (c == '1') { candidate_grades[i] = candidate_grades[i] + 1; }
				else if (c == '2') { candidate_grades[i] = candidate_grades[i] - 0.5; }
				else if (c == '3') { candidate_grades[i] = candidate_grades[i] + 2; }
				else { continue; }
			}
		}
		// change the best candidate if current is better
		if (candidate_grades[i] > candidate_grades[best_candidate_index]){
			best_candidate_index = i;
		}
	}			
	sem_post(&planSemaphore);

	// calculate target direction in degrees and write to output
	sem_wait(&target_directionSemaphore);
	target_direction = (candidate_directions[best_candidate_index]) * 57.295779513;
	if (target_direction > 360) { target_direction -= 360; }
	else if (target_direction < 0) { target_direction += 360; }
	sem_post(&target_directionSemaphore);

	return EXIT_SUCCESS;
}

/* function inspecting distance sensors */
int inspect_sensors(void) {
	// robot needs to stop if there is obstacle close

	// read input values 
	sem_wait(&dist_sensorsSemaphore);
	double tmp_front_sensor = front_sensor;
	double tmp_back_sensor = back_sensor;
	double tmp_left_sensor = left_sensor;
	double tmp_right_sensor = right_sensor;
	sem_post(&dist_sensorsSemaphore);
	sem_wait(&taskSemaphore);
	int current_task_tmp = current_task;
	sem_post(&taskSemaphore);

	// if robot is driving, then check
	// cant hit obstacle while rotating, because robot is round
	if (current_task_tmp == 1){
		if (tmp_front_sensor < 0.025 || tmp_left_sensor < 0.01 || tmp_right_sensor < 0.01){
			// if close to wall, notify about need to select new direction
			sem_post(&spool_set_new_task);
		}
	}
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
	sem_wait(&taskSemaphore);
	double tmp_current_task = current_task;
	sem_post(&taskSemaphore);
	sem_wait(&target_directionSemaphore);
	double target_direction_tmp = target_direction;
	sem_post(&target_directionSemaphore);
	if (algorithm_select == 0){
		switch (current_task)
			{
			case 0:
				// just standing
				movement_type = 0;
				break;
			case 1:
				// driving forward
				movement_type = 1;
				break;
			case 2:
				// rotating clockwise until reaching target_direction, then driving forward
				if (fabs(tmp_orientation - target_direction_tmp) < 0.2) { 
					current_task = 1;
					movement_type = 1; 
				} else {
					movement_type = 2;
				}
				break;
			case 3: 
				// rotating counter clokcwise until reaching target direction, then driving forward
				if (fabs(tmp_orientation - target_direction_tmp) < 0.2) {
					current_task = 1;
					movement_type = 1;
				} else {
					movement_type = 3;
				}
				break;
			default: 
				break;
		}
	}
	else if (algorithm_select == 1){
		int status;
		sem_wait(&calc_next_step_stcSemaphore);
		switch(next_step)
		{
			case 0:
				// just standing
				movement_type = 0;
				break;
			case 1:
				// rotate counter clockwise
				if (fabs(tmp_orientation_stc_step - fmod(tmp_orientation,360.0)) > 0.03) {
					movement_type = 3;
				} else {
					movement_type = 0;
					//current_orientation = ((int)(tmp_orientation/90.0)+3)%4;
					current_orientation = (current_orientation + 1) % 4;
					//printf("\nangle diff %f\n",fabs(tmp_orientation_stc_step - tmp_orientation));
					//printf("finished rotating 222\n");
					// printf("increased semaphore\n");
					// fflush(stdout);
					//sem_post(&spool_calc_next_step_stc);
					status = calc_next_step();
				}
				break;
			case 2:
				if (fabs(tmp_orientation_stc_step - fmod(tmp_orientation,360.0)) > 0.03) {
					movement_type = 2;
					//printf("%f %f\n",tmp_orientation_stc_step,tmp_orientation);
				} else {
					movement_type = 0;
					current_orientation = (current_orientation + 3) % 4;
					//printf("\nangle diff %f\n",fabs(tmp_orientation_stc_step - tmp_orientation));
					//current_orientation = ((int)(tmp_orientation/90.0)+2)%4;
					//current_orientation =  (current_orientation > 0) ? ((current_orientation - 1) %4) : 3;
					//printf("finished rotating 111\n");
					//printf("\n");
					// printf("increased semaphore\n");
					// fflush(stdout);
					//sem_post(&spool_calc_next_step_stc);
					status = calc_next_step();
				}
				break;
			case 3:
				if(fabs(tmp_pos_x_stc_step - tmp_pos_x) >= 0.25 ||
				   fabs(tmp_pos_y_stc_step - tmp_pos_y) >= 0.25) {
					movement_type = 0;

					//printf("reached here\n");
					// printf("increased semaphore\n");
					// fflush(stdout);
					//sem_post(&spool_calc_next_step_stc);
					status = calc_next_step();
					
					//printf("here\n");
					//printf("dist x: %f dist y: %f",fabs(tmp_pos_x_stc_step - tmp_pos_x),fabs(tmp_pos_y_stc_step - tmp_pos_y));
				} else {
					//printf("dist x: %f dist y: %f",fabs(tmp_pos_x_stc_step - tmp_pos_x),fabs(tmp_pos_y_stc_step - tmp_pos_y));
					//printf("\nposition_x: %f position_y: %f\n",position_x, position_y);
					//printf("+1");
					movement_type = 1;
				}
				break;
		}
		sem_post(&calc_next_step_stcSemaphore);
	}

	return EXIT_SUCCESS;
}

/* function adding to plan visited points */
int update_plan(int previous_x_pix, int previous_y_pix, int current_x_pix, int current_y_pix) {
	// function updates array "plan" every time a new position from sim is received
	if (previous_x_pix > 0){
		if ((previous_x_pix != current_x_pix) || (previous_y_pix != current_y_pix)) {
			// pixels from previous data packet differ from current
			sem_wait(&planSemaphore);
			sem_wait(&position_orientationSemaphore);
			double tmp_orientation = orientation;
			sem_post(&position_orientationSemaphore);

			for (int i = -2; i < 3; i++) {
				for (int j = -2; j < 3; j++) {
					if (plan[previous_y_pix+i][previous_x_pix+j] == '1' ||		// '1' - floor (not visited)
						plan[previous_y_pix+i][previous_x_pix+j] == '3'){		// '3' - trashes 
						plan[previous_y_pix+i][previous_x_pix+j] = '2';			// '2' - visited
					}
				}
			}
			sem_post(&planSemaphore);
		}
	}

	return EXIT_SUCCESS;
}

/* check battery and container state */
int inspect_battery_and_container(void) {

	// read inputs
	sem_wait(&batterySemaphore);
	double tmp_battery = battery_level;
	sem_post(&batterySemaphore);
	sem_wait(&containerSemaphore);
	double tmp_container = container_level;
	sem_post(&containerSemaphore);

	// if battery is less than 1%, stop robot
	if (tmp_battery < 1) {
		sem_wait(&taskSemaphore);
		current_task = 0;
		sem_post(&taskSemaphore);
	}

	// if container is filled in 99.5% turn off suction
	if (tmp_container > 99.5) {
		sem_wait(&containerSemaphore);
		container_full = 1;
		sem_post(&containerSemaphore);
	}

	return EXIT_SUCCESS;
}