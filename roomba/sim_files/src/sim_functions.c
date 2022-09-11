/* File with functions calculating various things */

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

#include "../include/sim_functions.h"
#include "../include/load_plan.h"
#include "../include/sim.h"

/* "Read only" variables definitions */
double sim_step_position = 0.001;
int sim_step_battery = 1;
int sim_step_container = 1;
double sim_step_dist_sensors = 0.001;
double max_speed = 0.5;						// 0.5m/s
// double max_rotating_speed = 0.055556;			// 360deg * 1/18 this is 0.02 deg in iteration
double max_rotating_speed = 0.055556;			// 360deg * 1/18 this is 0.02 deg in iteration
double simulation_speed = 2;

double position_x;
double position_y;
double previous_orientation;
double left_motor_power;
double right_motor_power;
double suction_power;
double battery_level;
double front_sensor;
double back_sensor;
double left_sensor;
double right_sensor;

int initialize_semaphores(void) {
	// initialization of semaphores
	if(	sem_init(&batterySemaphore, 0, 1) 				||
		sem_init(&position_orientationSemaphore, 0, 1)	||	
		sem_init(&dist_sensorsSemaphore, 0, 1) 			||
		sem_init(&controlSemaphore, 0, 1)				||
		sem_init(&planSemaphore, 0, 1)					||
		sem_init(&qiSemaphore, 0, 1)) {
			return 1;
		}
	return 0;
}

/* set starting position and orientation */
int initialize_position(void) {
	sem_wait(&position_orientationSemaphore);
	// for stc_ba_showcase around 13.0, 16.0 
	/* STC position */
	// position_x = 12.875;
	// position_y = 16.125;
	// previous_orientation = 270.0;
	/* BA* position */
	// position_x = 12.75;
	// position_y = 16.25;
	// previous_orientation = 90.0;

	// for clover around 10.0, 10.0
	// position_x = 10.0;
	// position_y = 10.125;
	// previous_orientation = 270;

	// for narrow around 10.0, 10.0
	// position_x = 9.875;
	// position_y = 10.125;
	// previous_orientation = 270;

	// for obstacles around 10.0, 12.0
	// position_x = 10.0;
	// position_y = 12.125;
	// previous_orientation = 90;

	//corner positions showcase:
	// position_x = 2.25;
	// position_y = 2.75;
	// previous_orientation = 270.0;

	// corner position narrow:
	// position_x = 2.25;
	// position_y = 2.0;
	// previous_orientation = 270.0;

	// corner clover
	position_x = 2.25;
	position_y = 4.5;
	previous_orientation = 270.0;

	// corner obstacles
	// position_x = 4.25;
	// position_y = 2.1;
	// previous_orientation = 270.0;

	sem_post(&position_orientationSemaphore);
	return EXIT_SUCCESS;
}

/* set battery state */
int initialize_battery(void) {
	sem_wait(&batterySemaphore);
	battery_level = 100.0;
	sem_post(&batterySemaphore);
    return EXIT_SUCCESS;
}

/* set first distance sensors values */
int initialize_dist_sensors(void) {
	// read house plan from file for calculating sensors
	init_plan();
	double front_sensor = 1.0;
	double back_sensor = 1.0;
	double left_sensor = 1.0;
	double right_sensor = 1.0;
	return 0;
}

/* set first control values */
int initialize_control_values(void) {
	// set first control values to 0 before getting from control process 
	sem_wait(&controlSemaphore);
	left_motor_power = 0.0;
	right_motor_power = 0.0;
	suction_power = 0.0;
	sem_post(&controlSemaphore);
	return EXIT_SUCCESS;
	}

/* initialize quality indexes with starting values */
int initialize_quality_indexes(void) {

	coverage_qi_total = 1;
	coverage_acc = 0;
	sem_wait(&planSemaphore);
	for(int i = 0; i < 200; i++){
		for (int j = 0; j < 200; j++){
			for (int k = 0; k < 4; k++){
				for (int l = 0; l < 4; l++){
					int a = plan[i][j] - '0';
					coverage_plan[4*i+k][4*j+l] = a;
					coverage_qi_total += a;
				}
			}	 
		}
	}
	sem_post(&planSemaphore);

	// FILE *fptr2;
    // fptr2 = fopen("../../roomba/log/log_cov.txt","w");
    // fclose(fptr2);


	// FILE* fptr9;
	// char c;
	// fptr9 = fopen("../../roomba/log/map_cov.txt","w");
	// for (int i = 0; i < 800; i++) {
	// 	for (int j = 0; j < 800; j++) {
	// 		c = coverage_plan[i][j] + '0';
	// 		fputc(c, fptr9);
	// 	}
	// 	fprintf(fptr9, "\r\n");
	// }
	// fclose(fptr9);

	sem_wait(&qiSemaphore);
	time_QI = 0;
	path_QI = 0;
	rotation_QI = 0;
	coverage_QI = 0.0;
	idle_time_QI = 0.0;
	sem_post(&qiSemaphore);
	sem_wait(&position_orientationSemaphore);
	path_qi_prev_x = position_x;
	path_qi_prev_y = position_y;
	rotation_qi_prev = previous_orientation;
	sem_post(&position_orientationSemaphore);

	return EXIT_SUCCESS;
};


/* function calculating cartesian position of robot */
int calculate_position(void) {
/* 	Function calculates new position and orientation based on previous position orientation and motors powers */

	// read input position values
	sem_wait(&position_orientationSemaphore);
	double tmp_x = position_x;
	double tmp_y = position_y;
	double tmp_orientation = previous_orientation;
	sem_post(&position_orientationSemaphore);

	// read input control values
	sem_wait(&controlSemaphore);
	double left_motor_power_tmp = left_motor_power;
	double right_motor_power_tmp = right_motor_power;
	sem_post(&controlSemaphore);

	// when both motors power is 0, do not change position
	if ((left_motor_power_tmp == 0 && right_motor_power_tmp == 0)){}

	// when left motor power is -1 and right motor power is 1, simulate rotating counter clockwise
	else if (left_motor_power_tmp == -1 && right_motor_power_tmp == 1){
		// angle change depends on sim_step and max rotating speed
		tmp_orientation = tmp_orientation + (-left_motor_power_tmp + right_motor_power_tmp) * simulation_speed * sim_step_position * 360 * max_rotating_speed/2;
		
		// normalize result to 0-360 degree
		if (tmp_orientation >= 360) { tmp_orientation -= 360; }
		else if (tmp_orientation < 0) { tmp_orientation += 360; } 
	}

	// when left motor power is 1 and right motor power is -1, simulate rotating clockwise
	else if ((left_motor_power_tmp == 1 && right_motor_power_tmp == -1)){
		tmp_orientation = tmp_orientation - (left_motor_power_tmp - right_motor_power_tmp) * simulation_speed * sim_step_position * 360 * max_rotating_speed/2;
		
		// normalize result to 0-360 degree
		if (tmp_orientation >= 360) {tmp_orientation -= 360; }
		else if ((tmp_orientation < 0)) { tmp_orientation += 360; } 
	}

	// when motors power values are equal, simulate displacement in given direction
 	else if (left_motor_power_tmp == right_motor_power_tmp){
		tmp_x = tmp_x + simulation_speed * max_speed * left_motor_power_tmp * sim_step_position * (cos(tmp_orientation * ST_TO_RAD));
		tmp_y = tmp_y + simulation_speed * max_speed * left_motor_power_tmp * sim_step_position * (sin(tmp_orientation * ST_TO_RAD));
	}

	// change here to adjust resolution
	// tmp_x = round(tmp_x * 80)/80;
	// tmp_y = round(tmp_y * 80)/80;

	//if ((fmod(tmp_orientation,1.0) < 0.01) || (fmod(tmp_orientation,1.0) > 0.99)){tmp_orientation = round(tmp_orientation);}
	tmp_orientation = round(tmp_orientation * 1000)/1000;
	// write output values
	sem_wait(&position_orientationSemaphore);
	position_x = tmp_x;
	position_y = tmp_y;
	previous_orientation = tmp_orientation;
	sem_post(&position_orientationSemaphore);
	//printf("%f\n", tmp_orientation);
	return 0;
}

/* function calculating battery state */
int calculate_battery(void) {

	/* battery changes by:
		battery time at max power: 60min 
    	~0.027778% battery every second at max power 
		that value consists of:
    	0.05 static battery usage
    	0.45 cleaning/suction power
    	0.5  both motors
	*/

	// read input values
	sem_wait(&controlSemaphore);
	double left_motor_power_tmp = left_motor_power;
	double right_motor_power_tmp = right_motor_power;
	double suction_power_tmp = suction_power;
	sem_post(&controlSemaphore);

	// calculate and write to output
	sem_wait(&batterySemaphore);
	battery_level = battery_level -
			  0.0277778 * simulation_speed *  (0.05 + 0.25 * (fabs(left_motor_power_tmp)) 
                                                	+ 0.25 * (fabs(right_motor_power_tmp)) 
                                                 	+ 0.45 * suction_power_tmp);
    if (battery_level < 0.0) { battery_level = 0.0; }
	sem_post(&batterySemaphore);

    return 0;
}

/* function calculating distance sensors indications */
int calculate_sensors(void) {

	/*	simulation of distance sensors indications
		values are calculated based on robot orientation (0-360st)
	 	and distance to closest "nofloor" object ("0" in background matrix)
	 	distance is normalized to 0-1 range, 0 for pixel, 1.0 for more than 100pixels (10m)
	 	distance from sensor is calculated as sqrt(dx + dy)
		starting cartesian position of sensors is modified to include robot diameter  	*/

	// read input values
	sem_wait(&position_orientationSemaphore);
	double front_orientation = previous_orientation;
	double x_coord = position_x;
	double y_coord = position_y;
	sem_post(&position_orientationSemaphore);	

	// change orientations of each sensor and covert to radian
	double back_orientation = (front_orientation + 180.0) * ST_TO_RAD;
	double left_orientation = (front_orientation + 90.0) * ST_TO_RAD;
	double right_orientation = (front_orientation + 270.0) * ST_TO_RAD;
	front_orientation = front_orientation * ST_TO_RAD;

	// robot has 35cm diameter, so sensors are 17.5cm from center
	double front_x = x_coord + 0.175 * cos(front_orientation);
	double front_y = y_coord + 0.175 * sin(front_orientation);
	double back_x = x_coord + 0.175 * cos(back_orientation);
	double back_y = y_coord + 0.175 * sin(back_orientation);
	double left_x = x_coord + 0.175 * cos(left_orientation);
	double left_y = y_coord + 0.175 * sin(left_orientation);
	double right_x = x_coord + 0.175 * cos(right_orientation);
	double right_y = y_coord + 0.175 * sin(right_orientation);

	// set flags to escape for loop
	int f_flag = 0;
	int b_flag = 0;
	int l_flag = 0;
	int r_flag = 0;

	// initialize sensors indications to 1.0 (closest obstacle farther than 10m)
	double temp_front_sensor = 1.0;
	double temp_back_sensor = 1.0;
	double temp_left_sensor = 1.0;
	double temp_right_sensor = 1.0;

	// check for obstacles every 5cm
	for (double i = 0.0; i < 2.0; i = i + 0.025) {
		// front sensor
		if (!f_flag) {
			// determine closest pixel on background matrix
			int f_x_pix = round(10 * (front_x + i * cos(front_orientation)));

			//int for_testing = (int)round(10 * (front_x + i * cos(front_orientation)));
			//if (f_x_pix != for_testing ){printf("bingo karwasz jego twarz\n");};

			int f_y_pix = round(10 * (front_y + i * sin(front_orientation)));
			// if that pixel is obstacle - set sensor indication
			if (plan[f_y_pix][f_x_pix] == '0') {
				temp_front_sensor = i/10;
				// set flag to 1 to skip this sensor in next iteration
				f_flag = 1;
			}
		}
		// back sensor
		if (!b_flag) {
			int b_x_pix = round(10 * (back_x + i * cos(back_orientation)));
			int b_y_pix = round(10 * (back_y + i * sin(back_orientation)));
			if (plan[b_y_pix][b_x_pix] == '0') {
				temp_back_sensor = i/10;
				b_flag = 1;
			}
		}
		// left sensor
		if (!l_flag) {
			int l_x_pix = round(10 * (left_x + i * cos(left_orientation)));
			int l_y_pix = round(10 * (left_y + i * sin(left_orientation)));
			if (plan[l_y_pix][l_x_pix] == '0') {
				temp_left_sensor = i/10;
				l_flag = 1;
			}
		}
		// right sensor
		if(!r_flag) {
			int r_x_pix = round(10 * (right_x + i * cos(right_orientation)));
			int r_y_pix = round(10 * (right_y + i * sin(right_orientation)));
			if (plan[r_y_pix][r_x_pix] == '0') {
				temp_right_sensor = i/10;
				r_flag = 1;

				//temp_right_sensor = sqrt(pow(i * cos(right_orientation),2) + pow(i*sin(right_orientation),2))/10;

			}
		}
	}


	// write values to output
	sem_wait(&dist_sensorsSemaphore);
	front_sensor = temp_front_sensor;
	back_sensor = temp_back_sensor;
	left_sensor = temp_left_sensor;
	right_sensor = temp_right_sensor;
	sem_post(&dist_sensorsSemaphore);
	// printf("s_x: %f,    s_y: %f    %f\n",position_x,position_y,previous_orientation);
	// printf("s_front: %.4f s_right: %.4f \n",front_sensor,right_sensor);
	return 0;
}

/* function calculating time_QI */
int calculate_qis(double time_step) {

	double time_QI_tmp = time_QI;
	double path_QI_tmp = path_QI;
	double rotation_QI_tmp = rotation_QI;
	double coverage_QI_tmp;
	double idle_time_QI_tmp = idle_time_QI;

	sem_wait(&position_orientationSemaphore);
	double curr_x_tmp = position_x;
	double curr_y_tmp = position_y;
	double curr_orient = previous_orientation;
	sem_post(&position_orientationSemaphore);

	sem_wait(&controlSemaphore);
	double left_motor_tmp = left_motor_power;
	double right_motor_tmp = right_motor_power;
	sem_post(&controlSemaphore);

	// time quality index
	time_QI_tmp += time_step * simulation_speed;

	// travelled distance quality index
	path_QI_tmp += sqrt(pow(path_qi_prev_x - curr_x_tmp, 2)
					  + pow(path_qi_prev_y - curr_y_tmp, 2));

	// rotation done quality index
	double a = fabs(rotation_qi_prev - curr_orient);
	if (a > 180) { a = fabs(a - 360.0);	}
	rotation_QI_tmp += a;

	for (double i = -0.125; i <= 0.125; i = i + 0.0125){
		for (double j = -0.125; j <= 0.125; j = j + 0.0125){
			if (coverage_plan[(int)(round(40*(curr_y_tmp+i)))][(int)(round(40*(curr_x_tmp+j)))] == 1){
				coverage_plan[(int)(round(40*(curr_y_tmp+i)))][(int)(round(40*(curr_x_tmp+j)))] = 2;
				coverage_acc++;
				
			}
		}
	}
	coverage_QI_tmp = (double)coverage_acc / (double)coverage_qi_total;

	if (right_motor_tmp == 0 && left_motor_power == 0) {
		idle_time_QI_tmp += time_step;
	}


	path_qi_prev_x = curr_x_tmp;
	path_qi_prev_y = curr_y_tmp;
	rotation_qi_prev = curr_orient;

	sem_wait(&qiSemaphore);
	time_QI = time_QI_tmp;
	path_QI = path_QI_tmp;
	rotation_QI = rotation_QI_tmp;
	coverage_QI = coverage_QI_tmp;
	idle_time_QI = idle_time_QI_tmp;
	battery_QI = battery_level;
	sem_post(&qiSemaphore);
	
	// printf("%d	",coverage_acc);
	// printf("%f\n",coverage_QI);
	return EXIT_SUCCESS;
}