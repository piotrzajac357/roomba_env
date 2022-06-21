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
double sim_step_position = 0.01;
int sim_step_battery = 1;
int sim_step_container = 1;
double sim_step_dist_sensors = 0.002;
double max_speed = 1;
double max_rotating_speed = 0.08;
int trashes[30];

double position_x;
double position_y;
double previous_orientation;
double left_motor_power;
double right_motor_power;
double suction_power;
double battery_level;
double container_level;
double front_sensor;
double back_sensor;
double left_sensor;
double right_sensor;
int trash_sensor;   
int new_trashes[2];

int initialize_semaphores(void) {
	// initialization of semaphores
	if(	sem_init(&batterySemaphore, 0, 1) 				||
		sem_init(&containerSemaphore, 0, 1) 			||
		sem_init(&position_orientationSemaphore, 0, 1)	||
		sem_init(&dist_sensorsSemaphore, 0, 1) 			||
		sem_init(&controlSemaphore, 0, 1)				||
		sem_init(&trashSemaphore, 0, 1)					||
		sem_init(&planSemaphore, 0, 1)) {
			return 1;
		}
	return 0;
}

/* set starting position and orientation */
int initialize_position(void) {
	sem_wait(&position_orientationSemaphore);
	position_x = 13;
	position_y = 10;
	previous_orientation = 270.0;
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

/* set container state */
int initialize_container(void) {
	sem_wait(&containerSemaphore);
	container_level = 0.0;
	sem_post(&containerSemaphore);
	return EXIT_SUCCESS;
}

/* read from file trash coords to trash generator */
int initialize_trash(void) {
	FILE *fptr;
	fptr = fopen("../../roomba/plan/trashes.txt","r");
    if(fptr == NULL) {
        perror("Error in opening file");
        return EXIT_FAILURE;;
    }

	sem_wait(&trashSemaphore);
	for (int i = 0; i < 15; i++) {
        fscanf(fptr, "%d ", &trashes[2*i]);
		trashes[2*i] = 200 - trashes[2*i];
		fscanf(fptr, "%d\n", &trashes[2*i+1]);
    }
	sem_post(&trashSemaphore);

	return EXIT_SUCCESS;;
}

/* set first distance sensors values */
int initialize_dist_sensors(void) {
	// read house plan from file for calculating sensors
	init_plan();
	double front_sensor = 1.0;
	double back_sensor = 1.0;
	double left_sensor = 1.0;
	double right_sensor = 1.0;
	int trash_sensor = 0; 
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
		tmp_orientation = tmp_orientation + (-left_motor_power_tmp + right_motor_power_tmp) * sim_step_position * 360 * max_rotating_speed/2;
		
		// normalize result to 0-360 degree
		if (tmp_orientation >= 360) { tmp_orientation -= 360; }
		else if (tmp_orientation < 0) { tmp_orientation += 360; } 
	}

	// when left motor power is 1 and right motor power is -1, simulate rotating clockwise
	else if ((left_motor_power_tmp == 1 && right_motor_power_tmp == -1)){
		tmp_orientation = tmp_orientation - (left_motor_power_tmp - right_motor_power_tmp) * sim_step_position * 360 * max_rotating_speed/2;
		
		// normalize result to 0-360 degree
		if (tmp_orientation >= 360) {tmp_orientation -= 360; }
		else if ((tmp_orientation < 0)) { tmp_orientation += 360; } 
	}

	// when motors power values are equal, simulate displacement in given direction
 	else if (left_motor_power_tmp == right_motor_power_tmp){
		tmp_x = tmp_x + max_speed * left_motor_power_tmp * sim_step_position * (cos(tmp_orientation * ST_TO_RAD));
		tmp_y = tmp_y + max_speed * left_motor_power_tmp * sim_step_position * (sin(tmp_orientation * ST_TO_RAD));
	}

	// change here to adjust resolution
	tmp_x = round(tmp_x * 80)/80;
	tmp_y = round(tmp_y * 80)/80;
	//if ((fmod(tmp_orientation,1.0) < 0.01) || (fmod(tmp_orientation,1.0) > 0.99)){tmp_orientation = round(tmp_orientation);}
	tmp_orientation = round(tmp_orientation * 10)/10;
	// write output values
	sem_wait(&position_orientationSemaphore);
	position_x = tmp_x;
	position_y = tmp_y;
	previous_orientation = tmp_orientation;
	sem_post(&position_orientationSemaphore);
	//printf("%f", tmp_orientation);
	return 0;
}

/* function calculating battery state */
int calculate_battery(void) {

	/* battery changes by:
    	~0.1% battery every second at max power 
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
	battery_level = battery_level - 0.1 * (0.05 	+ 0.25 * (fabs(left_motor_power_tmp)) 
                                                	+ 0.25 * (fabs(right_motor_power_tmp)) 
                                                 	+ 0.45 * suction_power_tmp);
    if (battery_level < 0.0) { battery_level = 0.0; }
	sem_post(&batterySemaphore);

    return 0;
}

/* function calculating container state */
int calculate_container(void) {

	/* container filling speed depends on suction power and trash presence */

	// read input values
	sem_wait(&controlSemaphore);
	double suction_power_tmp = suction_power;
	sem_post(&controlSemaphore);
	sem_wait(&trashSemaphore);
	int trash_sensor_tmp = trash_sensor;
	sem_post(&trashSemaphore);

	// calculate and write to output
	sem_wait(&containerSemaphore);
	container_level = container_level + suction_power_tmp * (0.6 + 0.6 * trash_sensor_tmp);
	if (container_level > 100.0) { container_level = 100.0; }
	sem_post(&containerSemaphore);

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

	/* function also determines whether there are trashes below robot by simply checking the closest pixel */

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

	// tmp trash sensor value
	int tmp_trash_sensor = 0;

	// initialize sensors indications to 1.0 (closest obstacle farther than 10m)
	double temp_front_sensor = 1.0;
	double temp_back_sensor = 1.0;
	double temp_left_sensor = 1.0;
	double temp_right_sensor = 1.0;

	// check for obstacles every 5cm
	for (double i = 0.0; i < 10.0; i = i + 0.02) {
		// front sensor
		if (!f_flag) {
			// determine closest pixel on background matrix
			int f_x_pix = round(10 * (front_x + i * cos(front_orientation)));
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
			}
		}
	}

	// check the closest pixel's value in background matrix 
	// '3' means trashes
	sem_wait(&planSemaphore);
	if (plan[(int)round(10*(y_coord))][(int)round(10*(x_coord))] == '3') { tmp_trash_sensor = 1; }
	else { tmp_trash_sensor = 0; }
	sem_post(&planSemaphore);

	// write values to output
	sem_wait(&dist_sensorsSemaphore);
	front_sensor = temp_front_sensor;
	back_sensor = temp_back_sensor;
	left_sensor = temp_left_sensor;
	right_sensor = temp_right_sensor;
	sem_post(&dist_sensorsSemaphore);
	sem_wait(&trashSemaphore);
	trash_sensor = tmp_trash_sensor;
	sem_post(&trashSemaphore);

	return 0;
}

/* trashes generator function */
int generate_trashes(int *thread_counter_ptr) {

	/*	function fills 9x9 pixel area (0.9m x 0.9m) in background matrix with trashes ('3' values)
		center points are predefined and read from file */

	// there are 15 different places for trashes
	if (*thread_counter_ptr == 15) { *thread_counter_ptr = 0;}
	int thread_counter = *thread_counter_ptr;
	 	
	// change 9x9 area in background to '3'
    sem_wait(&planSemaphore);
    for (int i = -4; i <= 4; i++) {
        for (int j = -4; j <= 4; j++){
            plan[(trashes[thread_counter*2])+i][(trashes[thread_counter*2+1])+j] = '3';
        }
    }
    sem_post(&planSemaphore);

	// change latest generated trashes coords so control and vis process can reconstruct them
    sem_wait(&trashSemaphore);
    new_trashes[0] = trashes[thread_counter*2];
    new_trashes[1] = trashes[thread_counter*2+1]; 
    sem_post(&trashSemaphore);

	return EXIT_SUCCESS;
}