#ifndef CONTROL_FUNCTIONS_H_
#define CONTROL_FUNCTIONS_H_

#ifndef ST_TO_RAD
#define ST_TO_RAD 0.01745329251         // degrees to radians convert multiplier
#endif

int algorithm_select;                   // algorithm selection
                                        // 0 - random bouncing
                                        // 1 - stc

/* sensors and odometry and output variables */
double position_x;                      // Current cartesian position x coord (0-20m)
double position_y;                      // Current cartesian position y coord (0-20m)
double orientation;                     // Current orientation (0-360deg)
sem_t position_orientationSemaphore;    // Semaphore

double left_motor_power;                // Left motor power (0-1)
double right_motor_power;               // Right motor power (0-1)
double suction_power;                   // Suction/brushes power (0-1)
sem_t controlSemaphore;                 // Semaphore

double battery_level;                   // Battery state (0-100%)
sem_t batterySemaphore;                 // Semaphore

double container_level;                 // Container state (0-100%)
int container_full;                     // Flag, 1 - container full, 0 container not full
sem_t containerSemaphore;               // Semaphore

double front_sensor;                    // front distance sensor
double back_sensor;                     // back distance sensor
double left_sensor;                     // left distance sensor
double right_sensor;                    // right distance sensor
sem_t dist_sensorsSemaphore;            // Semaphore

char plan[200][200];                    // 200x200 char array representing house 
sem_t planSemaphore;                    // Semaphore

int trash_sensor;                       // trash (below) sensor
sem_t trashSemaphore;                   // Semaphore


/* RANDOM algorithm variables */

/*  current task to be done 
    0 - no driving, motors off
    1 - driving forward
    2 - rotating clockwise
    3 - rotating counter clockwise
    4 - driving backward
    could be more like going to station, driving by wall    */
int current_task;
sem_t taskSemaphore;                    // Semaphore

/*  movement type
    0 - standing
    1 - driving forward
    2 - rotating clockwise
    3 - rotating counter clockwise
    4 - driving backward   */
int movement_type;
sem_t movement_typeSemaphore;           // Semaphore

double target_direction;                // target direction to rotate to
sem_t target_directionSemaphore;        // Semaphore

sem_t spool_set_new_task;               // notification that new task has to be selected


/* STC ALGORITHM VARIABLES */
sem_t spool_calc_next_step_stc;         // notification for STC algorithm - calculate next step

int spool_next_step_calculated;         // flag that next step has been calculated


/* discovered plan of house
    0 - not checked (default)
    1 - not visited
    2 - visited
    3 - obstacle */
int disc_plan2[40][40];                 

/* parent cells of each cell
0 - not calculated
1 - from lower
2 - from right
3 - from upper
4 - from left */
int parent_cells[80][80];

int current_position_x;                 // robot position x for stc algorithm (rounded)
int current_position_y;                 // robot position y for stc algorithm (rounded)

/* robot orientation for stc algorithm
0 - down
1 - right
2 - up
3 - left */
int current_orientation;

double tmp_orientation_stc_step;        // target orientation of stc (orientation +- 90)
double tmp_pos_x_stc_step;              // tmp position x for moving by quarter
double tmp_pos_y_stc_step;              // tmp position y for moving by quarter

/* current quarter of the cell
0 - upper left
1 - lower left
2 - lower right
3 - upper right */
int current_quarter;

/* next step to be done by stc
0 - nothing
1 - rotate counter clockwise by 90deg
2 - rotate clockwise by 90deg
3 - drive forward by quarter */
int next_step;

/* target cell to move to 
0 - upper 
1 - left
2 - lower 
3 - right */
int target_cell;

int initialize_semaphores(void);

int init_battery_container(void);

int init_control(void);

int init_new_task(void);

int init_movement_type(void);

int calculate_control(void);

int select_new_direction(void);

int inspect_sensors(void);

int calculate_movement_type(void);

int inspect_battery_and_container(void);

int update_plan(int previous_x_pixel, int previous_y_pixel, int current_x_pix, int current_y_pix);

int check_nbh(void);

int init_stc(void);

int update_position_orientation(void);

int calc_next_step(void);

int select_target_cell(void);

int update_quarter_and_cell(void);

int parent_to_target(int);

#endif /* CONTROL_FUNCTIONS_H_*/