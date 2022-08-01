#ifndef CONTROL_FUNCTIONS_H_
#define CONTROL_FUNCTIONS_H_
#include "inttypes.h"
#include <unistd.h>
#include "astar/coordinate.h"
#include "astar/grid.uint8_t.h"
#include "astar/path.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846     // PI
#endif

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
_Atomic int movement_type;
sem_t movement_typeSemaphore;           // Semaphore

double target_direction;                // target direction to rotate to
sem_t target_directionSemaphore;        // Semaphore

sem_t spool_set_new_task;               // notification that new task has to be selected


/* STC ALGORITHM VARIABLES */
sem_t spool_calc_next_step_stc;         // notification for STC algorithm - calculate next step

int spool_next_step_calculated;         // flag that next step has been calculated

int starting_cell_x;                    // first cell 
int starting_cell_y;                    // first cell
int algorithm_finished;

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

/* BA* algorithm variables */
sem_t spool_calc_next_step_ba;

int spool_next_step_ba_calculated;

/* 0 - no data
   1 - not visited
   2 - visited
   3 - obstacle */
int ba_disc_map[80][80];

/* backtracking map:
   0 - no backtracking point
   1 - backtracking point */
int backtracking_map[80][80];

/* current task that is being executed
0 - just standing
1 - rotating clockwise to certain orientation
2 - rotating counter clockwise to certain orientation
3 - moving forward, primary boustrophedon
    movement (until reaching obstacle)
4 - moving forward, secondary boustrophedon
    movement (to a certain point)
*/
int current_task_ba;
double dist;
double prev_dist;
double target_orientation_ba;
double target_position_x_ba;
double target_position_y_ba;
int bt_list[100][2];
double bt_target_x;
double bt_target_y;
path_t path;
int path_index;

/* movement mode:
0 - simple B. movement
1 - navigating to next area */
int movement_mode;

/* type of previous movement during B. movement
0 - vertical (up or down)
1 - horizontal (left or right) */
int vertical_horizontal;

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

int init_ba(void);

int update_position_orientation(void);

int calc_next_step(void);

int select_target_cell(void);

int update_quarter_and_cell(void);

int calc_next_task(void);

int update_ba_map(void);

int create_bt_list(void);

int select_bt_point(void);

int parent_to_target(int);

void init_a_grid(grid_uint8_t* grid);

int is_point_traversable(const uint8_t* val);

double calculate_target_angle(double start_x, double start_y, double end_x, double end_y);

int rotation_direction(double target_angle, double current_angle);

path_t smooth_path(path_t* path);

#endif /* CONTROL_FUNCTIONS_H_*/