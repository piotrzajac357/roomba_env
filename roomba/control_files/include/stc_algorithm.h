#ifndef STC_ALGORITHM_H_
#define STC_ALGORITHM_H_

int algorithm_select;                   // algorithm selection
                                        // 0 - random bouncing
                                        // 1 - stc

int disc_plan[500][500];                // discovered plan of house
                                        // 0 - not checked (default)
                                        // 1 - not visited
                                        // 2 - visited
                                        // 3 - obstacle
int disc_plan2[40][40];

int parent_cells[40][40];               // parent cells of each cell
                                        // 0 - not calculated
                                        // 1 - from lower
                                        // 2 - from right
                                        // 3 - from upper
                                        // 4 - from left

double front_sensor;                    // front distance sensor
double back_sensor;                     // back distance sensor
double left_sensor;                     // left distance sensor
double right_sensor;                    // right distance sensor
sem_t dist_sensorsSemaphore;            // Semaphore

int current_position_x;                 // robot position x
int current_position_y;                 // robot position y
int current_orientation;                // robot orientation
                                        // 0-down
                                        // 1-right
                                        // 2-up
                                        // 3-left
double starting_x;
double starting_y;

double tmp_orientation_stc_step;        // tmp orientation for rotating by 90deg task
double tmp_pos_x_stc_step;
double tmp_pos_y_stc_step;

int current_quarter;                    // current quarter of the tile
                                        // 0 - upper left
                                        // 1 - lower left
                                        // 2 - lower right
                                        // 3 - upper right

int next_step;                          // what is to be done at the moment
                                        // 0 - nothing
                                        // 1 - rotate counter clockwise
                                        // 2 - rotate clockwise
                                        // 3 - drive forward

int target_cell;                        // 0 - upper
                                        // 1 - left
                                        // 2 - lower
                                        // 3 - right
int check_nbh(void);

int init_stc(void);

int update_position_orientation();

int calc_next_step();

int select_target_cell();

int init_stc_algorithm();

int update_quarter_and_cell();

int parent_to_target(int);

void *tStcThreadFunc(void *cookie);

#endif /* STC_ALGORITHM_H_*/