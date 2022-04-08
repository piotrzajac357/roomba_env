#ifndef CONTROL_FUNCTIONS_H_
#define CONTROL_FUNCTIONS_H_

#ifndef ST_TO_RAD
#define ST_TO_RAD 0.01745329251         // degrees to radians convert multiplier
#endif

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

#endif /* CONTROL_FUNCTIONS_H_*/