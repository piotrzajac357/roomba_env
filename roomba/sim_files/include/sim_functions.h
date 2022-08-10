#ifndef SIM_FUNCTIONS_H_
#define SIM_FUNCTIONS_H_

#ifndef M_PI
#define M_PI 3.14159265358979323846     // PI
#endif

#ifndef ST_TO_RAD
#define ST_TO_RAD 0.01745329251         // Degrees to radian convertion multiplier
#endif

double position_x;                      // Current cartesian position x coord (0-20m)          
double position_y;                      // Current cartesian position y coord (0-20m)
double previous_orientation;            // Current orientation (0-360deg)
sem_t position_orientationSemaphore;    // Semaphore

double left_motor_power;                // Left motor power (0-1)
double right_motor_power;               // Right motor power (0-1)
double suction_power;                   // Suction/brushes power (0-1)
sem_t controlSemaphore;                 // Semaphore

double battery_level;                   // Battery state (0-100%)
sem_t batterySemaphore;                 // Semaphore

double container_level;                 // Container state (0-100%)
sem_t containerSemaphore;               // Semaphore

double front_sensor;                    // front distance sensor
double back_sensor;                     // back distance sensor
double left_sensor;                     // left distance sensor
double right_sensor;                    // right distance sensor
int trash_sensor;                       // trash (below) sensor
sem_t dist_sensorsSemaphore;            // Semaphore

char plan[200][200];                    // 200x200 char array representing house 
sem_t planSemaphore;                    // Semaphores

// Read only variables (after being once set)
double sim_step_position;               // Simulation step for calculating position (0-1sec)
int sim_step_battery;                   // Simulation step for calculating battery state (sec)
int sim_step_container;                 // Simulation step for calculating container state (sec)
double sim_step_dist_sensors;           // Simulation step for calculating sensors (0-1sec)
double max_speed;                       // Linear speed of robot. Increase this to fasten simulation.
double max_rotating_speed;              // Rotating speed of robot. Increase this to fasten simulation.

// quality indexes
double time_QI;                         // time quality index
double path_QI;
double path_qi_prev_x;
double path_qi_prev_y;
double rotation_QI;
double rotation_qi_prev;
double coverage_QI;
int coverage_plan[800][800];
int coverage_qi_total;
int coverage_acc;
double idle_time_QI;

int algorithm_finished;

sem_t qiSemaphore;                      // Semaphore


int initialize_semaphores(void);

int initialize_position(void);

int initialize_battery(void);

int initialize_container(void);

int initialize_dist_sensors(void);

int initialize_control_values(void);

int initialize_quality_indexes(void);

int calculate_position(void);

int calculate_battery(void);

int calculate_container(void);

int calculate_sensors(void);

int calculate_qis(double);

#endif  /* SIM_FUNCTIONS_H_ */