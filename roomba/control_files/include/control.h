#ifndef CONTROL_H_
#define CONTROL_H_

int algorithm_select;                   // algorithm selection
                                        // 0 - random bouncing
                                        // 1 - stc

/* Shared memory structure sim -> control */
struct s2c_shm {

    double front_sensor;        // front distance sensor
    double back_sensor;         // back distance sensor
    double left_sensor;         // left distance sensor
    double right_sensor;        // right distance sensor
    int trash_sensor;           // trash (below) sensor
    double battery_level;       // battery state [%]
    double container_level;     // container state [%]
    int new_trashes_coords[2];  // new trashes coords center (to reconstruct in control)
    double position_x;          // robot position cartesian (0-20m)
    double position_y;          // robot position cartesian (0-20m)
    double orientation;         // robot orientation (0-360deg)
};

// sim to control semaphores and pointer
int fd_shm_s2c;
struct s2c_shm *s2c_shm_ptr;
sem_t *mutex_sem_s2c, *spool_sem_s2c;

/* Shared memory structure control -> sim */
struct c2s_shm {
    double left_motor_power;    // left motor power (0-1)
    double right_motor_power;   // right motor power (0-1)
    double suction_power;       // suction/brushes power (0-1)
};

// control to sim semaphores and pointer
int fd_shm_c2s;
struct c2s_shm *c2s_shm_ptr;
sem_t *mutex_sem_c2s, *spool_sem_c2s;


#endif /* CONTROL_H_ */