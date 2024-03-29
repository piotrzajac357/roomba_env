#ifndef SIM_H_
#define SIM_H_


/* Shared memory structure sim -> control */
struct s2c_shm {

    double front_sensor;        // front distance sensor
    double back_sensor;         // back distance sensor
    double left_sensor;         // left distance sensor
    double right_sensor;        // right distance sensor
    double battery_level;       // battery state [%]
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
    int algorithm_finished;
};

// control to sim semaphores and pointer
int fd_shm_c2s;
struct c2s_shm *c2s_shm_ptr;
sem_t *mutex_sem_c2s, *spool_sem_c2s;

#endif /* SIM_H_ */