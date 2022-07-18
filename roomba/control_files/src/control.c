#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#include <semaphore.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

#include "../include/control.h"
#include "../include/control_functions.h"
#include "../include/control_to_sim.h"
#include "../include/control_from_sim.h"
#include "../include/control_update.h"
#include "../include/set_new_task.h"
#include "../include/load_plan.h"
#include "../include/sensors_inspect.h"
#include "../include/task_to_movement.h"
#include "../include/control_battery_container.h"
#include "../include/stc_algorithm.h"

int main(int argc, char *argv[]) {

    int status;
    algorithm_select = 1;

    //nice(-20);

    // shared memory initialization, sim - writer, control - reader
    if ((mutex_sem_s2c = sem_open("/sem-mutex_s2c", O_CREAT, 0660, 0)) == SEM_FAILED) {
        fprintf(stderr, "Cannot create semaphore.\n");
		return 0;
    }
    if ((spool_sem_s2c = sem_open("/sem-spool_s2c", O_CREAT, 0660, 0)) == SEM_FAILED) {
        fprintf(stderr, "Cannot create semaphore.\n");
		return 0;
    }
    if ((fd_shm_s2c = shm_open("/my-shared-mem_s2c", O_RDWR | O_CREAT, 0660)) == -1) {
        fprintf(stderr, "Cannot create shared memory.\n");
		return 0;
    }
    if (ftruncate(fd_shm_s2c, sizeof(struct s2c_shm)) == -1) {
        fprintf(stderr, "Cannot truncate shared memory.\n");
		return 0;
    }
    if ((s2c_shm_ptr = mmap(NULL, sizeof(struct s2c_shm), PROT_READ | PROT_WRITE, MAP_SHARED, fd_shm_s2c, 0)) == MAP_FAILED) {
        fprintf(stderr, "Cannot map shared memory.\n");
		return 0;
    }
    sem_post(mutex_sem_s2c);
    
    // shared memory initialization, control - writer, sim - reader
    if ((mutex_sem_c2s = sem_open("/sem-mutex_c2s", O_CREAT, 0660, 0)) == SEM_FAILED) {
        fprintf(stderr, "Cannot create semaphore.\n");
		return 0;
    }
    if ((spool_sem_c2s = sem_open("/sem-spool_c2s", O_CREAT, 0660, 0)) == SEM_FAILED) {
        fprintf(stderr, "Cannot create semaphore.\n");
		return 0;
    }
    if ((fd_shm_c2s = shm_open("/my-shared-mem_c2s", O_RDWR | O_CREAT, 0660)) == -1) {
        fprintf(stderr, "Cannot create shared memory.\n");
		return 0;
    }
    if ((c2s_shm_ptr = mmap(NULL, sizeof (struct c2s_shm), PROT_READ | PROT_WRITE, MAP_SHARED, fd_shm_c2s, 0)) == MAP_FAILED) {
        fprintf(stderr, "Cannot map shared memory.\n");
		return 0;
    }

    // semaphores initialization
    if ((status = initialize_semaphores())) {
        fprintf(stderr, "Error initializing semaphores : %d\n", status);
        return 0;
    }

    // threads initialization
    if ((status = init_control_to_sim())) {
        fprintf(stderr, "Error initializing control to simulation data thread : %d\n", status);
        return 0;
    }

    if ((status = init_control_from_sim())) {
        fprintf(stderr, "Error initializing control from simulation data thread : %d\n", status);
        return 0;
    }

    if ((status = init_control_update())) {
        fprintf(stderr, "Error initializing control update thread : %d\n", status);
        return 0;
    }

    // for random bouncing
    // if ((status = init_set_new_task())) {
    //     fprintf(stderr, "Error initializing new task setting thread : %d\n", status);
    //     return 0;
    // }

    // for random bouncing
    // if ((status = init_sensors_inspect())) {
    //     fprintf(stderr, "Error initializing sensors inspecting thread : %d\n", status);
    //     return 0;
    // }

    if ((status = init_task_to_movement())) {
        fprintf(stderr, "Error initializing task to movement converting thread : %d\n", status);
        return 0;
    }

    // for random bouncing
    // if ((status = init_battery_container_inspect())) {
    //     fprintf(stderr, "Error initializing container and battery watching thread : %d\n", status);
    //     return 0;
    // }

    if (algorithm_select == 1){
        if ((status = init_stc_algorithm())) {
            fprintf(stderr, "Error initializing STC algorithm thread : %d\n", status);
            return 0;
        }
    }
    while (1){        
        usleep(100000);
    }

    // Unmap shared memory
    if (munmap(c2s_shm_ptr, sizeof (struct c2s_shm)) == -1) {
        fprintf(stderr, "Cannot truncate shared memmory.\n");
	return 0;
    }

    return EXIT_SUCCESS;
}
