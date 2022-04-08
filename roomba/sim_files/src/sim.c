#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <semaphore.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "../include/sim.h"
#include "../include/position_update.h"
#include "../include/battery_update.h"
#include "../include/container_update.h"
#include "../include/dist_sensors_update.h"
#include "../include/trash_generator.h"
#include "../include/sim_functions.h"
#include "../include/sim_to_control.h"
#include "../include/sim_to_vis.h"
#include "../include/sim_from_control.h"

/* simulation process */
int main(int argc, char *argv[]) {

    int status;

    // shared memory initialization, sim -> reader
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
    if ((s2c_shm_ptr = mmap(NULL, sizeof (struct s2c_shm), PROT_READ | PROT_WRITE, MAP_SHARED, fd_shm_s2c, 0)) == MAP_FAILED) {
        fprintf(stderr, "Cannot map shared memory.\n");
		return 0;
    }

  
    // shared memory initialization, control -> reader
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
    if (ftruncate(fd_shm_c2s, sizeof(struct c2s_shm)) == -1) {
        fprintf(stderr, "Cannot truncate shared memory.\n");
		return 0;
    }
    if ((c2s_shm_ptr = mmap(NULL, sizeof(struct c2s_shm), PROT_READ | PROT_WRITE, MAP_SHARED, fd_shm_c2s, 0)) == MAP_FAILED) {
        fprintf(stderr, "Cannot map shared memory.\n");
		return 0;
    }
    // shared memory is ready
    sem_post(mutex_sem_c2s);

    // semaphores initialization
    if ((status = initialize_semaphores())) {
        fprintf(stderr, "Error initializing semaphores : %d\n", status);
        return 0;
    }

    // threads initialization
    if ((status = init_battery_update())) {
  	    fprintf(stderr, "Error initializing battery update thread : %d\n", status);
        return 0;
    }

    if ((status = init_position_update())) {
  	    fprintf(stderr, "Error initializing position update thread : %d\n", status);
        return 0;
    }

    if ((status = init_container_update())) {
  	    fprintf(stderr, "Error initializing container update thread : %d\n", status);
        return 0;
    }  

    if ((status = init_dist_sensors_update())) {
        fprintf(stderr, "Error initializing distance sensors update thread : %d\n", status);
        return 0;
    }

    /* do not generate trashes for now
    if ((status = init_trash_generator())) {
        fprintf(stderr, "Error initializing trash generator thread : %d\n", status);
    }
    */
   
    if ((status = init_sim_to_control())) {
        fprintf(stderr, "Error initializing sim_to_control data thread : %d\n", status);
        return 0;
    }

    if ((status = init_sim_to_vis())) {
        fprintf(stderr, "Error initializing sim_to_vis data thread : %d\n", status);
        return 0;
    }

    if ((status = init_sim_from_control())) {
        fprintf(stderr, "Error initializing sim_to_vis data thread : %d\n", status);
        return 0;
    }

    for(;;) {
        sleep(1);
    }
    
    // Unmap shared memory
    if (munmap(s2c_shm_ptr, sizeof(struct s2c_shm)) == -1){
        fprintf(stderr, "Cannot truncate shared memmory.\n");
        return 0;
    }
    // Close ipc socket
    close(my_socket);

    return EXIT_SUCCESS;
}