#include <stdio.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <unistd.h>

#include "../include/run_python_script.h"


/* main function of visualisation process */
int main(int argc, char** argv) {

    //nice(20);

    int status;
    // init visualisation thread
    if ((status = init_visualisation())) {
        fprintf(stderr, "Error initializing control to simulation data thread : %d\n", status);
    return 0;
    }

    for(;;) {
        sleep(1);
    }

    return EXIT_SUCCESS;
}