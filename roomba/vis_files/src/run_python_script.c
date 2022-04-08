#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <python3.6/Python.h>

#include "../include/run_python_script.h"


/* Thread runs python visualisation script with infinite loop */
int init_visualisation() {

    int status;
    pthread_t VisThread;
    pthread_attr_t aVisThreadAttr;
    pthread_attr_init(&aVisThreadAttr);
    pthread_attr_setschedpolicy(&aVisThreadAttr, SCHED_FIFO);

    if ((status = pthread_create( &VisThread, &aVisThreadAttr, tVisThreadFunc, NULL))) {
        fprintf(stderr, "Cannot create visualisation thread.\n");
    return 0;
	}

    return EXIT_SUCCESS;
}

void *tVisThreadFunc(void *cookie) {

    int policy = SCHED_FIFO;
    struct sched_param param;
    pthread_getschedparam(pthread_self(), &policy, &param);

    // Visualisation thread has the lowest possible priority
	param.sched_priority = sched_get_priority_min(policy);
	pthread_setschedparam(pthread_self(), policy, &param);

    // python interpreter initialization and launching script
    Py_Initialize();
    FILE *file = fopen("../../roomba/vis_files/vis.py", "r");
    if(file != NULL) {
        PyRun_SimpleFile(file, "../../roomba/vis_files/vis.py");
    }

    return EXIT_SUCCESS;
}