#include <stdio.h>
#include <stdlib.h>

#include "../include/load_plan.h"

/* Open file with house plan and load it into an array in process global memory */
void init_plan(){

    FILE *fptr;
    char c;
    fptr = fopen("../../roomba/test_plans/plan_showcase_stc_ba.txt","r");
    if(fptr == NULL) {
        perror("Error in opening file");
        return;
    }
    for(int i = 0; i < 200; i++) {
        for(int j = 0; j < 201; j++) {
            c = fgetc(fptr);
            if (!(j == 200)) {
                // inverted y coord, higher index - smaller coord
                plan[199-i][j] = c;
            }
        }
    }
    fclose(fptr);
    return;
}
