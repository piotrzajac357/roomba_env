#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/wait.h>
#include <signal.h>

/* Main project function */
int main(int argc, char *argv[]) {

	

	pid_t pid;
	char* child_arg_control[3] = {"./control", NULL};
	char* child_arg_sim[3] = {"./sim", NULL};
	char* child_arg_vis[3] = {"./vis", NULL};
	int ret;

	pid = fork();

	if (pid == 0) {
		/* execute file with control process */
		execvp(child_arg_control[0], child_arg_control);
	}
	else {
		pid = fork();
		if (pid == 0) {
			/* execute file with simulation process */
			execvp(child_arg_sim[0], child_arg_sim);	
		}
		else {
			pid = fork();
			if (pid == 0) {
				/* execute file with visualisation process */
				execvp(child_arg_vis[0], child_arg_vis);
			}
			else {
				/* parent process */
				while(getc(stdin) == 'q') {
					printf("\nClosing all processes...\n");
					ret = kill(0, SIGTERM);
					
				};
			}
		}

	}

	return EXIT_SUCCESS;
}

