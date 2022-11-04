# roomba_env

Repository contains source files and executables of roomba_env. roomba_env is a soft real-time simulation environment of house cleaning robot.
Roomba_env allows to implement and test various algorithms of house cleaning robots. It contains visualization and quality indexes system.
Code is written in C, application works on ubuntu 18.04 with preempt_rt kernel patch.

Application is divided into severeal processes and threads making it possible to run concurrently.

Several algorithms controlling robot has been implemented (STC, BA*, RG, DWA*) and tested as a part of master thesis. 
