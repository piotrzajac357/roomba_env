add_executable(roomba roomba.c)

target_link_libraries(roomba -pthread rt m)

include_directories(sim sim_files/include)

add_executable(sim  sim_files/src/sim.c
                    sim_files/src/load_plan.c
                    sim_files/src/position_update.c 
                    sim_files/src/battery_update.c
                    sim_files/src/dist_sensors_update.c
                    sim_files/src/quality_indexes.c
                    sim_files/src/sim_functions.c
                    sim_files/src/sim_to_control.c
                    sim_files/src/sim_to_vis.c
                    sim_files/src/sim_from_control.c)
                    

target_link_libraries(sim pthread rt m)

include_directories(control control_files/include)

add_executable(control  control_files/src/control
                        control_files/src/control_battery.c
                        control_files/src/control_from_sim.c
                        control_files/src/control_functions.c
                        control_files/src/control_to_sim.c
                        control_files/src/control_update.c
                        control_files/src/rg_algorithm.c
                        control_files/src/task_to_movement.c
                        control_files/src/stc_algorithm.c
                        control_files/src/ba_algorithm.c
                        control_files/src/swf_algorithm.c)




find_library(PATHE_LIB libpath.so)
#target_link_libraries(control PRIVATE )
target_link_libraries(control pthread rt m "${PATHE_LIB}")

##find_package (PythonLibs 3.6 REQUIRED)
add_executable(vis  vis_files/src/vis.c
                    vis_files/vis.py
                    vis_files/src/run_python_script.c)


target_link_libraries(vis pthread rt m python3.6m)
