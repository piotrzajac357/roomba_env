# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/piotr/Desktop/pm_project/roomba_env

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/piotr/Desktop/pm_project/roomba_env/build

# Include any dependencies generated for this target.
include roomba/CMakeFiles/sim.dir/depend.make

# Include the progress variables for this target.
include roomba/CMakeFiles/sim.dir/progress.make

# Include the compile flags for this target's objects.
include roomba/CMakeFiles/sim.dir/flags.make

roomba/CMakeFiles/sim.dir/sim_files/src/sim.c.o: roomba/CMakeFiles/sim.dir/flags.make
roomba/CMakeFiles/sim.dir/sim_files/src/sim.c.o: ../roomba/sim_files/src/sim.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/piotr/Desktop/pm_project/roomba_env/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object roomba/CMakeFiles/sim.dir/sim_files/src/sim.c.o"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/sim.dir/sim_files/src/sim.c.o   -c /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/sim.c

roomba/CMakeFiles/sim.dir/sim_files/src/sim.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/sim.dir/sim_files/src/sim.c.i"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/sim.c > CMakeFiles/sim.dir/sim_files/src/sim.c.i

roomba/CMakeFiles/sim.dir/sim_files/src/sim.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/sim.dir/sim_files/src/sim.c.s"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/sim.c -o CMakeFiles/sim.dir/sim_files/src/sim.c.s

roomba/CMakeFiles/sim.dir/sim_files/src/sim.c.o.requires:

.PHONY : roomba/CMakeFiles/sim.dir/sim_files/src/sim.c.o.requires

roomba/CMakeFiles/sim.dir/sim_files/src/sim.c.o.provides: roomba/CMakeFiles/sim.dir/sim_files/src/sim.c.o.requires
	$(MAKE) -f roomba/CMakeFiles/sim.dir/build.make roomba/CMakeFiles/sim.dir/sim_files/src/sim.c.o.provides.build
.PHONY : roomba/CMakeFiles/sim.dir/sim_files/src/sim.c.o.provides

roomba/CMakeFiles/sim.dir/sim_files/src/sim.c.o.provides.build: roomba/CMakeFiles/sim.dir/sim_files/src/sim.c.o


roomba/CMakeFiles/sim.dir/sim_files/src/load_plan.c.o: roomba/CMakeFiles/sim.dir/flags.make
roomba/CMakeFiles/sim.dir/sim_files/src/load_plan.c.o: ../roomba/sim_files/src/load_plan.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/piotr/Desktop/pm_project/roomba_env/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object roomba/CMakeFiles/sim.dir/sim_files/src/load_plan.c.o"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/sim.dir/sim_files/src/load_plan.c.o   -c /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/load_plan.c

roomba/CMakeFiles/sim.dir/sim_files/src/load_plan.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/sim.dir/sim_files/src/load_plan.c.i"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/load_plan.c > CMakeFiles/sim.dir/sim_files/src/load_plan.c.i

roomba/CMakeFiles/sim.dir/sim_files/src/load_plan.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/sim.dir/sim_files/src/load_plan.c.s"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/load_plan.c -o CMakeFiles/sim.dir/sim_files/src/load_plan.c.s

roomba/CMakeFiles/sim.dir/sim_files/src/load_plan.c.o.requires:

.PHONY : roomba/CMakeFiles/sim.dir/sim_files/src/load_plan.c.o.requires

roomba/CMakeFiles/sim.dir/sim_files/src/load_plan.c.o.provides: roomba/CMakeFiles/sim.dir/sim_files/src/load_plan.c.o.requires
	$(MAKE) -f roomba/CMakeFiles/sim.dir/build.make roomba/CMakeFiles/sim.dir/sim_files/src/load_plan.c.o.provides.build
.PHONY : roomba/CMakeFiles/sim.dir/sim_files/src/load_plan.c.o.provides

roomba/CMakeFiles/sim.dir/sim_files/src/load_plan.c.o.provides.build: roomba/CMakeFiles/sim.dir/sim_files/src/load_plan.c.o


roomba/CMakeFiles/sim.dir/sim_files/src/position_update.c.o: roomba/CMakeFiles/sim.dir/flags.make
roomba/CMakeFiles/sim.dir/sim_files/src/position_update.c.o: ../roomba/sim_files/src/position_update.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/piotr/Desktop/pm_project/roomba_env/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object roomba/CMakeFiles/sim.dir/sim_files/src/position_update.c.o"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/sim.dir/sim_files/src/position_update.c.o   -c /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/position_update.c

roomba/CMakeFiles/sim.dir/sim_files/src/position_update.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/sim.dir/sim_files/src/position_update.c.i"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/position_update.c > CMakeFiles/sim.dir/sim_files/src/position_update.c.i

roomba/CMakeFiles/sim.dir/sim_files/src/position_update.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/sim.dir/sim_files/src/position_update.c.s"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/position_update.c -o CMakeFiles/sim.dir/sim_files/src/position_update.c.s

roomba/CMakeFiles/sim.dir/sim_files/src/position_update.c.o.requires:

.PHONY : roomba/CMakeFiles/sim.dir/sim_files/src/position_update.c.o.requires

roomba/CMakeFiles/sim.dir/sim_files/src/position_update.c.o.provides: roomba/CMakeFiles/sim.dir/sim_files/src/position_update.c.o.requires
	$(MAKE) -f roomba/CMakeFiles/sim.dir/build.make roomba/CMakeFiles/sim.dir/sim_files/src/position_update.c.o.provides.build
.PHONY : roomba/CMakeFiles/sim.dir/sim_files/src/position_update.c.o.provides

roomba/CMakeFiles/sim.dir/sim_files/src/position_update.c.o.provides.build: roomba/CMakeFiles/sim.dir/sim_files/src/position_update.c.o


roomba/CMakeFiles/sim.dir/sim_files/src/battery_update.c.o: roomba/CMakeFiles/sim.dir/flags.make
roomba/CMakeFiles/sim.dir/sim_files/src/battery_update.c.o: ../roomba/sim_files/src/battery_update.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/piotr/Desktop/pm_project/roomba_env/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object roomba/CMakeFiles/sim.dir/sim_files/src/battery_update.c.o"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/sim.dir/sim_files/src/battery_update.c.o   -c /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/battery_update.c

roomba/CMakeFiles/sim.dir/sim_files/src/battery_update.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/sim.dir/sim_files/src/battery_update.c.i"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/battery_update.c > CMakeFiles/sim.dir/sim_files/src/battery_update.c.i

roomba/CMakeFiles/sim.dir/sim_files/src/battery_update.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/sim.dir/sim_files/src/battery_update.c.s"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/battery_update.c -o CMakeFiles/sim.dir/sim_files/src/battery_update.c.s

roomba/CMakeFiles/sim.dir/sim_files/src/battery_update.c.o.requires:

.PHONY : roomba/CMakeFiles/sim.dir/sim_files/src/battery_update.c.o.requires

roomba/CMakeFiles/sim.dir/sim_files/src/battery_update.c.o.provides: roomba/CMakeFiles/sim.dir/sim_files/src/battery_update.c.o.requires
	$(MAKE) -f roomba/CMakeFiles/sim.dir/build.make roomba/CMakeFiles/sim.dir/sim_files/src/battery_update.c.o.provides.build
.PHONY : roomba/CMakeFiles/sim.dir/sim_files/src/battery_update.c.o.provides

roomba/CMakeFiles/sim.dir/sim_files/src/battery_update.c.o.provides.build: roomba/CMakeFiles/sim.dir/sim_files/src/battery_update.c.o


roomba/CMakeFiles/sim.dir/sim_files/src/container_update.c.o: roomba/CMakeFiles/sim.dir/flags.make
roomba/CMakeFiles/sim.dir/sim_files/src/container_update.c.o: ../roomba/sim_files/src/container_update.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/piotr/Desktop/pm_project/roomba_env/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object roomba/CMakeFiles/sim.dir/sim_files/src/container_update.c.o"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/sim.dir/sim_files/src/container_update.c.o   -c /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/container_update.c

roomba/CMakeFiles/sim.dir/sim_files/src/container_update.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/sim.dir/sim_files/src/container_update.c.i"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/container_update.c > CMakeFiles/sim.dir/sim_files/src/container_update.c.i

roomba/CMakeFiles/sim.dir/sim_files/src/container_update.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/sim.dir/sim_files/src/container_update.c.s"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/container_update.c -o CMakeFiles/sim.dir/sim_files/src/container_update.c.s

roomba/CMakeFiles/sim.dir/sim_files/src/container_update.c.o.requires:

.PHONY : roomba/CMakeFiles/sim.dir/sim_files/src/container_update.c.o.requires

roomba/CMakeFiles/sim.dir/sim_files/src/container_update.c.o.provides: roomba/CMakeFiles/sim.dir/sim_files/src/container_update.c.o.requires
	$(MAKE) -f roomba/CMakeFiles/sim.dir/build.make roomba/CMakeFiles/sim.dir/sim_files/src/container_update.c.o.provides.build
.PHONY : roomba/CMakeFiles/sim.dir/sim_files/src/container_update.c.o.provides

roomba/CMakeFiles/sim.dir/sim_files/src/container_update.c.o.provides.build: roomba/CMakeFiles/sim.dir/sim_files/src/container_update.c.o


roomba/CMakeFiles/sim.dir/sim_files/src/dist_sensors_update.c.o: roomba/CMakeFiles/sim.dir/flags.make
roomba/CMakeFiles/sim.dir/sim_files/src/dist_sensors_update.c.o: ../roomba/sim_files/src/dist_sensors_update.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/piotr/Desktop/pm_project/roomba_env/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object roomba/CMakeFiles/sim.dir/sim_files/src/dist_sensors_update.c.o"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/sim.dir/sim_files/src/dist_sensors_update.c.o   -c /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/dist_sensors_update.c

roomba/CMakeFiles/sim.dir/sim_files/src/dist_sensors_update.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/sim.dir/sim_files/src/dist_sensors_update.c.i"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/dist_sensors_update.c > CMakeFiles/sim.dir/sim_files/src/dist_sensors_update.c.i

roomba/CMakeFiles/sim.dir/sim_files/src/dist_sensors_update.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/sim.dir/sim_files/src/dist_sensors_update.c.s"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/dist_sensors_update.c -o CMakeFiles/sim.dir/sim_files/src/dist_sensors_update.c.s

roomba/CMakeFiles/sim.dir/sim_files/src/dist_sensors_update.c.o.requires:

.PHONY : roomba/CMakeFiles/sim.dir/sim_files/src/dist_sensors_update.c.o.requires

roomba/CMakeFiles/sim.dir/sim_files/src/dist_sensors_update.c.o.provides: roomba/CMakeFiles/sim.dir/sim_files/src/dist_sensors_update.c.o.requires
	$(MAKE) -f roomba/CMakeFiles/sim.dir/build.make roomba/CMakeFiles/sim.dir/sim_files/src/dist_sensors_update.c.o.provides.build
.PHONY : roomba/CMakeFiles/sim.dir/sim_files/src/dist_sensors_update.c.o.provides

roomba/CMakeFiles/sim.dir/sim_files/src/dist_sensors_update.c.o.provides.build: roomba/CMakeFiles/sim.dir/sim_files/src/dist_sensors_update.c.o


roomba/CMakeFiles/sim.dir/sim_files/src/trash_generator.c.o: roomba/CMakeFiles/sim.dir/flags.make
roomba/CMakeFiles/sim.dir/sim_files/src/trash_generator.c.o: ../roomba/sim_files/src/trash_generator.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/piotr/Desktop/pm_project/roomba_env/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building C object roomba/CMakeFiles/sim.dir/sim_files/src/trash_generator.c.o"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/sim.dir/sim_files/src/trash_generator.c.o   -c /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/trash_generator.c

roomba/CMakeFiles/sim.dir/sim_files/src/trash_generator.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/sim.dir/sim_files/src/trash_generator.c.i"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/trash_generator.c > CMakeFiles/sim.dir/sim_files/src/trash_generator.c.i

roomba/CMakeFiles/sim.dir/sim_files/src/trash_generator.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/sim.dir/sim_files/src/trash_generator.c.s"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/trash_generator.c -o CMakeFiles/sim.dir/sim_files/src/trash_generator.c.s

roomba/CMakeFiles/sim.dir/sim_files/src/trash_generator.c.o.requires:

.PHONY : roomba/CMakeFiles/sim.dir/sim_files/src/trash_generator.c.o.requires

roomba/CMakeFiles/sim.dir/sim_files/src/trash_generator.c.o.provides: roomba/CMakeFiles/sim.dir/sim_files/src/trash_generator.c.o.requires
	$(MAKE) -f roomba/CMakeFiles/sim.dir/build.make roomba/CMakeFiles/sim.dir/sim_files/src/trash_generator.c.o.provides.build
.PHONY : roomba/CMakeFiles/sim.dir/sim_files/src/trash_generator.c.o.provides

roomba/CMakeFiles/sim.dir/sim_files/src/trash_generator.c.o.provides.build: roomba/CMakeFiles/sim.dir/sim_files/src/trash_generator.c.o


roomba/CMakeFiles/sim.dir/sim_files/src/quality_indexes.c.o: roomba/CMakeFiles/sim.dir/flags.make
roomba/CMakeFiles/sim.dir/sim_files/src/quality_indexes.c.o: ../roomba/sim_files/src/quality_indexes.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/piotr/Desktop/pm_project/roomba_env/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building C object roomba/CMakeFiles/sim.dir/sim_files/src/quality_indexes.c.o"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/sim.dir/sim_files/src/quality_indexes.c.o   -c /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/quality_indexes.c

roomba/CMakeFiles/sim.dir/sim_files/src/quality_indexes.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/sim.dir/sim_files/src/quality_indexes.c.i"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/quality_indexes.c > CMakeFiles/sim.dir/sim_files/src/quality_indexes.c.i

roomba/CMakeFiles/sim.dir/sim_files/src/quality_indexes.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/sim.dir/sim_files/src/quality_indexes.c.s"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/quality_indexes.c -o CMakeFiles/sim.dir/sim_files/src/quality_indexes.c.s

roomba/CMakeFiles/sim.dir/sim_files/src/quality_indexes.c.o.requires:

.PHONY : roomba/CMakeFiles/sim.dir/sim_files/src/quality_indexes.c.o.requires

roomba/CMakeFiles/sim.dir/sim_files/src/quality_indexes.c.o.provides: roomba/CMakeFiles/sim.dir/sim_files/src/quality_indexes.c.o.requires
	$(MAKE) -f roomba/CMakeFiles/sim.dir/build.make roomba/CMakeFiles/sim.dir/sim_files/src/quality_indexes.c.o.provides.build
.PHONY : roomba/CMakeFiles/sim.dir/sim_files/src/quality_indexes.c.o.provides

roomba/CMakeFiles/sim.dir/sim_files/src/quality_indexes.c.o.provides.build: roomba/CMakeFiles/sim.dir/sim_files/src/quality_indexes.c.o


roomba/CMakeFiles/sim.dir/sim_files/src/sim_functions.c.o: roomba/CMakeFiles/sim.dir/flags.make
roomba/CMakeFiles/sim.dir/sim_files/src/sim_functions.c.o: ../roomba/sim_files/src/sim_functions.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/piotr/Desktop/pm_project/roomba_env/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building C object roomba/CMakeFiles/sim.dir/sim_files/src/sim_functions.c.o"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/sim.dir/sim_files/src/sim_functions.c.o   -c /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/sim_functions.c

roomba/CMakeFiles/sim.dir/sim_files/src/sim_functions.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/sim.dir/sim_files/src/sim_functions.c.i"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/sim_functions.c > CMakeFiles/sim.dir/sim_files/src/sim_functions.c.i

roomba/CMakeFiles/sim.dir/sim_files/src/sim_functions.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/sim.dir/sim_files/src/sim_functions.c.s"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/sim_functions.c -o CMakeFiles/sim.dir/sim_files/src/sim_functions.c.s

roomba/CMakeFiles/sim.dir/sim_files/src/sim_functions.c.o.requires:

.PHONY : roomba/CMakeFiles/sim.dir/sim_files/src/sim_functions.c.o.requires

roomba/CMakeFiles/sim.dir/sim_files/src/sim_functions.c.o.provides: roomba/CMakeFiles/sim.dir/sim_files/src/sim_functions.c.o.requires
	$(MAKE) -f roomba/CMakeFiles/sim.dir/build.make roomba/CMakeFiles/sim.dir/sim_files/src/sim_functions.c.o.provides.build
.PHONY : roomba/CMakeFiles/sim.dir/sim_files/src/sim_functions.c.o.provides

roomba/CMakeFiles/sim.dir/sim_files/src/sim_functions.c.o.provides.build: roomba/CMakeFiles/sim.dir/sim_files/src/sim_functions.c.o


roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_control.c.o: roomba/CMakeFiles/sim.dir/flags.make
roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_control.c.o: ../roomba/sim_files/src/sim_to_control.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/piotr/Desktop/pm_project/roomba_env/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building C object roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_control.c.o"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/sim.dir/sim_files/src/sim_to_control.c.o   -c /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/sim_to_control.c

roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_control.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/sim.dir/sim_files/src/sim_to_control.c.i"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/sim_to_control.c > CMakeFiles/sim.dir/sim_files/src/sim_to_control.c.i

roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_control.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/sim.dir/sim_files/src/sim_to_control.c.s"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/sim_to_control.c -o CMakeFiles/sim.dir/sim_files/src/sim_to_control.c.s

roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_control.c.o.requires:

.PHONY : roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_control.c.o.requires

roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_control.c.o.provides: roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_control.c.o.requires
	$(MAKE) -f roomba/CMakeFiles/sim.dir/build.make roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_control.c.o.provides.build
.PHONY : roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_control.c.o.provides

roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_control.c.o.provides.build: roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_control.c.o


roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_vis.c.o: roomba/CMakeFiles/sim.dir/flags.make
roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_vis.c.o: ../roomba/sim_files/src/sim_to_vis.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/piotr/Desktop/pm_project/roomba_env/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building C object roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_vis.c.o"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/sim.dir/sim_files/src/sim_to_vis.c.o   -c /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/sim_to_vis.c

roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_vis.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/sim.dir/sim_files/src/sim_to_vis.c.i"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/sim_to_vis.c > CMakeFiles/sim.dir/sim_files/src/sim_to_vis.c.i

roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_vis.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/sim.dir/sim_files/src/sim_to_vis.c.s"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/sim_to_vis.c -o CMakeFiles/sim.dir/sim_files/src/sim_to_vis.c.s

roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_vis.c.o.requires:

.PHONY : roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_vis.c.o.requires

roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_vis.c.o.provides: roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_vis.c.o.requires
	$(MAKE) -f roomba/CMakeFiles/sim.dir/build.make roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_vis.c.o.provides.build
.PHONY : roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_vis.c.o.provides

roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_vis.c.o.provides.build: roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_vis.c.o


roomba/CMakeFiles/sim.dir/sim_files/src/sim_from_control.c.o: roomba/CMakeFiles/sim.dir/flags.make
roomba/CMakeFiles/sim.dir/sim_files/src/sim_from_control.c.o: ../roomba/sim_files/src/sim_from_control.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/piotr/Desktop/pm_project/roomba_env/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building C object roomba/CMakeFiles/sim.dir/sim_files/src/sim_from_control.c.o"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/sim.dir/sim_files/src/sim_from_control.c.o   -c /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/sim_from_control.c

roomba/CMakeFiles/sim.dir/sim_files/src/sim_from_control.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/sim.dir/sim_files/src/sim_from_control.c.i"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/sim_from_control.c > CMakeFiles/sim.dir/sim_files/src/sim_from_control.c.i

roomba/CMakeFiles/sim.dir/sim_files/src/sim_from_control.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/sim.dir/sim_files/src/sim_from_control.c.s"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/piotr/Desktop/pm_project/roomba_env/roomba/sim_files/src/sim_from_control.c -o CMakeFiles/sim.dir/sim_files/src/sim_from_control.c.s

roomba/CMakeFiles/sim.dir/sim_files/src/sim_from_control.c.o.requires:

.PHONY : roomba/CMakeFiles/sim.dir/sim_files/src/sim_from_control.c.o.requires

roomba/CMakeFiles/sim.dir/sim_files/src/sim_from_control.c.o.provides: roomba/CMakeFiles/sim.dir/sim_files/src/sim_from_control.c.o.requires
	$(MAKE) -f roomba/CMakeFiles/sim.dir/build.make roomba/CMakeFiles/sim.dir/sim_files/src/sim_from_control.c.o.provides.build
.PHONY : roomba/CMakeFiles/sim.dir/sim_files/src/sim_from_control.c.o.provides

roomba/CMakeFiles/sim.dir/sim_files/src/sim_from_control.c.o.provides.build: roomba/CMakeFiles/sim.dir/sim_files/src/sim_from_control.c.o


# Object files for target sim
sim_OBJECTS = \
"CMakeFiles/sim.dir/sim_files/src/sim.c.o" \
"CMakeFiles/sim.dir/sim_files/src/load_plan.c.o" \
"CMakeFiles/sim.dir/sim_files/src/position_update.c.o" \
"CMakeFiles/sim.dir/sim_files/src/battery_update.c.o" \
"CMakeFiles/sim.dir/sim_files/src/container_update.c.o" \
"CMakeFiles/sim.dir/sim_files/src/dist_sensors_update.c.o" \
"CMakeFiles/sim.dir/sim_files/src/trash_generator.c.o" \
"CMakeFiles/sim.dir/sim_files/src/quality_indexes.c.o" \
"CMakeFiles/sim.dir/sim_files/src/sim_functions.c.o" \
"CMakeFiles/sim.dir/sim_files/src/sim_to_control.c.o" \
"CMakeFiles/sim.dir/sim_files/src/sim_to_vis.c.o" \
"CMakeFiles/sim.dir/sim_files/src/sim_from_control.c.o"

# External object files for target sim
sim_EXTERNAL_OBJECTS =

roomba/sim: roomba/CMakeFiles/sim.dir/sim_files/src/sim.c.o
roomba/sim: roomba/CMakeFiles/sim.dir/sim_files/src/load_plan.c.o
roomba/sim: roomba/CMakeFiles/sim.dir/sim_files/src/position_update.c.o
roomba/sim: roomba/CMakeFiles/sim.dir/sim_files/src/battery_update.c.o
roomba/sim: roomba/CMakeFiles/sim.dir/sim_files/src/container_update.c.o
roomba/sim: roomba/CMakeFiles/sim.dir/sim_files/src/dist_sensors_update.c.o
roomba/sim: roomba/CMakeFiles/sim.dir/sim_files/src/trash_generator.c.o
roomba/sim: roomba/CMakeFiles/sim.dir/sim_files/src/quality_indexes.c.o
roomba/sim: roomba/CMakeFiles/sim.dir/sim_files/src/sim_functions.c.o
roomba/sim: roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_control.c.o
roomba/sim: roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_vis.c.o
roomba/sim: roomba/CMakeFiles/sim.dir/sim_files/src/sim_from_control.c.o
roomba/sim: roomba/CMakeFiles/sim.dir/build.make
roomba/sim: roomba/CMakeFiles/sim.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/piotr/Desktop/pm_project/roomba_env/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Linking C executable sim"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sim.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
roomba/CMakeFiles/sim.dir/build: roomba/sim

.PHONY : roomba/CMakeFiles/sim.dir/build

roomba/CMakeFiles/sim.dir/requires: roomba/CMakeFiles/sim.dir/sim_files/src/sim.c.o.requires
roomba/CMakeFiles/sim.dir/requires: roomba/CMakeFiles/sim.dir/sim_files/src/load_plan.c.o.requires
roomba/CMakeFiles/sim.dir/requires: roomba/CMakeFiles/sim.dir/sim_files/src/position_update.c.o.requires
roomba/CMakeFiles/sim.dir/requires: roomba/CMakeFiles/sim.dir/sim_files/src/battery_update.c.o.requires
roomba/CMakeFiles/sim.dir/requires: roomba/CMakeFiles/sim.dir/sim_files/src/container_update.c.o.requires
roomba/CMakeFiles/sim.dir/requires: roomba/CMakeFiles/sim.dir/sim_files/src/dist_sensors_update.c.o.requires
roomba/CMakeFiles/sim.dir/requires: roomba/CMakeFiles/sim.dir/sim_files/src/trash_generator.c.o.requires
roomba/CMakeFiles/sim.dir/requires: roomba/CMakeFiles/sim.dir/sim_files/src/quality_indexes.c.o.requires
roomba/CMakeFiles/sim.dir/requires: roomba/CMakeFiles/sim.dir/sim_files/src/sim_functions.c.o.requires
roomba/CMakeFiles/sim.dir/requires: roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_control.c.o.requires
roomba/CMakeFiles/sim.dir/requires: roomba/CMakeFiles/sim.dir/sim_files/src/sim_to_vis.c.o.requires
roomba/CMakeFiles/sim.dir/requires: roomba/CMakeFiles/sim.dir/sim_files/src/sim_from_control.c.o.requires

.PHONY : roomba/CMakeFiles/sim.dir/requires

roomba/CMakeFiles/sim.dir/clean:
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && $(CMAKE_COMMAND) -P CMakeFiles/sim.dir/cmake_clean.cmake
.PHONY : roomba/CMakeFiles/sim.dir/clean

roomba/CMakeFiles/sim.dir/depend:
	cd /home/piotr/Desktop/pm_project/roomba_env/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/piotr/Desktop/pm_project/roomba_env /home/piotr/Desktop/pm_project/roomba_env/roomba /home/piotr/Desktop/pm_project/roomba_env/build /home/piotr/Desktop/pm_project/roomba_env/build/roomba /home/piotr/Desktop/pm_project/roomba_env/build/roomba/CMakeFiles/sim.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : roomba/CMakeFiles/sim.dir/depend

