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
include roomba/CMakeFiles/roomba.dir/depend.make

# Include the progress variables for this target.
include roomba/CMakeFiles/roomba.dir/progress.make

# Include the compile flags for this target's objects.
include roomba/CMakeFiles/roomba.dir/flags.make

roomba/CMakeFiles/roomba.dir/roomba.c.o: roomba/CMakeFiles/roomba.dir/flags.make
roomba/CMakeFiles/roomba.dir/roomba.c.o: ../roomba/roomba.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/piotr/Desktop/pm_project/roomba_env/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object roomba/CMakeFiles/roomba.dir/roomba.c.o"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/roomba.dir/roomba.c.o   -c /home/piotr/Desktop/pm_project/roomba_env/roomba/roomba.c

roomba/CMakeFiles/roomba.dir/roomba.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/roomba.dir/roomba.c.i"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/piotr/Desktop/pm_project/roomba_env/roomba/roomba.c > CMakeFiles/roomba.dir/roomba.c.i

roomba/CMakeFiles/roomba.dir/roomba.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/roomba.dir/roomba.c.s"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/piotr/Desktop/pm_project/roomba_env/roomba/roomba.c -o CMakeFiles/roomba.dir/roomba.c.s

roomba/CMakeFiles/roomba.dir/roomba.c.o.requires:

.PHONY : roomba/CMakeFiles/roomba.dir/roomba.c.o.requires

roomba/CMakeFiles/roomba.dir/roomba.c.o.provides: roomba/CMakeFiles/roomba.dir/roomba.c.o.requires
	$(MAKE) -f roomba/CMakeFiles/roomba.dir/build.make roomba/CMakeFiles/roomba.dir/roomba.c.o.provides.build
.PHONY : roomba/CMakeFiles/roomba.dir/roomba.c.o.provides

roomba/CMakeFiles/roomba.dir/roomba.c.o.provides.build: roomba/CMakeFiles/roomba.dir/roomba.c.o


# Object files for target roomba
roomba_OBJECTS = \
"CMakeFiles/roomba.dir/roomba.c.o"

# External object files for target roomba
roomba_EXTERNAL_OBJECTS =

roomba/roomba: roomba/CMakeFiles/roomba.dir/roomba.c.o
roomba/roomba: roomba/CMakeFiles/roomba.dir/build.make
roomba/roomba: roomba/CMakeFiles/roomba.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/piotr/Desktop/pm_project/roomba_env/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable roomba"
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/roomba.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
roomba/CMakeFiles/roomba.dir/build: roomba/roomba

.PHONY : roomba/CMakeFiles/roomba.dir/build

roomba/CMakeFiles/roomba.dir/requires: roomba/CMakeFiles/roomba.dir/roomba.c.o.requires

.PHONY : roomba/CMakeFiles/roomba.dir/requires

roomba/CMakeFiles/roomba.dir/clean:
	cd /home/piotr/Desktop/pm_project/roomba_env/build/roomba && $(CMAKE_COMMAND) -P CMakeFiles/roomba.dir/cmake_clean.cmake
.PHONY : roomba/CMakeFiles/roomba.dir/clean

roomba/CMakeFiles/roomba.dir/depend:
	cd /home/piotr/Desktop/pm_project/roomba_env/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/piotr/Desktop/pm_project/roomba_env /home/piotr/Desktop/pm_project/roomba_env/roomba /home/piotr/Desktop/pm_project/roomba_env/build /home/piotr/Desktop/pm_project/roomba_env/build/roomba /home/piotr/Desktop/pm_project/roomba_env/build/roomba/CMakeFiles/roomba.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : roomba/CMakeFiles/roomba.dir/depend

