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
CMAKE_SOURCE_DIR = /home/piotr/Desktop/pm_project/roomba_env/roomba

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/piotr/Desktop/pm_project/roomba_env/roomba/build

# Include any dependencies generated for this target.
include CMakeFiles/roomba.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/roomba.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/roomba.dir/flags.make

CMakeFiles/roomba.dir/roomba.o: CMakeFiles/roomba.dir/flags.make
CMakeFiles/roomba.dir/roomba.o: ../roomba.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/piotr/Desktop/pm_project/roomba_env/roomba/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/roomba.dir/roomba.o"
	/usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/roomba.dir/roomba.o   -c /home/piotr/Desktop/pm_project/roomba_env/roomba/roomba.c

CMakeFiles/roomba.dir/roomba.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/roomba.dir/roomba.i"
	/usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/piotr/Desktop/pm_project/roomba_env/roomba/roomba.c > CMakeFiles/roomba.dir/roomba.i

CMakeFiles/roomba.dir/roomba.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/roomba.dir/roomba.s"
	/usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/piotr/Desktop/pm_project/roomba_env/roomba/roomba.c -o CMakeFiles/roomba.dir/roomba.s

CMakeFiles/roomba.dir/roomba.o.requires:

.PHONY : CMakeFiles/roomba.dir/roomba.o.requires

CMakeFiles/roomba.dir/roomba.o.provides: CMakeFiles/roomba.dir/roomba.o.requires
	$(MAKE) -f CMakeFiles/roomba.dir/build.make CMakeFiles/roomba.dir/roomba.o.provides.build
.PHONY : CMakeFiles/roomba.dir/roomba.o.provides

CMakeFiles/roomba.dir/roomba.o.provides.build: CMakeFiles/roomba.dir/roomba.o


# Object files for target roomba
roomba_OBJECTS = \
"CMakeFiles/roomba.dir/roomba.o"

# External object files for target roomba
roomba_EXTERNAL_OBJECTS =

roomba: CMakeFiles/roomba.dir/roomba.o
roomba: CMakeFiles/roomba.dir/build.make
roomba: CMakeFiles/roomba.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/piotr/Desktop/pm_project/roomba_env/roomba/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable roomba"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/roomba.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/roomba.dir/build: roomba

.PHONY : CMakeFiles/roomba.dir/build

CMakeFiles/roomba.dir/requires: CMakeFiles/roomba.dir/roomba.o.requires

.PHONY : CMakeFiles/roomba.dir/requires

CMakeFiles/roomba.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/roomba.dir/cmake_clean.cmake
.PHONY : CMakeFiles/roomba.dir/clean

CMakeFiles/roomba.dir/depend:
	cd /home/piotr/Desktop/pm_project/roomba_env/roomba/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/piotr/Desktop/pm_project/roomba_env/roomba /home/piotr/Desktop/pm_project/roomba_env/roomba /home/piotr/Desktop/pm_project/roomba_env/roomba/build /home/piotr/Desktop/pm_project/roomba_env/roomba/build /home/piotr/Desktop/pm_project/roomba_env/roomba/build/CMakeFiles/roomba.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/roomba.dir/depend
