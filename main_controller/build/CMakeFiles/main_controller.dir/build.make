# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.18

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/student/Documents/lab_git_augu/info_robot/main_controller

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/student/Documents/lab_git_augu/info_robot/main_controller/build

# Include any dependencies generated for this target.
include CMakeFiles/main_controller.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/main_controller.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/main_controller.dir/flags.make

CMakeFiles/main_controller.dir/main.c.o: CMakeFiles/main_controller.dir/flags.make
CMakeFiles/main_controller.dir/main.c.o: ../main.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/Documents/lab_git_augu/info_robot/main_controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/main_controller.dir/main.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/main_controller.dir/main.c.o -c /home/student/Documents/lab_git_augu/info_robot/main_controller/main.c

CMakeFiles/main_controller.dir/main.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/main_controller.dir/main.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/student/Documents/lab_git_augu/info_robot/main_controller/main.c > CMakeFiles/main_controller.dir/main.c.i

CMakeFiles/main_controller.dir/main.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/main_controller.dir/main.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/student/Documents/lab_git_augu/info_robot/main_controller/main.c -o CMakeFiles/main_controller.dir/main.c.s

CMakeFiles/main_controller.dir/variables.c.o: CMakeFiles/main_controller.dir/flags.make
CMakeFiles/main_controller.dir/variables.c.o: ../variables.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/Documents/lab_git_augu/info_robot/main_controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/main_controller.dir/variables.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/main_controller.dir/variables.c.o -c /home/student/Documents/lab_git_augu/info_robot/main_controller/variables.c

CMakeFiles/main_controller.dir/variables.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/main_controller.dir/variables.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/student/Documents/lab_git_augu/info_robot/main_controller/variables.c > CMakeFiles/main_controller.dir/variables.c.i

CMakeFiles/main_controller.dir/variables.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/main_controller.dir/variables.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/student/Documents/lab_git_augu/info_robot/main_controller/variables.c -o CMakeFiles/main_controller.dir/variables.c.s

CMakeFiles/main_controller.dir/fieldpath.c.o: CMakeFiles/main_controller.dir/flags.make
CMakeFiles/main_controller.dir/fieldpath.c.o: ../fieldpath.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/Documents/lab_git_augu/info_robot/main_controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/main_controller.dir/fieldpath.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/main_controller.dir/fieldpath.c.o -c /home/student/Documents/lab_git_augu/info_robot/main_controller/fieldpath.c

CMakeFiles/main_controller.dir/fieldpath.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/main_controller.dir/fieldpath.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/student/Documents/lab_git_augu/info_robot/main_controller/fieldpath.c > CMakeFiles/main_controller.dir/fieldpath.c.i

CMakeFiles/main_controller.dir/fieldpath.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/main_controller.dir/fieldpath.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/student/Documents/lab_git_augu/info_robot/main_controller/fieldpath.c -o CMakeFiles/main_controller.dir/fieldpath.c.s

CMakeFiles/main_controller.dir/spi_com.c.o: CMakeFiles/main_controller.dir/flags.make
CMakeFiles/main_controller.dir/spi_com.c.o: ../spi_com.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/Documents/lab_git_augu/info_robot/main_controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object CMakeFiles/main_controller.dir/spi_com.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/main_controller.dir/spi_com.c.o -c /home/student/Documents/lab_git_augu/info_robot/main_controller/spi_com.c

CMakeFiles/main_controller.dir/spi_com.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/main_controller.dir/spi_com.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/student/Documents/lab_git_augu/info_robot/main_controller/spi_com.c > CMakeFiles/main_controller.dir/spi_com.c.i

CMakeFiles/main_controller.dir/spi_com.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/main_controller.dir/spi_com.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/student/Documents/lab_git_augu/info_robot/main_controller/spi_com.c -o CMakeFiles/main_controller.dir/spi_com.c.s

CMakeFiles/main_controller.dir/utils.c.o: CMakeFiles/main_controller.dir/flags.make
CMakeFiles/main_controller.dir/utils.c.o: ../utils.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/Documents/lab_git_augu/info_robot/main_controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object CMakeFiles/main_controller.dir/utils.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/main_controller.dir/utils.c.o -c /home/student/Documents/lab_git_augu/info_robot/main_controller/utils.c

CMakeFiles/main_controller.dir/utils.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/main_controller.dir/utils.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/student/Documents/lab_git_augu/info_robot/main_controller/utils.c > CMakeFiles/main_controller.dir/utils.c.i

CMakeFiles/main_controller.dir/utils.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/main_controller.dir/utils.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/student/Documents/lab_git_augu/info_robot/main_controller/utils.c -o CMakeFiles/main_controller.dir/utils.c.s

CMakeFiles/main_controller.dir/controller.c.o: CMakeFiles/main_controller.dir/flags.make
CMakeFiles/main_controller.dir/controller.c.o: ../controller.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/Documents/lab_git_augu/info_robot/main_controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object CMakeFiles/main_controller.dir/controller.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/main_controller.dir/controller.c.o -c /home/student/Documents/lab_git_augu/info_robot/main_controller/controller.c

CMakeFiles/main_controller.dir/controller.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/main_controller.dir/controller.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/student/Documents/lab_git_augu/info_robot/main_controller/controller.c > CMakeFiles/main_controller.dir/controller.c.i

CMakeFiles/main_controller.dir/controller.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/main_controller.dir/controller.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/student/Documents/lab_git_augu/info_robot/main_controller/controller.c -o CMakeFiles/main_controller.dir/controller.c.s

# Object files for target main_controller
main_controller_OBJECTS = \
"CMakeFiles/main_controller.dir/main.c.o" \
"CMakeFiles/main_controller.dir/variables.c.o" \
"CMakeFiles/main_controller.dir/fieldpath.c.o" \
"CMakeFiles/main_controller.dir/spi_com.c.o" \
"CMakeFiles/main_controller.dir/utils.c.o" \
"CMakeFiles/main_controller.dir/controller.c.o"

# External object files for target main_controller
main_controller_EXTERNAL_OBJECTS =

main_controller: CMakeFiles/main_controller.dir/main.c.o
main_controller: CMakeFiles/main_controller.dir/variables.c.o
main_controller: CMakeFiles/main_controller.dir/fieldpath.c.o
main_controller: CMakeFiles/main_controller.dir/spi_com.c.o
main_controller: CMakeFiles/main_controller.dir/utils.c.o
main_controller: CMakeFiles/main_controller.dir/controller.c.o
main_controller: CMakeFiles/main_controller.dir/build.make
main_controller: /usr/lib/libpigpio.so
main_controller: CMakeFiles/main_controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/student/Documents/lab_git_augu/info_robot/main_controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking C executable main_controller"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/main_controller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/main_controller.dir/build: main_controller

.PHONY : CMakeFiles/main_controller.dir/build

CMakeFiles/main_controller.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/main_controller.dir/cmake_clean.cmake
.PHONY : CMakeFiles/main_controller.dir/clean

CMakeFiles/main_controller.dir/depend:
	cd /home/student/Documents/lab_git_augu/info_robot/main_controller/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/student/Documents/lab_git_augu/info_robot/main_controller /home/student/Documents/lab_git_augu/info_robot/main_controller /home/student/Documents/lab_git_augu/info_robot/main_controller/build /home/student/Documents/lab_git_augu/info_robot/main_controller/build /home/student/Documents/lab_git_augu/info_robot/main_controller/build/CMakeFiles/main_controller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/main_controller.dir/depend

