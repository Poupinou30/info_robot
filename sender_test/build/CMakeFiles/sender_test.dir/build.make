# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_SOURCE_DIR = /home/augustin/Documents/receiver_test/sender_test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/augustin/Documents/receiver_test/sender_test/build

# Include any dependencies generated for this target.
include CMakeFiles/sender_test.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/sender_test.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/sender_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sender_test.dir/flags.make

CMakeFiles/sender_test.dir/main.cpp.o: CMakeFiles/sender_test.dir/flags.make
CMakeFiles/sender_test.dir/main.cpp.o: ../main.cpp
CMakeFiles/sender_test.dir/main.cpp.o: CMakeFiles/sender_test.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/augustin/Documents/receiver_test/sender_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/sender_test.dir/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/sender_test.dir/main.cpp.o -MF CMakeFiles/sender_test.dir/main.cpp.o.d -o CMakeFiles/sender_test.dir/main.cpp.o -c /home/augustin/Documents/receiver_test/sender_test/main.cpp

CMakeFiles/sender_test.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sender_test.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/augustin/Documents/receiver_test/sender_test/main.cpp > CMakeFiles/sender_test.dir/main.cpp.i

CMakeFiles/sender_test.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sender_test.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/augustin/Documents/receiver_test/sender_test/main.cpp -o CMakeFiles/sender_test.dir/main.cpp.s

# Object files for target sender_test
sender_test_OBJECTS = \
"CMakeFiles/sender_test.dir/main.cpp.o"

# External object files for target sender_test
sender_test_EXTERNAL_OBJECTS =

sender_test: CMakeFiles/sender_test.dir/main.cpp.o
sender_test: CMakeFiles/sender_test.dir/build.make
sender_test: CMakeFiles/sender_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/augustin/Documents/receiver_test/sender_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable sender_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sender_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sender_test.dir/build: sender_test
.PHONY : CMakeFiles/sender_test.dir/build

CMakeFiles/sender_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sender_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sender_test.dir/clean

CMakeFiles/sender_test.dir/depend:
	cd /home/augustin/Documents/receiver_test/sender_test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/augustin/Documents/receiver_test/sender_test /home/augustin/Documents/receiver_test/sender_test /home/augustin/Documents/receiver_test/sender_test/build /home/augustin/Documents/receiver_test/sender_test/build /home/augustin/Documents/receiver_test/sender_test/build/CMakeFiles/sender_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sender_test.dir/depend

