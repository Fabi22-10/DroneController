# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

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
CMAKE_SOURCE_DIR = /home/drone/drone/raspberry-sbus/examples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/drone/drone/raspberry-sbus/examples/build

# Include any dependencies generated for this target.
include CMakeFiles/blocking_receiver.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/blocking_receiver.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/blocking_receiver.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/blocking_receiver.dir/flags.make

CMakeFiles/blocking_receiver.dir/blocking_receiver.o: CMakeFiles/blocking_receiver.dir/flags.make
CMakeFiles/blocking_receiver.dir/blocking_receiver.o: /home/drone/drone/raspberry-sbus/examples/blocking_receiver.cpp
CMakeFiles/blocking_receiver.dir/blocking_receiver.o: CMakeFiles/blocking_receiver.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/drone/drone/raspberry-sbus/examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/blocking_receiver.dir/blocking_receiver.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/blocking_receiver.dir/blocking_receiver.o -MF CMakeFiles/blocking_receiver.dir/blocking_receiver.o.d -o CMakeFiles/blocking_receiver.dir/blocking_receiver.o -c /home/drone/drone/raspberry-sbus/examples/blocking_receiver.cpp

CMakeFiles/blocking_receiver.dir/blocking_receiver.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/blocking_receiver.dir/blocking_receiver.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/drone/drone/raspberry-sbus/examples/blocking_receiver.cpp > CMakeFiles/blocking_receiver.dir/blocking_receiver.i

CMakeFiles/blocking_receiver.dir/blocking_receiver.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/blocking_receiver.dir/blocking_receiver.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/drone/drone/raspberry-sbus/examples/blocking_receiver.cpp -o CMakeFiles/blocking_receiver.dir/blocking_receiver.s

# Object files for target blocking_receiver
blocking_receiver_OBJECTS = \
"CMakeFiles/blocking_receiver.dir/blocking_receiver.o"

# External object files for target blocking_receiver
blocking_receiver_EXTERNAL_OBJECTS =

blocking_receiver: CMakeFiles/blocking_receiver.dir/blocking_receiver.o
blocking_receiver: CMakeFiles/blocking_receiver.dir/build.make
blocking_receiver: CMakeFiles/blocking_receiver.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/drone/drone/raspberry-sbus/examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable blocking_receiver"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/blocking_receiver.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/blocking_receiver.dir/build: blocking_receiver
.PHONY : CMakeFiles/blocking_receiver.dir/build

CMakeFiles/blocking_receiver.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/blocking_receiver.dir/cmake_clean.cmake
.PHONY : CMakeFiles/blocking_receiver.dir/clean

CMakeFiles/blocking_receiver.dir/depend:
	cd /home/drone/drone/raspberry-sbus/examples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/drone/drone/raspberry-sbus/examples /home/drone/drone/raspberry-sbus/examples /home/drone/drone/raspberry-sbus/examples/build /home/drone/drone/raspberry-sbus/examples/build /home/drone/drone/raspberry-sbus/examples/build/CMakeFiles/blocking_receiver.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/blocking_receiver.dir/depend

