# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Produce verbose output by default.
VERBOSE = 1

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
CMAKE_SOURCE_DIR = /home/avasalya/mc_handover_controller

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/avasalya/mc_handover_controller/build

# Include any dependencies generated for this target.
include src/CMakeFiles/mc_handover_controller.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/mc_handover_controller.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/mc_handover_controller.dir/flags.make

src/CMakeFiles/mc_handover_controller.dir/mc_handover_controller.cpp.o: src/CMakeFiles/mc_handover_controller.dir/flags.make
src/CMakeFiles/mc_handover_controller.dir/mc_handover_controller.cpp.o: ../src/mc_handover_controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/avasalya/mc_handover_controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/mc_handover_controller.dir/mc_handover_controller.cpp.o"
	cd /home/avasalya/mc_handover_controller/build/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mc_handover_controller.dir/mc_handover_controller.cpp.o -c /home/avasalya/mc_handover_controller/src/mc_handover_controller.cpp

src/CMakeFiles/mc_handover_controller.dir/mc_handover_controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mc_handover_controller.dir/mc_handover_controller.cpp.i"
	cd /home/avasalya/mc_handover_controller/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/avasalya/mc_handover_controller/src/mc_handover_controller.cpp > CMakeFiles/mc_handover_controller.dir/mc_handover_controller.cpp.i

src/CMakeFiles/mc_handover_controller.dir/mc_handover_controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mc_handover_controller.dir/mc_handover_controller.cpp.s"
	cd /home/avasalya/mc_handover_controller/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/avasalya/mc_handover_controller/src/mc_handover_controller.cpp -o CMakeFiles/mc_handover_controller.dir/mc_handover_controller.cpp.s

src/CMakeFiles/mc_handover_controller.dir/mc_handover_controller.cpp.o.requires:

.PHONY : src/CMakeFiles/mc_handover_controller.dir/mc_handover_controller.cpp.o.requires

src/CMakeFiles/mc_handover_controller.dir/mc_handover_controller.cpp.o.provides: src/CMakeFiles/mc_handover_controller.dir/mc_handover_controller.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/mc_handover_controller.dir/build.make src/CMakeFiles/mc_handover_controller.dir/mc_handover_controller.cpp.o.provides.build
.PHONY : src/CMakeFiles/mc_handover_controller.dir/mc_handover_controller.cpp.o.provides

src/CMakeFiles/mc_handover_controller.dir/mc_handover_controller.cpp.o.provides.build: src/CMakeFiles/mc_handover_controller.dir/mc_handover_controller.cpp.o


# Object files for target mc_handover_controller
mc_handover_controller_OBJECTS = \
"CMakeFiles/mc_handover_controller.dir/mc_handover_controller.cpp.o"

# External object files for target mc_handover_controller
mc_handover_controller_EXTERNAL_OBJECTS =

src/mc_handover_controller.so: src/CMakeFiles/mc_handover_controller.dir/mc_handover_controller.cpp.o
src/mc_handover_controller.so: src/CMakeFiles/mc_handover_controller.dir/build.make
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libpthread.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/liblz4.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_signals.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libpthread.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libpthread.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libpthread.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libpthread.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/liblz4.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_signals.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libpthread.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libpthread.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libpthread.so
src/mc_handover_controller.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
src/mc_handover_controller.so: src/CMakeFiles/mc_handover_controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/avasalya/mc_handover_controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library mc_handover_controller.so"
	cd /home/avasalya/mc_handover_controller/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mc_handover_controller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/mc_handover_controller.dir/build: src/mc_handover_controller.so

.PHONY : src/CMakeFiles/mc_handover_controller.dir/build

src/CMakeFiles/mc_handover_controller.dir/requires: src/CMakeFiles/mc_handover_controller.dir/mc_handover_controller.cpp.o.requires

.PHONY : src/CMakeFiles/mc_handover_controller.dir/requires

src/CMakeFiles/mc_handover_controller.dir/clean:
	cd /home/avasalya/mc_handover_controller/build/src && $(CMAKE_COMMAND) -P CMakeFiles/mc_handover_controller.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/mc_handover_controller.dir/clean

src/CMakeFiles/mc_handover_controller.dir/depend:
	cd /home/avasalya/mc_handover_controller/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/avasalya/mc_handover_controller /home/avasalya/mc_handover_controller/src /home/avasalya/mc_handover_controller/build /home/avasalya/mc_handover_controller/build/src /home/avasalya/mc_handover_controller/build/src/CMakeFiles/mc_handover_controller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/mc_handover_controller.dir/depend

