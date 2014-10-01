# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/home/sriram/projects/Independent Motion Detection"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/sriram/projects/Independent Motion Detection/build"

# Include any dependencies generated for this target.
include mover/CMakeFiles/mover.dir/depend.make

# Include the progress variables for this target.
include mover/CMakeFiles/mover.dir/progress.make

# Include the compile flags for this target's objects.
include mover/CMakeFiles/mover.dir/flags.make

mover/CMakeFiles/mover.dir/main.cpp.o: mover/CMakeFiles/mover.dir/flags.make
mover/CMakeFiles/mover.dir/main.cpp.o: ../mover/main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report "/home/sriram/projects/Independent Motion Detection/build/CMakeFiles" $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object mover/CMakeFiles/mover.dir/main.cpp.o"
	cd "/home/sriram/projects/Independent Motion Detection/build/mover" && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/mover.dir/main.cpp.o -c "/home/sriram/projects/Independent Motion Detection/mover/main.cpp"

mover/CMakeFiles/mover.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mover.dir/main.cpp.i"
	cd "/home/sriram/projects/Independent Motion Detection/build/mover" && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E "/home/sriram/projects/Independent Motion Detection/mover/main.cpp" > CMakeFiles/mover.dir/main.cpp.i

mover/CMakeFiles/mover.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mover.dir/main.cpp.s"
	cd "/home/sriram/projects/Independent Motion Detection/build/mover" && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S "/home/sriram/projects/Independent Motion Detection/mover/main.cpp" -o CMakeFiles/mover.dir/main.cpp.s

mover/CMakeFiles/mover.dir/main.cpp.o.requires:
.PHONY : mover/CMakeFiles/mover.dir/main.cpp.o.requires

mover/CMakeFiles/mover.dir/main.cpp.o.provides: mover/CMakeFiles/mover.dir/main.cpp.o.requires
	$(MAKE) -f mover/CMakeFiles/mover.dir/build.make mover/CMakeFiles/mover.dir/main.cpp.o.provides.build
.PHONY : mover/CMakeFiles/mover.dir/main.cpp.o.provides

mover/CMakeFiles/mover.dir/main.cpp.o.provides.build: mover/CMakeFiles/mover.dir/main.cpp.o

# Object files for target mover
mover_OBJECTS = \
"CMakeFiles/mover.dir/main.cpp.o"

# External object files for target mover
mover_EXTERNAL_OBJECTS =

bin/mover: mover/CMakeFiles/mover.dir/main.cpp.o
bin/mover: mover/CMakeFiles/mover.dir/build.make
bin/mover: /usr/lib/libgsl.so
bin/mover: /usr/lib/libgslcblas.so
bin/mover: /home/sriram/Documents/yarp-2.3.62/build/lib/libYARP_OS.a
bin/mover: /home/sriram/Documents/yarp-2.3.62/build/lib/libYARP_sig.a
bin/mover: /home/sriram/Documents/yarp-2.3.62/build/lib/libYARP_math.a
bin/mover: /home/sriram/Documents/yarp-2.3.62/build/lib/libYARP_dev.a
bin/mover: /home/sriram/Documents/yarp-2.3.62/build/lib/libYARP_name.a
bin/mover: /home/sriram/Documents/yarp-2.3.62/build/lib/libYARP_init.a
bin/mover: /usr/lib/libgsl.so
bin/mover: /usr/lib/libgslcblas.so
bin/mover: /home/sriram/Documents/yarp-2.3.62/build/lib/libYARP_sig.a
bin/mover: /home/sriram/Documents/yarp-2.3.62/build/lib/libYARP_OS.a
bin/mover: /usr/lib/libACE.so
bin/mover: /usr/lib/x86_64-linux-gnu/libdl.so
bin/mover: /usr/lib/x86_64-linux-gnu/librt.so
bin/mover: mover/CMakeFiles/mover.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/mover"
	cd "/home/sriram/projects/Independent Motion Detection/build/mover" && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mover.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
mover/CMakeFiles/mover.dir/build: bin/mover
.PHONY : mover/CMakeFiles/mover.dir/build

mover/CMakeFiles/mover.dir/requires: mover/CMakeFiles/mover.dir/main.cpp.o.requires
.PHONY : mover/CMakeFiles/mover.dir/requires

mover/CMakeFiles/mover.dir/clean:
	cd "/home/sriram/projects/Independent Motion Detection/build/mover" && $(CMAKE_COMMAND) -P CMakeFiles/mover.dir/cmake_clean.cmake
.PHONY : mover/CMakeFiles/mover.dir/clean

mover/CMakeFiles/mover.dir/depend:
	cd "/home/sriram/projects/Independent Motion Detection/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/sriram/projects/Independent Motion Detection" "/home/sriram/projects/Independent Motion Detection/mover" "/home/sriram/projects/Independent Motion Detection/build" "/home/sriram/projects/Independent Motion Detection/build/mover" "/home/sriram/projects/Independent Motion Detection/build/mover/CMakeFiles/mover.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : mover/CMakeFiles/mover.dir/depend
