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
CMAKE_SOURCE_DIR = /home/howard/depth/trackingball

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/howard/depth/trackingball/build

# Include any dependencies generated for this target.
include CMakeFiles/tracking.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/tracking.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/tracking.dir/flags.make

CMakeFiles/tracking.dir/main.cpp.o: CMakeFiles/tracking.dir/flags.make
CMakeFiles/tracking.dir/main.cpp.o: ../main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/howard/depth/trackingball/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/tracking.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/tracking.dir/main.cpp.o -c /home/howard/depth/trackingball/main.cpp

CMakeFiles/tracking.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tracking.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/howard/depth/trackingball/main.cpp > CMakeFiles/tracking.dir/main.cpp.i

CMakeFiles/tracking.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tracking.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/howard/depth/trackingball/main.cpp -o CMakeFiles/tracking.dir/main.cpp.s

CMakeFiles/tracking.dir/main.cpp.o.requires:
.PHONY : CMakeFiles/tracking.dir/main.cpp.o.requires

CMakeFiles/tracking.dir/main.cpp.o.provides: CMakeFiles/tracking.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/tracking.dir/build.make CMakeFiles/tracking.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/tracking.dir/main.cpp.o.provides

CMakeFiles/tracking.dir/main.cpp.o.provides.build: CMakeFiles/tracking.dir/main.cpp.o

# Object files for target tracking
tracking_OBJECTS = \
"CMakeFiles/tracking.dir/main.cpp.o"

# External object files for target tracking
tracking_EXTERNAL_OBJECTS =

tracking: CMakeFiles/tracking.dir/main.cpp.o
tracking: CMakeFiles/tracking.dir/build.make
tracking: /usr/local/lib/libopencv_videostab.so.2.4.11
tracking: /usr/local/lib/libopencv_video.so.2.4.11
tracking: /usr/local/lib/libopencv_ts.a
tracking: /usr/local/lib/libopencv_superres.so.2.4.11
tracking: /usr/local/lib/libopencv_stitching.so.2.4.11
tracking: /usr/local/lib/libopencv_photo.so.2.4.11
tracking: /usr/local/lib/libopencv_ocl.so.2.4.11
tracking: /usr/local/lib/libopencv_objdetect.so.2.4.11
tracking: /usr/local/lib/libopencv_nonfree.so.2.4.11
tracking: /usr/local/lib/libopencv_ml.so.2.4.11
tracking: /usr/local/lib/libopencv_legacy.so.2.4.11
tracking: /usr/local/lib/libopencv_imgproc.so.2.4.11
tracking: /usr/local/lib/libopencv_highgui.so.2.4.11
tracking: /usr/local/lib/libopencv_flann.so.2.4.11
tracking: /usr/local/lib/libopencv_features2d.so.2.4.11
tracking: /usr/local/lib/libopencv_core.so.2.4.11
tracking: /usr/local/lib/libopencv_contrib.so.2.4.11
tracking: /usr/local/lib/libopencv_calib3d.so.2.4.11
tracking: /usr/lib/x86_64-linux-gnu/libGLU.so
tracking: /usr/lib/x86_64-linux-gnu/libGL.so
tracking: /usr/lib/x86_64-linux-gnu/libSM.so
tracking: /usr/lib/x86_64-linux-gnu/libICE.so
tracking: /usr/lib/x86_64-linux-gnu/libX11.so
tracking: /usr/lib/x86_64-linux-gnu/libXext.so
tracking: /usr/local/lib/libopencv_nonfree.so.2.4.11
tracking: /usr/local/lib/libopencv_ocl.so.2.4.11
tracking: /usr/local/lib/libopencv_video.so.2.4.11
tracking: /usr/local/lib/libopencv_objdetect.so.2.4.11
tracking: /usr/local/lib/libopencv_ml.so.2.4.11
tracking: /usr/local/lib/libopencv_calib3d.so.2.4.11
tracking: /usr/local/lib/libopencv_features2d.so.2.4.11
tracking: /usr/local/lib/libopencv_highgui.so.2.4.11
tracking: /usr/local/lib/libopencv_imgproc.so.2.4.11
tracking: /usr/local/lib/libopencv_flann.so.2.4.11
tracking: /usr/local/lib/libopencv_core.so.2.4.11
tracking: CMakeFiles/tracking.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable tracking"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tracking.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/tracking.dir/build: tracking
.PHONY : CMakeFiles/tracking.dir/build

CMakeFiles/tracking.dir/requires: CMakeFiles/tracking.dir/main.cpp.o.requires
.PHONY : CMakeFiles/tracking.dir/requires

CMakeFiles/tracking.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/tracking.dir/cmake_clean.cmake
.PHONY : CMakeFiles/tracking.dir/clean

CMakeFiles/tracking.dir/depend:
	cd /home/howard/depth/trackingball/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/howard/depth/trackingball /home/howard/depth/trackingball /home/howard/depth/trackingball/build /home/howard/depth/trackingball/build /home/howard/depth/trackingball/build/CMakeFiles/tracking.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/tracking.dir/depend

