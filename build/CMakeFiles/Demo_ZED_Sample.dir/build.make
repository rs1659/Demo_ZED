# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/nvidia/Projects/Demo_ZED

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nvidia/Projects/Demo_ZED/build

# Include any dependencies generated for this target.
include CMakeFiles/Demo_ZED_Sample.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Demo_ZED_Sample.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Demo_ZED_Sample.dir/flags.make

CMakeFiles/Demo_ZED_Sample.dir/src/CameraOperations/CameraZED.cpp.o: CMakeFiles/Demo_ZED_Sample.dir/flags.make
CMakeFiles/Demo_ZED_Sample.dir/src/CameraOperations/CameraZED.cpp.o: ../src/CameraOperations/CameraZED.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nvidia/Projects/Demo_ZED/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Demo_ZED_Sample.dir/src/CameraOperations/CameraZED.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Demo_ZED_Sample.dir/src/CameraOperations/CameraZED.cpp.o -c /home/nvidia/Projects/Demo_ZED/src/CameraOperations/CameraZED.cpp

CMakeFiles/Demo_ZED_Sample.dir/src/CameraOperations/CameraZED.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Demo_ZED_Sample.dir/src/CameraOperations/CameraZED.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nvidia/Projects/Demo_ZED/src/CameraOperations/CameraZED.cpp > CMakeFiles/Demo_ZED_Sample.dir/src/CameraOperations/CameraZED.cpp.i

CMakeFiles/Demo_ZED_Sample.dir/src/CameraOperations/CameraZED.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Demo_ZED_Sample.dir/src/CameraOperations/CameraZED.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nvidia/Projects/Demo_ZED/src/CameraOperations/CameraZED.cpp -o CMakeFiles/Demo_ZED_Sample.dir/src/CameraOperations/CameraZED.cpp.s

CMakeFiles/Demo_ZED_Sample.dir/src/CameraOperations/TrackingViewer.cpp.o: CMakeFiles/Demo_ZED_Sample.dir/flags.make
CMakeFiles/Demo_ZED_Sample.dir/src/CameraOperations/TrackingViewer.cpp.o: ../src/CameraOperations/TrackingViewer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nvidia/Projects/Demo_ZED/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Demo_ZED_Sample.dir/src/CameraOperations/TrackingViewer.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Demo_ZED_Sample.dir/src/CameraOperations/TrackingViewer.cpp.o -c /home/nvidia/Projects/Demo_ZED/src/CameraOperations/TrackingViewer.cpp

CMakeFiles/Demo_ZED_Sample.dir/src/CameraOperations/TrackingViewer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Demo_ZED_Sample.dir/src/CameraOperations/TrackingViewer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nvidia/Projects/Demo_ZED/src/CameraOperations/TrackingViewer.cpp > CMakeFiles/Demo_ZED_Sample.dir/src/CameraOperations/TrackingViewer.cpp.i

CMakeFiles/Demo_ZED_Sample.dir/src/CameraOperations/TrackingViewer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Demo_ZED_Sample.dir/src/CameraOperations/TrackingViewer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nvidia/Projects/Demo_ZED/src/CameraOperations/TrackingViewer.cpp -o CMakeFiles/Demo_ZED_Sample.dir/src/CameraOperations/TrackingViewer.cpp.s

CMakeFiles/Demo_ZED_Sample.dir/src/main.cpp.o: CMakeFiles/Demo_ZED_Sample.dir/flags.make
CMakeFiles/Demo_ZED_Sample.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nvidia/Projects/Demo_ZED/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/Demo_ZED_Sample.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Demo_ZED_Sample.dir/src/main.cpp.o -c /home/nvidia/Projects/Demo_ZED/src/main.cpp

CMakeFiles/Demo_ZED_Sample.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Demo_ZED_Sample.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nvidia/Projects/Demo_ZED/src/main.cpp > CMakeFiles/Demo_ZED_Sample.dir/src/main.cpp.i

CMakeFiles/Demo_ZED_Sample.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Demo_ZED_Sample.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nvidia/Projects/Demo_ZED/src/main.cpp -o CMakeFiles/Demo_ZED_Sample.dir/src/main.cpp.s

# Object files for target Demo_ZED_Sample
Demo_ZED_Sample_OBJECTS = \
"CMakeFiles/Demo_ZED_Sample.dir/src/CameraOperations/CameraZED.cpp.o" \
"CMakeFiles/Demo_ZED_Sample.dir/src/CameraOperations/TrackingViewer.cpp.o" \
"CMakeFiles/Demo_ZED_Sample.dir/src/main.cpp.o"

# External object files for target Demo_ZED_Sample
Demo_ZED_Sample_EXTERNAL_OBJECTS =

Demo_ZED_Sample: CMakeFiles/Demo_ZED_Sample.dir/src/CameraOperations/CameraZED.cpp.o
Demo_ZED_Sample: CMakeFiles/Demo_ZED_Sample.dir/src/CameraOperations/TrackingViewer.cpp.o
Demo_ZED_Sample: CMakeFiles/Demo_ZED_Sample.dir/src/main.cpp.o
Demo_ZED_Sample: CMakeFiles/Demo_ZED_Sample.dir/build.make
Demo_ZED_Sample: /usr/local/zed/lib/libsl_zed.so
Demo_ZED_Sample: /usr/lib/aarch64-linux-gnu/libopenblas.so
Demo_ZED_Sample: /usr/lib/aarch64-linux-gnu/libusb-1.0.so
Demo_ZED_Sample: /usr/lib/aarch64-linux-gnu/libcuda.so
Demo_ZED_Sample: /usr/local/cuda/lib64/libcudart.so
Demo_ZED_Sample: /usr/lib/aarch64-linux-gnu/libOpenGL.so
Demo_ZED_Sample: /usr/lib/aarch64-linux-gnu/libGLX.so
Demo_ZED_Sample: /usr/lib/aarch64-linux-gnu/libGLU.so
Demo_ZED_Sample: /usr/lib/aarch64-linux-gnu/libglut.so
Demo_ZED_Sample: /usr/lib/aarch64-linux-gnu/libGLEW.so
Demo_ZED_Sample: /usr/lib/aarch64-linux-gnu/libopencv_gapi.so.4.5.4
Demo_ZED_Sample: /usr/lib/aarch64-linux-gnu/libopencv_highgui.so.4.5.4
Demo_ZED_Sample: /usr/lib/aarch64-linux-gnu/libopencv_ml.so.4.5.4
Demo_ZED_Sample: /usr/lib/aarch64-linux-gnu/libopencv_objdetect.so.4.5.4
Demo_ZED_Sample: /usr/lib/aarch64-linux-gnu/libopencv_photo.so.4.5.4
Demo_ZED_Sample: /usr/lib/aarch64-linux-gnu/libopencv_stitching.so.4.5.4
Demo_ZED_Sample: /usr/lib/aarch64-linux-gnu/libopencv_video.so.4.5.4
Demo_ZED_Sample: /usr/lib/aarch64-linux-gnu/libopencv_videoio.so.4.5.4
Demo_ZED_Sample: /usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.5.4
Demo_ZED_Sample: /usr/lib/aarch64-linux-gnu/libopencv_dnn.so.4.5.4
Demo_ZED_Sample: /usr/lib/aarch64-linux-gnu/libopencv_calib3d.so.4.5.4
Demo_ZED_Sample: /usr/lib/aarch64-linux-gnu/libopencv_features2d.so.4.5.4
Demo_ZED_Sample: /usr/lib/aarch64-linux-gnu/libopencv_flann.so.4.5.4
Demo_ZED_Sample: /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.5.4
Demo_ZED_Sample: /usr/lib/aarch64-linux-gnu/libopencv_core.so.4.5.4
Demo_ZED_Sample: CMakeFiles/Demo_ZED_Sample.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nvidia/Projects/Demo_ZED/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable Demo_ZED_Sample"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Demo_ZED_Sample.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Demo_ZED_Sample.dir/build: Demo_ZED_Sample

.PHONY : CMakeFiles/Demo_ZED_Sample.dir/build

CMakeFiles/Demo_ZED_Sample.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Demo_ZED_Sample.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Demo_ZED_Sample.dir/clean

CMakeFiles/Demo_ZED_Sample.dir/depend:
	cd /home/nvidia/Projects/Demo_ZED/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nvidia/Projects/Demo_ZED /home/nvidia/Projects/Demo_ZED /home/nvidia/Projects/Demo_ZED/build /home/nvidia/Projects/Demo_ZED/build /home/nvidia/Projects/Demo_ZED/build/CMakeFiles/Demo_ZED_Sample.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Demo_ZED_Sample.dir/depend

