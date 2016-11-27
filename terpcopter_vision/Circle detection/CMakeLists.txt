CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
project(Project_DetectCircles)

SET(execName DetectCircles)

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)  
endif(COMMAND cmake_policy)

SET(EXECUTABLE_OUTPUT_PATH ".")

if(MSVC12)
	SET(VS_VERSION VS2013)
else(MSVC12) 
	if(MSVC11)
		SET(VS_VERSION VS2012)
	else(MSVC11)
		if(MSVC10)
			SET(VS_VERSION VS2010)
		endif(MSVC10)
	endif(MSVC11)
endif(MSVC12)

SET(ZED_INCLUDE_DIRS $ENV{ZED_INCLUDE_DIRS})
SET(ZED_LIBRARY_DIR $ENV{ZED_LIBRARY_DIR})
if (CMAKE_CL_64) # 64 bits
	SET(ZED_LIBRARIES $ENV{ZED_LIBRARIES_64})
else(CMAKE_CL_64) # 32 bits
	SET(ZED_LIBRARIES $ENV{ZED_LIBRARIES_32})
endif(CMAKE_CL_64)


# Mandatory
find_package(OpenCV 3.1 COMPONENTS core highgui imgproc REQUIRED) #for zed SDK 1.0 -> opencv 3.1


include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${OpenCV_LIBRARY_DIRS})

SET(SRC_FOLDER src)
FILE(GLOB_RECURSE SRC_FILES "${SRC_FOLDER}/*.cpp")
 
ADD_EXECUTABLE(${execName} ${SRC_FILES})
set_property(TARGET ${execName} PROPERTY OUTPUT_NAME "DetectCircles")
add_definitions(-std=c++0x)# -m64)

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${execName}		
			${OpenCV_LIBRARIES}
 )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3" ) # Release Perf mode
