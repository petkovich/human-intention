# Minimum required version.
cmake_minimum_required(VERSION 2.8.11)

# The project name.
set(PRJ_NAME IntentionRecognition)
project("${PRJ_NAME}")

# Compiler settings
set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_BUILD_TYPE release)
set(CMAKE_BUILD_TYPE debug)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wfloat-equal -Wundef -Wcast-align -Wwrite-strings -Wlogical-op -Wmissing-declarations -Wredundant-decls -Wshadow -Woverloaded-virtual -pthread")
if(CMAKE_MAJOR_VERSION EQUAL "2")
	add_definitions(-std=c++14)
endif()

# Set directories
#	This project
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib")
set(INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
#   HumanAwareRoutePlanning library include
set(PLANNER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../HumanAwareRoutePlanning/Planning/include")
set(PLANNER_DATASTRUCTURE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../HumanAwareRoutePlanning/DataStructure/include")
set(PLANNER_OTHER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../HumanAwareRoutePlanning/Other/include")
set(PLANNER_CONSTS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../HumanAwareRoutePlanning/Consts/include")

set(DATASTRUCTURE_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../HumanAwareRoutePlanning/DataStructure/lib")
set(UTILITY_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../HumanAwareRoutePlanning/Other/lib")

# Set library output directory
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${LIB_DIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${LIB_DIR}")

# Set include directories
include_directories(
		"${INC_DIR}"
		"${PLANNER_DIR}"
		"${PLANNER_DATASTRUCTURE_DIR}"
		"${PLANNER_OTHER_DIR}"
		"${PLANNER_CONSTS_DIR}"
)

# Set link directories
link_directories("${UTILITY_LIB_DIR}")
link_directories("${DATASTRUCTURE_LIB_DIR}")

# Create library
add_library(
	"${PRJ_NAME}"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/HumanIntentionRecognition.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/include/HumanIntentionRecognition.h"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/HumanIntentionRecognizer.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/include/HumanIntentionRecognizer.h"
	"${CMAKE_CURRENT_SOURCE_DIR}/include/HIRutils.h"	
    "${CMAKE_CURRENT_SOURCE_DIR}/include/TrackedHuman.h"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/HIRutils.cpp"
)


# link library
target_link_libraries("${PRJ_NAME}" HumanAwarePlanning)

target_include_directories("${PRJ_NAME}" PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

