cmake_minimum_required(VERSION 2.8.3)

# handling lesser cmake versions
cmake_policy(SET CMP0017 NEW)
if(NOT ${CMAKE_VERSION} VERSION_LESS 3.0.0)
  cmake_policy(SET CMP0042 NEW)
endif()
project(pyre)

set(LIBRARY_NAME ${PROJECT_NAME})

# use our cmake modules
set(CMAKE_MODULE_PATH
  "${CMAKE_MODULE_PATH}"
  "${CMAKE_ROOT_DIR}/cmake/Modules"
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

# setting up compile settings (c++11 by default)
include(CompileOptions)
include(FeatureSummary)
include(HelperFunctions)

message(STATUS "Building ${CMAKE_BUILD_TYPE}")

list(APPEND CATKIN_SOURCES
  robowflex_ompl
  robowflex_library
  )

find_package(catkin REQUIRED COMPONENTS ${CATKIN_SOURCES})

catkin_package(
    LIBRARIES ${LIBRARY_NAME} 
    CATKIN_DEPENDS ${CATKIN_SOURCES}
  DEPENDS
  INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
  )

# add all the include directories
list(APPEND INCLUDES
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${catkin_INCLUDE_DIRS}
  )

include_directories(${INCLUDES})
link_directories(${catkin_LIBRARY_DIRS})

# add all source files for your library here
list(APPEND SOURCES
   src/biased_sampler.cpp
   src/pyre.cpp
   src/yaml.cpp
   src/spark.cpp
   src/flame.cpp
   src/math.cpp
   src/database.cpp
   src/eplanner.cpp
   src/distances.cpp
   )

# add all the libraries to link against here 
list(APPEND LIBRARIES
  ${catkin_LIBRARIES}
  yaml-cpp
  )

# create library with all source files
add_library(${LIBRARY_NAME} ${SOURCES})
set_target_properties(${LIBRARY_NAME} PROPERTIES VERSION 0.0.1)
target_link_libraries(${LIBRARY_NAME} ${LIBRARIES})

# add executable files
add_script(process) 
add_script(merge) 
add_script(benchmark) 
add_script(visualize_db) 

# print out cmake summary
feature_summary(DESCRIPTION INCLUDE_QUIET_PACKAGES WHAT ALL)
