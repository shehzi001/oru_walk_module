cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)
project ( oru_walk )

include("${CMAKE_CURRENT_SOURCE_DIR}/bootstrap.cmake")

####################################
# Settings
####################################
set (CMAKE_SKIP_ASSEMBLY_SOURCE_RULES ON)
set (CMAKE_SKIP_PREPROCESSED_SOURCE_RULES ON)
set (CMAKE_VERBOSE_MAKEFILE ON) # print full commands during compilation
set (CMAKE_RULE_MESSAGES OFF CACHE BOOL "")

set(VREP_DIR "." CACHE PATH "v-rep directory")

set(CMAKE_C_FLAGS "-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -Wno-write-strings")

find_package(Boost REQUIRED system thread python)
####################################
# Other parameters
####################################

# set path to directories containing headers
find_path(EIGEN_INCLUDE_DIRS "Eigen/Core" PATHS "/usr/include/" PATH_SUFFIXES "eigen3")
if (EIGEN_INCLUDE_DIRS)
    include_directories ("${EIGEN_INCLUDE_DIRS}")
endif (EIGEN_INCLUDE_DIRS)

include_directories(oru_walk "${PROJECT_SOURCE_DIR}/smpc_solver/include")
include_directories(oru_walk "${PROJECT_SOURCE_DIR}/nao_igm/include")
include_directories(oru_walk "${PROJECT_SOURCE_DIR}/nao_vrep/include")

INCLUDE_DIRECTORIES(
    ${Boost_INCLUDE_DIR}
)

include_directories (
    /usr/local/include
    ${VREP_DIR}/programming/common
    ${VREP_DIR}/programming/include
    ${VREP_DIR}/programming/remoteApi
)

link_directories(
    ${VREP_DIR}/programming/v_repExtRemoteApi/lib
)

file (GLOB ORU_WALK_SRC "${PROJECT_SOURCE_DIR}/src/*.cpp")

add_library(oru_walk 
    ${ORU_WALK_SRC}
)


# link to libraries
target_link_libraries(oru_walk
  ${Boost_LIBRARIES}
  "${PROJECT_SOURCE_DIR}/smpc_solver/lib/libwmg.a" 
  "${PROJECT_SOURCE_DIR}/smpc_solver/lib/libsmpc_solver.a" 
  "${PROJECT_SOURCE_DIR}/nao_igm/lib/libnaoigm.a"
  "${PROJECT_SOURCE_DIR}/nao_vrep/lib/libnao_vrep.a"
)

add_executable (example 
  src/example_nao/example.cpp 
) 

target_link_libraries(example 
  oru_walk
)
