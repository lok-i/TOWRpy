cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)

project(towr_pybullet)
find_package(ifopt)
find_package(PythonLibs 2.7)
find_package(raisimOgre CONFIG REQUIRED)
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})

include_directories (${EIGEN3_INCLUDE_DIRS})
find_package(towr 1.2 REQUIRED)



function(create_sharedlib lib_name file_name)
  

  ADD_LIBRARY(${lib_name} SHARED ${file_name})

  target_include_directories(${lib_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIRS})
  target_include_directories(${lib_name} PUBLIC ${PYTHON_INCLUDE_DIRS})

  target_link_libraries(${lib_name} PUBLIC ${PYTHON_LIBRARIES})
  target_link_libraries(${lib_name} PUBLIC raisim::raisimOgre)
  target_link_libraries(${lib_name} PUBLIC ifopt::ifopt_ipopt) 
  target_link_libraries(${lib_name} PUBLIC towr::towr) # adds include directories and libraries

  target_compile_definitions(${lib_name} PRIVATE "-DEXAMPLE_ROBOT_RESOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/rsc/")

endfunction()






create_sharedlib(towr_anymal_dll towr_anymal_dll.cpp)