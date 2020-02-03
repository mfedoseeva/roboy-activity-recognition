project(opticalflow)

#**************************************************************************************************
# General cMake settings
#**************************************************************************************************
cmake_minimum_required(VERSION 3.5)

#**************************************************************************************************
# Find Package **************************************************************************************************
find_package(OpenCV REQUIRED)
MESSAGE( " *** OpenCV_INCLUDE_DIRS : " ${OpenCV_INCLUDE_DIRS} )
MESSAGE( " *** OpenCV_LIB_DIRS : " ${OpenCV_LIB_DIRS} )
MESSAGE( " *** OpenCV_LIBS : " ${OpenCV_LIBS} )

# find_package(PythonLibs 3 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development NumPy)
MESSAGE( " *** PYTHON_INCLUDE_DIRS : " ${Python3_INCLUDE_DIRS} )
MESSAGE( " *** PYTHON_LIBRARIES : " ${Python3_LIBRARIES} )

MESSAGE( " *** NUMPY : " ${Python3_NumPy_FOUND} )
MESSAGE( " *** PYTHON3 : " ${Python3_NumPy_VERSION} )

find_package(pybind11 CONFIG REQUIRED)

#**************************************************************************************************
# Include **************************************************************************************************
include_directories(${Python3_INCLUDE_DIRS})
include_directories(${Python3_NumPy_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${pybind11_INCLUDE_DIR})

#**************************************************************************************************
# Set variable **************************************************************************************************
SET(SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/opticalflow.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ndarray_converter.cpp
)

#**************************************************************************************************
# Set compiler **************************************************************************************************
SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall -O3 -fPIC")

#**************************************************************************************************
# Linker **************************************************************************************************
LINK_DIRECTORIES(
  ${OpenCV_LIB_DIR}
)

#**************************************************************************************************
# Make configuration
#**************************************************************************************************
add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_link_libraries(${PROJECT_NAME} Python3::NumPy ${OpenCV_LIBS} )
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES PREFIX "")

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/opticalflow)
