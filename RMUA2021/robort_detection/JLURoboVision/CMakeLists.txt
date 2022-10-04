cmake_minimum_required(VERSION 3.5)

project(JLURoboVision)

add_definitions(-std=c++11)

#option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

# include dir of GxCamera
include_directories(${PROJECT_SOURCE_DIR}/JLURoboVision/AngleSolver/include)
include_directories(${PROJECT_SOURCE_DIR}/JLURoboVision/Armor/include)
include_directories(${PROJECT_SOURCE_DIR}/JLURoboVision/General/include)
include_directories(${PROJECT_SOURCE_DIR}/JLURoboVision/GxCamera/include)
include_directories(${PROJECT_SOURCE_DIR}/JLURoboVision/GxCamera/GxSDK/include)
include_directories(${PROJECT_SOURCE_DIR}/JLURoboVision/Wind/include)
include_directories(${PROJECT_SOURCE_DIR}/JLURoboVision/Serial/include)
include_directories(${PROJECT_SOURCE_DIR}/JLURoboVision/Can/include)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Ofast -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")

find_package(OpenCV)
include_directories(OpenCV_INCLUDE_DIRS)

file(GLOB JLU_VISION ${PROJECT_SOURCE_DIR}/JLURoboVision/Main/ArmorDetecting.cpp ${PROJECT_SOURCE_DIR}/JLURoboVision/Main/main.cpp ${PROJECT_SOURCE_DIR}/JLURoboVision/Main/ImageUpdating.cpp ${PROJECT_SOURCE_DIR}/JLURoboVision/GxCamera/GxCamera.cpp ${PROJECT_SOURCE_DIR}/JLURoboVision/Wind/*.cpp ${PROJECT_SOURCE_DIR}/JLURoboVision/AngleSolver/AngleSolver.cpp ${PROJECT_SOURCE_DIR}/JLURoboVision/Armor/*.cpp ${PROJECT_SOURCE_DIR}/JLURoboVision/Can/Can.cpp ${PROJECT_SOURCE_DIR}/JLURoboVision/Serial/Serial.cpp ${PROJECT_SOURCE_DIR}/JLURoboVision/Serial/predict.cpp ${PROJECT_SOURCE_DIR}/JLURoboVision/Main/filter.cpp ${PROJECT_SOURCE_DIR}/JLURoboVision/Main/SerialThread.cpp)

add_executable(JLURoboVision ${JLU_VISION})

#target_link_libraries(yolov5 nvinfer)
#target_link_libraries(yolov5 cudart)
#target_link_libraries(yolov5 myplugins)
target_link_libraries(JLURoboVision ${OpenCV_LIBS})
#pthread GxCamera X11
#target_link_libraries(JLURoboVision -lpthread -lm -lgxiapi -lX11)
target_link_libraries(JLURoboVision -lpthread -lm -lgxiapi)

add_definitions(-O2 -pthread)

