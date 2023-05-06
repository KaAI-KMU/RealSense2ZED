cmake_minimum_required(VERSION 3.5)

project(stereo-calibration LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()

find_package(ZED 3 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED )

include_directories(include
    ${OpenCV_INCLUDE_DIRS}
    ${realsense_INCLUDE_DIR}
    ${ZED_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
)

link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}
    main.cpp
    zedcamera.h zedcamera.cpp
    realsensecamera.h realsensecamera.cpp
)

if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${OpenCV_LIBRARY_DIRS})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY} ${OpenCV_LIBRARY_DIRS})
endif()

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${realsense2_LIBRARY} ${ZED_LIBS})

install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
