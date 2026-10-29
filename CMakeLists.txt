cmake_minimum_required(VERSION 3.10)

project(InterceptorVision LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    message(FATAL_ERROR
        "This project must be configured with clang++ on Linux because the prebuilt "
        "AirLib/rpclib libraries use libc++. Reconfigure with: "
        "cmake -S . -B build -DCMAKE_CXX_COMPILER=clang++"
    )
endif()

set(AIRLIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/AirLib")
set(AIRLIB_INCLUDE_DIRS
    ${AIRLIB_PATH}/include
    ${AIRLIB_PATH}/deps/rpclib/include
    ${AIRLIB_PATH}/deps/MavLinkCom/include
    ${AIRLIB_PATH}/deps/eigen3
)
set(AIRLIB_LIBRARIES
    ${AIRLIB_PATH}/lib/libAirLib.a
    ${AIRLIB_PATH}/deps/rpclib/lib/librpc.a
)

# Use libc++-compatible OpenCV built from source.
# Override with: cmake -DOpenCV_DIR=/path/to/opencv-libc++/lib/cmake/opencv4
if(NOT DEFINED OpenCV_DIR)
    set(OpenCV_DIR "/home/kira/opencv-libc++/lib/cmake/opencv4")
endif()
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

file(GLOB INTERCEPTOR_SOURCES "src/*.cpp")
add_executable(interceptor_vision ${INTERCEPTOR_SOURCES})
target_compile_features(interceptor_vision PRIVATE cxx_std_17)
target_include_directories(interceptor_vision PRIVATE ${AIRLIB_INCLUDE_DIRS})
target_link_libraries(interceptor_vision PRIVATE ${OpenCV_LIBS} ${AIRLIB_LIBRARIES} Threads::Threads)
target_compile_options(interceptor_vision PRIVATE "$<$<PLATFORM_ID:Linux>:-stdlib=libc++>")
target_link_options(interceptor_vision PRIVATE "$<$<PLATFORM_ID:Linux>:-stdlib=libc++>")
