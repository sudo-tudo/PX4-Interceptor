cmake_minimum_required(VERSION 3.10)

project(InterceptorVision LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(AIRLIB_PATH "/mnt/e/CodexLabs/Colosseum")
set(AIRLIB_INCLUDE_DIRS
    ${AIRLIB_PATH}/AirLib/include
    ${AIRLIB_PATH}/AirLib/deps/rpclib/include
    ${AIRLIB_PATH}/AirLib/deps/mavlinkcom/include
    ${AIRLIB_PATH}/AirLib/deps/eigen3
)
set(AIRLIB_LIBRARIES
    ${AIRLIB_PATH}/AirLib/lib/x64/release/libAirLib.a
    ${AIRLIB_PATH}/AirLib/deps/rpclib/lib/librpc.a
)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

add_executable(interceptor_vision src/main.cpp)
target_compile_features(interceptor_vision PRIVATE cxx_std_17)
target_include_directories(interceptor_vision PRIVATE ${AIRLIB_INCLUDE_DIRS})
target_link_libraries(interceptor_vision PRIVATE ${OpenCV_LIBS} ${AIRLIB_LIBRARIES} Threads::Threads)
target_compile_options(interceptor_vision PRIVATE "$<$<PLATFORM_ID:Linux>:-stdlib=libc++>")
target_link_options(interceptor_vision PRIVATE "$<$<PLATFORM_ID:Linux>:-stdlib=libc++>")
