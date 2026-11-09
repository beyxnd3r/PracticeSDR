cmake_minimum_required(VERSION 3.16)
project(PlutoSDR C CXX)

set(CMAKE_BUILD_TYPE Debug)

find_package(SoapySDR REQUIRED)

set(MAIN_SOURCE_FILES
    src/main.cpp
)

add_executable(main ${MAIN_SOURCE_FILES})

if(TARGET SoapySDR::SoapySDR)
    target_link_libraries(main PRIVATE SoapySDR::SoapySDR)
else()
    target_link_libraries(main PRIVATE ${SoapySDR_LIBRARIES})
endif()