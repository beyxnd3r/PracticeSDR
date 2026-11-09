cmake_minimum_required(VERSION 3.16)
# Добавляем CXX (C++) к языкам проекта
project(PlutoSDR C CXX)

set(CMAKE_BUILD_TYPE Debug)

option(INSTALL_DEPS "Установить зависимости" ON)
option(PLUTO_TIMESTAMP "Build pluto with timestamp" ON)

find_package(SoapySDR REQUIRED)

set(MAIN_SOURCE_FILES
    src/main.cpp
)

add_executable(main ${MAIN_SOURCE_FILES})

# Современный и более надежный способ линковки SoapySDR
if(TARGET SoapySDR::SoapySDR)
    target_link_libraries(main PRIVATE SoapySDR::SoapySDR)
else()
    target_link_libraries(main PRIVATE ${SoapySDR_LIBRARIES})
endif()