# Copyright (c) Orbbec Inc. All Rights Reserved.
# Licensed under the MIT License.

cmake_minimum_required(VERSION 3.10)
project(ob_femto_proxy)

find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
find_package(ZLIB REQUIRED)

add_executable(${PROJECT_NAME} femto_proxy.cpp)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)

target_include_directories(${PROJECT_NAME} PRIVATE ${ZMQ_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
    ob::${OB_SDK_LIB_NAME}
    Threads::Threads
    ${ZMQ_LIBRARIES}
    ZLIB::ZLIB
)

if(APPLE)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        INSTALL_RPATH "@loader_path;@loader_path/../lib"
    )
endif()

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
