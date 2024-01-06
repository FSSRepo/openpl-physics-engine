
cmake_minimum_required(VERSION 3.0)
project("open-pysics-library")

option(BUILD_SHARED_LIBS             "pl: build shared libs" OFF)
option(PL_JAVA                       "sd: build java bindings" OFF)

if(PL_JAVA)
    set(PL_SOURCES_EXTRAS java-bindings/wrapper.cpp)
    set(PL_HEADER_EXTRAS
        $ENV{JAVA_HOME}/include
        $ENV{JAVA_HOME}/include/win32)
    set(BUILD_SHARED_LIBS ON)
endif()

set(PL_LIB openpl)

include_directories(src bullet ${PL_HEADER_EXTRAS})

add_library(${PL_LIB} src/PL/BulletContext.cpp src/PL/pl.cpp src/PL/pl10.h ${PL_SOURCES_EXTRAS})

if(BUILD_SHARED_LIBS)
    message("Build shared library")
    if(NOT PL_JAVA)
        add_definitions(-DPL_BUILD_SHARED_LIB)
        target_compile_definitions(${PL_LIB} PRIVATE -DPL_BUILD_DLL)
    endif()
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
else()
    message("Build static library")
endif()

add_subdirectory(bullet)

target_link_libraries(${PL_LIB} PUBLIC bullet-lib)
target_compile_features(${PL_LIB} PUBLIC cxx_std_11)