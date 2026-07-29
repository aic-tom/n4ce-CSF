cmake_minimum_required(VERSION 3.13)
project(CSF LANGUAGES CXX)

# Library name suffixes per configuration
set(CMAKE_DEBUG_POSTFIX d)
set(CMAKE_RELWITHDEBINFO_POSTFIX rd)

if(MSVC)
    set(OpenMP_CXX_FLAGS "/openmp:llvm" CACHE STRING "" FORCE)
endif()
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    add_compile_definitions("CSF_USE_OPENMP")
endif()

add_subdirectory(src)

option(BUILD_DEMO "Build csfdemo executable" OFF)
set(BUILD_SHARED_LIBS "Build as shared library" OFF)

if(BUILD_DEMO)
  add_subdirectory(CSFDemo)
endif()
