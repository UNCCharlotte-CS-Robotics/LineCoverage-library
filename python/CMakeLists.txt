cmake_minimum_required(VERSION 3.12)

project(py_lclibrary VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GNUInstallDirs)

find_package(Python COMPONENTS Interpreter Development REQUIRED)

message("Installing for python version: ${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}")

find_package(lclibrary REQUIRED)
find_package(Boost COMPONENTS system python numpy REQUIRED)

Python_add_library(py_lclibrary ${PROJECT_SOURCE_DIR}/src/slc.cc)

target_compile_options(py_lclibrary PRIVATE -O3 -Wall -pedantic)

target_link_libraries(py_lclibrary PUBLIC lclibrary ${Boost_LIBRARIES} ${Python_LIRARIES} glpk)
target_include_directories(py_lclibrary PUBLIC ${Boost_INCLUDE_DIRS} ${Python_INCLUDE_DIRS})

install(TARGETS py_lclibrary LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

install (FILES ${PROJECT_SOURCE_DIR}/rpp_3by2.py
	DESTINATION "${CMAKE_INSTALL_BINDIR}" PERMISSIONS
	OWNER_WRITE OWNER_READ OWNER_EXECUTE
	GROUP_WRITE GROUP_READ GROUP_EXECUTE)
