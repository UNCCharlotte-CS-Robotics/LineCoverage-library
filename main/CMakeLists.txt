cmake_minimum_required(VERSION 3.12)

project(lclibrary_main VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GNUInstallDirs)

find_package(yaml-cpp REQUIRED)
find_package(lclibrary REQUIRED)

if(LCLIBRARY_USE_GLPK)
	add_executable(slc ${CMAKE_CURRENT_SOURCE_DIR}/slc.cc)
	target_link_libraries(slc PUBLIC lclibrary glpk yaml-cpp)
endif()
if(LCLIBRARY_USE_GUROBI)
	link_directories($ENV{GUROBI_HOME}/lib)
	target_include_directories(slc PRIVATE $ENV{GUROBI_HOME}/include/)
endif()
	install(TARGETS slc DESTINATION ${CMAKE_INSTALL_BINDIR}/)

add_executable(mlc ${CMAKE_CURRENT_SOURCE_DIR}/mlc.cc)
target_link_libraries(mlc PUBLIC lclibrary yaml-cpp)
if(LCLIBRARY_USE_GUROBI)
	target_include_directories(mlc PRIVATE $ENV{GUROBI_HOME}/include/)
endif()
install(TARGETS mlc DESTINATION ${CMAKE_INSTALL_BINDIR}/)
