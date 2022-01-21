cmake_minimum_required(VERSION 3.12)

project(lclibrary VERSION 1.0)

include(CheckCXXCompilerFlag)
include(GNUInstallDirs)
# include(InstallRequiredSystemLibraries)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(LCLIBRARY_USE_GUROBI
	"Use GUROBI to solve LP and ILP. GUROBI needs to be configured and licensed.
	See installation and terms of use at <https://www.gurobi.com/>"
	OFF)

option(LCLIBRARY_USE_GLPK
	"Use GLPK to solve LP and ILP. GLPK needs to be configured and is available under GPL license.
	See <https://www.gnu.org/software/glpk/>"
	ON)

option(LCLIBRARY_USE_LKH
	"Use LKH to solve large ATSP instances.
	See the terms of use at <http://webhotel4.ruc.dk/~keld/research/LKH/>"
	OFF)

option(LCLIBRARY_USE_GLKH
	"Use GLKH to solve GTSP.
	See the terms of use at <http://webhotel4.ruc.dk/~keld/research/GLKH/>"
	OFF)

if(LCLIBRARY_USE_GUROBI)
	set(GUROBI_INCLUDE_DIR $ENV{GUROBI_HOME}/include/)
	set(GUROBI_LIB $ENV{GUROBI_LIB})
endif()

set(LCLIBRARY_EXTERNAL_JSON_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/external/json/include/)
set(LCLIBRARY_EXTERNAL_MCPM_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/external/mcpm/include/)

option(LCLIBRARY_BUILD_SHARED_LIBS "Build lclibrary shared library" ${BUILD_SHARED_LIBS})

set(build-shared $<BOOL:${LCLIBRARY_BUILD_SHARED_LIBS}>)
if (LCLIBRARY_BUILD_SHARED_LIBS)
	set(lclibrary-type SHARED)
	set(lclibrary-label-postfix "shared")
else()
	set(lclibrary-type STATIC)
	set(lclibrary-label-postfix "static")
endif()

set(lclibrary-src-core-files
	adjacency_list_generation.cc
	graph.cc
	graph_file_parser.cc
	graph_io.cc
	graph_utilities.cc)

set(lclibrary-src-utils-files
	plot_graph.cc
	video_generator.cc)

list(TRANSFORM lclibrary-src-core-files PREPEND "${PROJECT_SOURCE_DIR}/src/core/")
list(TRANSFORM lclibrary-src-utils-files PREPEND "${PROJECT_SOURCE_DIR}/src/utils/")

if(LCLIBRARY_USE_GLPK)
	set(lclibrary-src-slc-files
		ilp_glpk.cc
		lp_beta3_glpk.cc
		lp_glpk.cc)
	list(TRANSFORM lclibrary-src-slc-files PREPEND "${PROJECT_SOURCE_DIR}/src/slc/")
endif()

if(LCLIBRARY_USE_GLPK)
	set(lclibrary-sources
		${lclibrary-src-core-files}
		${lclibrary-src-slc-files}
		${lclibrary-src-utils-files})
else()
	set(lclibrary-sources
		${lclibrary-src-core-files}
		${lclibrary-src-utils-files})
endif()

add_library(lclibrary ${lclibrary-type} "")

target_compile_options(lclibrary PRIVATE
	-O2 -fPIC
	-Wall -pedantic)
# -Wextra -Wshadow -Weffc++ -Wno-long-long
# -pedantic -pedantic-errors)

target_include_directories(lclibrary PUBLIC
	$<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/>
	$<BUILD_INTERFACE:${LCLIBRARY_EXTERNAL_JSON_INCLUDE_DIR}>
	$<BUILD_INTERFACE:${LCLIBRARY_EXTERNAL_MCPM_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

if(LCLIBRARY_USE_GUROBI)
	target_include_directories(lclibrary PUBLIC
		$<BUILD_INTERFACE:${GUROBI_INCLUDE_DIR}>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
	target_link_libraries(lclibrary INTERFACE stdc++fs m gurobi_c++ ${GUROBI_LIB})
else()
	target_link_libraries(lclibrary INTERFACE stdc++fs m)
endif()


target_sources(lclibrary
	PRIVATE
	${lclibrary-sources})

set_target_properties(lclibrary PROPERTIES
	VERSION "${PROJECT_VERSION}"
	SOVERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}"
	PROJECT_LABEL "lclibrary ${lclibrary-label-postfix}")

# message(STATUS "CMAKE_INSTALL_INCLUDEDIR: ${CMAKE_INSTALL_INCLUDEDIR}")
install(TARGETS lclibrary
	EXPORT lclibrary-targets
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(DIRECTORY
	${PROJECT_SOURCE_DIR}/include/
	${LCLIBRARY_EXTERNAL_JSON_INCLUDE_DIR}
	${LCLIBRARY_EXTERNAL_MCPM_INCLUDE_DIR}
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(FILES "${PROJECT_BINARY_DIR}/lclibrary_config.h" DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY "${PROJECT_SOURCE_DIR}/config" DESTINATION ${CMAKE_INSTALL_BINDIR}/)
install(TARGETS lclibrary LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(EXPORT lclibrary-targets
	FILE lclibrary-targets.cmake
	DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/lclibrary")

include(CMakePackageConfigHelpers)

# generate the config file that is includes the exports
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/lclibrary-config.cmake.in
	"${CMAKE_CURRENT_BINARY_DIR}/lclibrary-config.cmake"
	INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/lclibrary")

write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/lclibrary-config-version.cmake"
	VERSION "${lclibrary_VERSION_MAJOR}.${lclibrary_VERSION_MINOR}"
	COMPATIBILITY AnyNewerVersion)

install(FILES
	${CMAKE_CURRENT_BINARY_DIR}/lclibrary-config.cmake
	${CMAKE_CURRENT_BINARY_DIR}/lclibrary-config-version.cmake
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/lclibrary)

export(EXPORT lclibrary-targets
	FILE "${CMAKE_CURRENT_BINARY_DIR}/lclibrary-targets.cmake")

if(LCLIBRARY_USE_GLPK)
	add_executable(slc ${PROJECT_SOURCE_DIR}/main/slc.cc)
	install(TARGETS slc DESTINATION ${CMAKE_INSTALL_BINDIR}/)
	target_link_libraries(slc PUBLIC lclibrary glpk yaml-cpp)
endif()

add_executable(mlc ${PROJECT_SOURCE_DIR}/main/mlc.cc)
install(TARGETS mlc DESTINATION ${CMAKE_INSTALL_BINDIR}/)
target_link_libraries(mlc PUBLIC lclibrary yaml-cpp)

if(LCLIBRARY_USE_LKH)
	set(LCLIBRARY_EXTERNAL_LKH_SOURCE_DIR ${PROJECT_SOURCE_DIR}/external/lkh/)
	set(lkh-src-pattern "${LCLIBRARY_EXTERNAL_LKH_SOURCE_DIR}/SRC/*.c")
	file(GLOB lkh-sources ${lkh-src-pattern})
	add_executable(LKH ${lkh-sources})
	target_include_directories(LKH PRIVATE "${LCLIBRARY_EXTERNAL_LKH_SOURCE_DIR}/SRC/INCLUDE/")
	target_compile_options(LKH PRIVATE -O3 -D$(TWO_LEVEL_TREE))
	target_link_libraries(LKH PRIVATE m)
	install(TARGETS LKH DESTINATION ${CMAKE_INSTALL_BINDIR}/)
else()
	set(LCLIBRARY_USE_GLKH OFF)
endif()

if(LCLIBRARY_USE_GLKH)
	set(LCLIBRARY_EXTERNAL_GLKH_SOURCE_DIR ${PROJECT_SOURCE_DIR}/external/glkh/)
	set(glkh-src-pattern "${LCLIBRARY_EXTERNAL_GLKH_SOURCE_DIR}/SRC/*.c")
	file(GLOB glkh-sources ${glkh-src-pattern})

	add_executable(GLKH ${glkh-sources})
	target_include_directories(GLKH PRIVATE "${LCLIBRARY_EXTERNAL_GLKH_SOURCE_DIR}/SRC/INCLUDE/")
	target_compile_options(GLKH PRIVATE -O3)
	target_link_libraries(GLKH PRIVATE m)
	install(TARGETS GLKH DESTINATION ${CMAKE_INSTALL_BINDIR}/glkh/)
	install(TARGETS LKH DESTINATION ${CMAKE_INSTALL_BINDIR}/glkh/)
	install (DIRECTORY  DESTINATION "${CMAKE_INSTALL_BINDIR}/glkh/TMP/" DIRECTORY_PERMISSIONS
		OWNER_WRITE OWNER_READ OWNER_EXECUTE
		GROUP_WRITE GROUP_READ GROUP_EXECUTE)
	install (FILES ${LCLIBRARY_EXTERNAL_GLKH_SOURCE_DIR}/glkh.sh
		DESTINATION "${CMAKE_INSTALL_BINDIR}/glkh/" PERMISSIONS
		OWNER_WRITE OWNER_READ OWNER_EXECUTE
		GROUP_WRITE GROUP_READ GROUP_EXECUTE)
	set(LCLIBRARY_GLKH_PATH "std::string(\"${CMAKE_INSTALL_FULL_BINDIR}/glkh/\")")
endif()

configure_file(lclibrary_config.h.in lclibrary_config.h)

