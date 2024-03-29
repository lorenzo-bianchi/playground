cmake_minimum_required(VERSION 3.5)

project(toppra
  VERSION 0.6.2
  # Disable because not available in CMake 3.5
  #DESCRIPTION "Library computing the time-optimal path parameterization."
  LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
# Tests
option(BUILD_TESTS "Build Gtests" OFF)
# Dynamics
option(BUILD_WITH_PINOCCHIO "Compile with Pinocchio library" OFF)
# Solvers
option(BUILD_WITH_qpOASES "Compile the wrapper for qpOASES" OFF)
option(BUILD_WITH_GLPK "Compile the wrapper for GLPK (GPL license)" OFF)
# General options
option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(TOPPRA_DEBUG_ON "Set logging mode to on" OFF)
option(TOPPRA_WARN_ON "Enable warning messages" ON)
option(OPT_MSGPACK "Serialization using msgpack" OFF)
# Bindings
option(PYTHON_BINDINGS "Build bindings for Python" ON)
set(PYTHON_VERSION 3.7 CACHE STRING "Build bindings for Python version")
set(BUILD_WITH_PINOCCHIO_PYTHON false CACHE BOOL "Force compile with Pinocchio bindings")
# This option is needed because sometime the flag
# PINOCCHIO_WITH_PYTHON_INTERFACE is not properly set depsite a python
# binding is installed.
# End of options


set(PYBIND11_PYTHON_VERSION "${PYTHON_VERSION}")
include(CMakePrintHelpers)

find_package (Threads)
find_package (Eigen3 REQUIRED)
message(STATUS "Found Eigen version ${EIGEN3_VERSION_STRING}")

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
if(BUILD_WITH_PINOCCHIO)
    find_package(pinocchio REQUIRED)
    message(STATUS "Found pinocchio ${pinocchio_VERSION}")
endif()
if(BUILD_WITH_qpOASES)
    find_package(qpOASES REQUIRED)
    message(STATUS "Found qpOASES")
endif()
if(BUILD_WITH_GLPK)
    find_package(GLPK REQUIRED)
    message(STATUS "Found glpk ${GLPK_LIBRARY} ${GLPK_INCLUDE_DIR}")
endif(BUILD_WITH_GLPK)

add_subdirectory(src)
add_subdirectory(doc)
add_subdirectory(bindings)

# if(BUILD_TESTS)

#   # Download and unpack googletest at configure time
#   # source: https://github.com/google/googletest/tree/master/googletest
#   configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
#   execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
#     RESULT_VARIABLE result
#     WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
#   if(result)
#     message(FATAL_ERROR "CMake step for googletest failed: ${result}")
#   endif()
#   execute_process(COMMAND ${CMAKE_COMMAND} --build .
#     RESULT_VARIABLE result
#     WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
#   if(result)
#     message(FATAL_ERROR "Build step for googletest failed: ${result}")
#   endif()

#   # Prevent overriding the parent project's compiler/linker
#   # settings on Windows
#   set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

#   # Add googletest directly to our build. This defines
#   # the gtest and gtest_main targets.
#   add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/googletest-src
#                   ${CMAKE_CURRENT_BINARY_DIR}/googletest-build
#                   EXCLUDE_FROM_ALL)

#   # The gtest/gtest_main targets carry header search path
#   # dependencies automatically when using CMake 2.8.11 or
#   # later. Otherwise we have to add them here ourselves.
#   if (CMAKE_VERSION VERSION_LESS 2.8.11)
#     include_directories("${gtest_SOURCE_DIR}/include")
#   endif()

#   enable_testing()
#   add_subdirectory(tests)

# endif()

# Generate and install CMake config, version and target files.
include(CMakePackageConfigHelpers)
set(CONFIG_INSTALL_DIR "lib/cmake/${PROJECT_NAME}")
set(GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(VERSION_CONFIG "${GENERATED_DIR}/${PROJECT_NAME}ConfigVersion.cmake")
set(PROJECT_CONFIG "${GENERATED_DIR}/${PROJECT_NAME}Config.cmake")
write_basic_package_version_file(
    "${VERSION_CONFIG}" VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    "cmake/Config.cmake.in"
    "${PROJECT_CONFIG}"
    INSTALL_DESTINATION "${CONFIG_INSTALL_DIR}"
)

install(FILES "${PROJECT_CONFIG}" "${VERSION_CONFIG}"
    DESTINATION "${CONFIG_INSTALL_DIR}")
install(EXPORT toppra::toppra
  NAMESPACE toppra::
  FILE toppraTargets.cmake
  DESTINATION "${CONFIG_INSTALL_DIR}")
