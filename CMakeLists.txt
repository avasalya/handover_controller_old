cmake_minimum_required(VERSION 2.8)

set(CXX_DISABLE_WERROR 1)
include(cmake/base.cmake)
include(cmake/boost.cmake)

set(PROJECT_NAME mc_handover_controller)
set(PROJECT_DESCRIPTION "human-robot tool handover controller for mc_rtc")
set(PROJECT_URL https://gite.lirmm.fr/ashesh/mc_handover_controller)

setup_project()

# Enable C++11
if(NOT WIN32)
  add_definitions(-std=c++0x)
endif()

# Use jrl-cmakemoduels to get mc_rtc depency
set(PKG_CONFIG_ADDITIONAL_VARIABLES "${PKG_CONFIG_ADDITIONAL_VARIABLES} libdir")
add_required_dependency(mc_rtc)

add_subdirectory(src)
add_subdirectory(states)

configure_file(etc/mc_rtc.conf ${CMAKE_CURRENT_BINARY_DIR}/etc/mc_rtc.conf)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/etc/mc_rtc.conf
        DESTINATION ${CMAKE_INSTALL_PREFIX}/etc)

install(FILES etc/Handover.conf
    DESTINATION ${MC_RTC_LIBDIR}/mc_controller/etc/)

setup_project_finalize()
