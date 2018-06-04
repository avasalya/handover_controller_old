set(FSM_STATES_INSTALL_PREFIX "${MC_RTC_LIBDIR}/mc_controller/mc_handover_controller/states")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../src)

macro(add_fsm_state state_name state_SRC state_HDR)
  add_library(${state_name} SHARED  ${state_SRC} ${state_HDR})
  set_target_properties(${state_name} PROPERTIES PREFIX "")
  pkg_config_use_dependency(${state_name} mc_rtc)
  target_link_libraries(${state_name} mc_handover_controller)
  install(TARGETS ${state_name} DESTINATION ${FSM_STATES_INSTALL_PREFIX})
  install(FILES ${state_HDR} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/mc_control/fsm/states)
endmacro()


macro(add_fsm_handover_state state_name)
  add_fsm_state(${state_name} ${state_name}.cpp ${state_name}.h)
endmacro()


# INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}/usr/include/Eigen3" )

add_fsm_handover_state(GoInitPoseStep)
add_fsm_handover_state(OpenGrippersStep)
add_fsm_handover_state(ApproachObjectStep)
add_fsm_handover_state(CloseGrippersStep)
# add_fsm_handover_state(GoDummyPoseStep)
add_fsm_handover_state(ForceControlGrippersStep)
add_fsm_handover_state(GoHalfSittingStep)
